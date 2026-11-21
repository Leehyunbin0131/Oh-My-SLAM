"""Passive path diagnostics in a separate process, with bounded asynchronous disk IO.

The recorder never publishes motion, changes parameters or owns navigation.
Original ROS messages are retained alongside derived metrics; metrics alone are
not evidence that a path loops or that the robot followed it.
"""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import zlib

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters, ListParameters
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from nav2_msgs.action import NavigateToPose, FollowPath
from nav2_msgs.msg import Costmap
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from omslam_operator_msgs.action import OperatorNavigate
from omslam_operator_msgs.msg import KeepoutZones, OperatorState


SCHEMA_VERSION = 1
MAX_TOTAL_BYTES = 2 * 1024 ** 3
MAX_QUEUE_BYTES = 16 * 1024 ** 2
MAX_GRID_CELLS = 16 * 1024 ** 2
# These plugins and IDs are the runtime contract validated by the navigation launch.
REQUIRED_PARAMETERS = {
    'planner_server': {'GridBased.plugin', 'GridBased.xy_goal_tolerance', 'GridBased.tolerance',
                       'GridBased.smooth_path', 'GridBased.lattice_filepath'},
    'controller_server': {'FollowPath.plugin', 'FollowPath.critics', 'FollowPath.max_vel_x',
                          'FollowPath.xy_goal_tolerance', 'FollowPath.trans_stopped_velocity',
                          'goal_checker.plugin', 'goal_checker.xy_goal_tolerance',
                          'goal_checker.yaw_goal_tolerance', 'goal_checker.trans_stopped_velocity',
                          'goal_checker.rot_stopped_velocity', 'goal_checker.stateful'},
    'bt_navigator': {'navigators', 'default_nav_to_pose_bt_xml'},
    'global_costmap/global_costmap': {'plugins', 'resolution', 'global_frame'},
    'local_costmap/local_costmap': {'plugins', 'resolution', 'global_frame'},
}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def plain(message):
    return message_to_ordereddict(message)


def yaw(q):
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1 - 2 * (q['y'] ** 2 + q['z'] ** 2))


def path_metrics(message, target=None):
    """Describe path geometry without classifying a loop by an arbitrary cutoff."""
    poses = message['poses']
    points = [(p['pose']['position']['x'], p['pose']['position']['y']) for p in poses]
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    chord = math.hypot(points[-1][0]-points[0][0], points[-1][1]-points[0][1]) if points else 0.
    result = {'pose_count': len(points), 'length_m': length, 'chord_m': chord,
              'length_over_chord': length / chord if chord > 1e-6 else None}
    if points:
        result.update(start_xy=list(points[0]), end_xy=list(points[-1]),
                      start_yaw=yaw(poses[0]['pose']['orientation']),
                      end_yaw=yaw(poses[-1]['pose']['orientation']))
        if target and target.get('frame_id') == message['header']['frame_id']:
            result['endpoint_goal_distance_m'] = math.hypot(
                points[-1][0]-target['x'], points[-1][1]-target['y'])
            difference = result['end_yaw'] - target['yaw']
            result['endpoint_goal_yaw_error_rad'] = math.atan2(math.sin(difference), math.cos(difference))
    return result


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class RecordWriter:
    """Bound both queued bytes and on-disk bytes, never overwrite another run."""
    def __init__(self, directory, max_bytes=MAX_TOTAL_BYTES, queue_bytes=MAX_QUEUE_BYTES):
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.limit, self.queue_limit = max_bytes, queue_bytes
        self.lock = threading.Lock()
        self.queue = queue.Queue(maxsize=2048)
        self.stop = threading.Event()
        self.error = ''
        self.received = self.written = self.dropped = self.bytes_written = self.queued_bytes = 0
        self.thread = threading.Thread(target=self._run, name='path-debug-disk', daemon=True)
        self.stream = (self.directory / 'records.jsonl').open('x', encoding='utf-8')
        self.thread.start()

    def fail(self, reason):
        with self.lock:
            if not self.error:
                self.error = str(reason)

    def submit(self, record, blob=None):
        # Encoding is in this independent recorder process, never the GUI worker.
        try:
            line = json.dumps(record, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n'
            size = len(line.encode('utf-8')) + (len(blob[1]) if blob else 0)
        except (ValueError, TypeError) as exc:
            with self.lock:
                self.last_invalid_record = str(exc)
                self.received += 1
                self.dropped += 1
            return False
        with self.lock:
            self.received += 1
            if self.error or self.stop.is_set():
                self.dropped += 1
                return False
            if self.queued_bytes + size > self.queue_limit or self.queue.full():
                self.error = 'path debug queue capacity exceeded; recording incomplete'
                self.dropped += 1
                return False
            self.queued_bytes += size
            self.queue.put_nowait((line, blob, size))
        return True

    def _run(self):
        last_flush = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    line, blob, size = self.queue.get(timeout=.1)
                except queue.Empty:
                    if time.monotonic() - last_flush >= .5:
                        self.stream.flush()
                        last_flush = time.monotonic()
                    continue
                try:
                    if self.bytes_written + size > self.limit:
                        raise OSError('path debug size limit reached; recording incomplete')
                    if blob:
                        name, content = blob
                        # Names are generated internally, never from topic/user input.
                        with (self.directory / name).open('xb') as stream:
                            stream.write(content)
                    self.stream.write(line)
                    self.bytes_written += size
                    self.written += 1
                    if time.monotonic() - last_flush >= .5:
                        self.stream.flush()
                        last_flush = time.monotonic()
                except (OSError, ValueError) as exc:
                    self.fail('path debug write failed: ' + str(exc))
                    self.dropped += 1
                finally:
                    with self.lock:
                        self.queued_bytes -= size
                    self.queue.task_done()
            self.stream.flush()
            os.fsync(self.stream.fileno())
        except (OSError, ValueError) as exc:
            self.fail('path debug flush failed: ' + str(exc))
        finally:
            try:
                self.stream.close()
            except OSError as exc:
                self.fail('path debug close failed: ' + str(exc))

    def status(self, stopped=False):
        with self.lock:
            return {'directory': str(self.directory), 'status': 'error' if self.error else
                    'stopped' if stopped else 'recording', 'error': self.error,
                    'last_invalid_record': getattr(self, 'last_invalid_record', ''),
                    'received': self.received, 'written': self.written, 'dropped': self.dropped,
                    'bytes_written': self.bytes_written, 'queued_bytes': self.queued_bytes}

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            self.fail('path debug writer did not drain within five seconds')
        return self.status(stopped=True)


class GoalCorrelation:
    """Record submitted GUI goals separately from server-side action goal IDs."""
    def __init__(self, directory):
        self.directory = str(Path(directory).resolve())
        self.active = None
        self.target = None
        self.goals = {}
        self.seen = set()
        self.seen_order = deque()

    def event(self, value):
        if not isinstance(value, dict) or value.get('directory', self.directory) != self.directory:
            return False
        event_id = value.get('event_id')
        if event_id:
            if event_id in self.seen:
                return False
            self.seen.add(event_id)
            self.seen_order.append(event_id)
            if len(self.seen_order) > 2048:
                self.seen.discard(self.seen_order.popleft())
        request = value.get('request_id')
        kind = value.get('event')
        if kind == 'goal_requested' and request:
            self.active = request
            self.target = {key: value[key] for key in ('x', 'y', 'yaw', 'frame_id') if key in value}
            if len(self.target) != 4:
                self.target = None
            self.goals[request] = {'requested': value, 'events': []}
            # Complete history stays on disk; bound the in-memory final index.
            while len(self.goals) > 128:
                del self.goals[next(iter(self.goals))]
        if request in self.goals:
            self.goals[request]['events'].append(value)
            del self.goals[request]['events'][:-16]
        # Keep the latest target after terminal status to explain lingering paths.
        # Each path additionally stores current operator state, so active movement
        # is never inferred merely from this most-recent correlation identifier.
        return True


def metadata(workspace, use_sim_time, namespace):
    from ament_index_python.packages import get_package_prefix
    root = Path(workspace).resolve()
    result = {'schema_version': SCHEMA_VERSION, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'workspace': str(root), 'pid': os.getpid(), 'use_sim_time': use_sim_time,
              'namespace': namespace, 'ros_domain_id': os.getenv('ROS_DOMAIN_ID', '0'),
              'limits': {'total_bytes': MAX_TOTAL_BYTES, 'queued_bytes': MAX_QUEUE_BYTES},
              'sampling': {'robot_pose_hz': 10., 'local_path_max_hz': 10., 'grid_max_hz': 1.},
              'scope': 'passive ROS observation; messages retain original stamps and frames',
              'installed_prefixes': {}, 'config_sha256': {}}
    for package in ('omslam_operator_support', 'omslam_operator', 'omslam_navigation', 'nav2_smac_planner'):
        try:
            prefix = get_package_prefix(package)
            result['installed_prefixes'][package] = prefix
            if package == 'omslam_navigation':
                share = Path(prefix) / 'share' / package
                for pattern in ('config/*.yaml', 'config/lattice/*.json', 'behavior_trees/*.xml'):
                    for path in share.glob(pattern):
                        result['config_sha256'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        except Exception as exc:
            result['installed_prefixes'][package] = {'error': str(exc)}
    for key, command in [('git_head', ['git', 'rev-parse', 'HEAD']),
                         ('git_status', ['git', 'status', '--short'])]:
        try:
            result[key] = subprocess.run(command, cwd=root, capture_output=True, text=True,
                                         timeout=3., check=True).stdout.strip()
        except Exception as exc:
            result[key] = {'error': str(exc)}
    return result


class PathDebugNode(Node):
    def __init__(self, writer, workspace, use_sim_time=True, namespace=''):
        super().__init__('operator_path_debug', namespace=namespace,
                         parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        self.writer = writer
        self.goals = GoalCorrelation(writer.directory)
        self.sequence = 0
        self.counts = Counter()
        self.throttled = Counter()
        self.last_samples = {}
        self.latest_pose = None
        self.latest_state = None
        self.last_clock_ns = None
        self.parameter_jobs = {}
        self.expected_topics = []
        # A console-owned simulator may still be starting when recording begins.
        self.parameter_deadline = time.monotonic() + 120.
        self.parameters_completed = set()
        self.parameters_failed = {}
        self.parameter_errors = {}
        self.parameter_session = ''
        self.tf = Buffer(cache_time=Duration(seconds=10.), node=self)
        self.tf_listener = TransformListener(self.tf, self, spin_thread=False)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        events = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.topic('/operator/path_debug/status'), transient)
        self.create_subscription(String, self.topic('/operator/path_debug/events'), self.gui_event, events)
        self.expected_topics.append('/operator/path_debug/events')
        self.subscriptions_keep = []
        for topic in ('/plan', '/unsmoothed_plan', '/received_global_plan', '/local_plan', '/transformed_global_plan'):
            self.subscribe(RosPath, topic, self.path, latest)
        for topic in ('/odom', '/wheel/odometry'):
            self.subscribe(Odometry, topic, self.message, latest)
        for topic in ('/cmd_vel/nav', '/cmd_vel/requested', '/cmd_vel/smoothed', '/cmd_vel/monitored', '/cmd_vel'):
            self.subscribe(TwistStamped, topic, self.message, latest)
        self.subscribe(OperatorState, '/operator/state', self.operator, transient)
        self.subscribe(KeepoutZones, '/operator/keepouts', self.message, transient)
        self.subscribe(String, '/navigation/state', self.message, transient)
        self.subscribe(TFMessage, '/tf', self.message, latest)
        self.subscribe(TFMessage, '/tf_static', self.message, transient)
        self.subscribe(OccupancyGrid, '/map', self.grid, transient)
        for topic in ('/global_costmap/costmap_raw', '/local_costmap/costmap_raw'):
            self.subscribe(Costmap, topic, self.grid, transient)
        for topic, action in (('/operator/navigate', OperatorNavigate),
                              ('/navigate_to_pose', NavigateToPose), ('/follow_path', FollowPath)):
            self.subscribe(action.Impl.FeedbackMessage, topic + '/_action/feedback', self.message, latest)
            self.subscribe(GoalStatusArray, topic + '/_action/status', self.message, transient)
        self.parameter_clients = {}
        for target in REQUIRED_PARAMETERS:
            name = self.topic('/' + target)
            self.parameter_clients[target] = (
                self.create_client(GetState, name + '/get_state'),
                self.create_client(ListParameters, name + '/list_parameters'),
                self.create_client(GetParameters, name + '/get_parameters'))
        steady = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
        self.create_timer(.1, self.pose, clock=steady)
        self.create_timer(1., self.housekeeping, clock=steady)
        self.record('session_start', '', {'schema_version': SCHEMA_VERSION})
        self.publish_status()
        atomic_json(writer.directory / 'ready.json', {'status': 'recording', 'pid': os.getpid(),
                                                      'directory': str(writer.directory)})

    def topic(self, value):
        return self.get_namespace().rstrip('/') + value

    def subscribe(self, kind, topic, callback, qos):
        self.expected_topics.append(topic)
        def receive(message):
            self.counts[topic] += 1
            try:
                callback(topic, message)
            except Exception as exc:
                self.record('capture_error', topic, {'error': str(exc)})
                self.writer.fail('message capture failed for ' + topic + ': ' + str(exc))
        self.subscriptions_keep.append(self.create_subscription(kind, self.topic(topic), receive, qos))

    def record(self, kind, topic, payload, blob=None):
        self.sequence += 1
        record = {'seq': self.sequence, 'kind': kind, 'topic': self.topic(topic) if topic else '',
                  'received_monotonic_ns': time.monotonic_ns(), 'received_wall_ns': time.time_ns(),
                  'ros_time_ns': self.get_clock().now().nanoseconds,
                  'latest_request_id': self.goals.active, 'payload': payload}
        return self.writer.submit(record, blob)

    def message(self, topic, message):
        self.record('message', topic, plain(message))

    def operator(self, topic, message):
        if message.session_id and message.session_id != self.parameter_session:
            for stage, future, started, names in self.parameter_jobs.values():
                future.cancel()
            self.parameter_jobs.clear()
            self.parameters_completed.clear()
            self.parameters_failed.clear()
            self.parameter_errors.clear()
            self.parameter_deadline = time.monotonic() + 120.
            self.parameter_session = message.session_id
        self.latest_state = plain(message)
        self.message(topic, message)

    def gui_event(self, message):
        try:
            event = json.loads(message.data)
            if self.goals.event(event):
                self.counts['/operator/path_debug/events'] += 1
                self.record('gui_event', '/operator/path_debug/events', event)
        except (ValueError, TypeError) as exc:
            self.record('capture_error', '/operator/path_debug/events', {'error': str(exc)})

    def path(self, topic, message):
        if topic in ('/local_plan', '/transformed_global_plan') and not self.sample(topic, .1):
            return
        if len(message.poses) > 100000:
            self.record('path_omitted', topic, {'reason': 'path exceeds 100000 pose capture limit',
                                               'poses': len(message.poses)})
            return
        value = plain(message)
        self.record('path', topic, {'message': value, 'metrics': path_metrics(value, self.goals.target),
                    'robot_pose': self.latest_pose, 'operator_state': self.latest_state})

    def sample(self, topic, interval):
        now = time.monotonic()
        if now - self.last_samples.get(topic, -1e10) < interval:
            self.throttled[topic] += 1
            return False
        self.last_samples[topic] = now
        return True

    def grid(self, topic, message):
        if not self.sample(topic, 1.):
            return
        cells = message.data
        if len(cells) > MAX_GRID_CELLS:
            self.record('grid_omitted', topic, {'reason': 'grid exceeds 16 MiB capture limit',
                                              'cells': len(cells)})
            return
        raw = cells.tobytes() if hasattr(cells, 'tobytes') else bytes(v % 256 for v in cells)
        compressed = zlib.compress(raw, level=1)
        name = 'grid-' + str(self.sequence + 1).zfill(8) + '.bin.zlib'
        metadata_value = {'header': plain(message.header),
                          'metadata': plain(message.info if isinstance(message, OccupancyGrid) else message.metadata),
                          'file': name, 'encoding': 'int8' if isinstance(message, OccupancyGrid) else 'uint8',
                          'compression': 'zlib', 'cells': len(cells),
                          'raw_sha256': hashlib.sha256(raw).hexdigest()}
        self.record('grid', topic, metadata_value, (name, compressed))

    def pose(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            self.tf.clear()
            self.latest_pose = None
            self.record('clock_reset', '', {'previous_ros_ns': self.last_clock_ns, 'ros_ns': now})
        self.last_clock_ns = now
        try:
            tr = self.tf.lookup_transform('map', 'base_footprint', Time())
            stamp = stamp_ns(tr.header.stamp)
            age = (now - stamp) / 1e9
            value = plain(tr)
            q = value['transform']['rotation']
            p = value['transform']['translation']
            self.latest_pose = {'transform': value, 'sample_ros_ns': now,
                                'age_s': age, 'fresh': -.05 <= age < .3,
                                'x': p['x'], 'y': p['y'], 'yaw': yaw(q)}
            self.record('robot_pose', '', self.latest_pose)
        except Exception as exc:
            self.latest_pose = None
            if self.sample('pose_error', 1.):
                self.record('pose_unavailable', '', {'error': str(exc)})

    def housekeeping(self):
        self.publish_status()
        for target in self.parameter_clients:
            if target not in self.parameters_completed and target not in self.parameters_failed:
                self.snapshot_parameters(target)

    def snapshot_parameters(self, target):
        now = time.monotonic()
        node = self.topic('/' + target)
        job = self.parameter_jobs.get(target)
        if now >= self.parameter_deadline:
            if job:
                job[1].cancel()
                del self.parameter_jobs[target]
            reason = self.parameter_errors.get(target, 'lifecycle or parameter services not ready')
            self.parameters_failed[target] = reason
            self.record('parameters_unavailable', '', {'node': node, 'session_id': self.parameter_session,
                                                       'reason': reason})
            return
        lifecycle, listing, getting = self.parameter_clients[target]
        try:
            if job is None:
                if all(client.service_is_ready() for client in (lifecycle, listing, getting)):
                    self.parameter_jobs[target] = ('state', lifecycle.call_async(GetState.Request()), now, [])
                return
            stage, future, started, names = job
            if not future.done():
                if now - started < 5.:
                    return
                future.cancel()
                raise TimeoutError(stage + ' service response timed out')
            response = future.result()
            del self.parameter_jobs[target]
            if stage == 'state':
                if response.current_state.id not in (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE):
                    raise ValueError('node is not configured: ' + response.current_state.label)
                self.parameter_jobs[target] = ('list', listing.call_async(ListParameters.Request(depth=0)), now, [])
            elif stage == 'list':
                names = list(response.result.names)
                missing = REQUIRED_PARAMETERS[target].difference(names)
                if missing:
                    raise ValueError('waiting for configured parameters: ' + ', '.join(sorted(missing)))
                self.parameter_jobs[target] = ('get', getting.call_async(GetParameters.Request(names=names)), now, names)
            else:
                values = {name: plain(value) for name, value in zip(names, response.values) if value.type}
                if not REQUIRED_PARAMETERS[target].issubset(values):
                    raise ValueError('required runtime values are not set')
                if self.record('parameters', '', {'node': node, 'session_id': self.parameter_session, 'values': values}):
                    self.parameters_completed.add(target)
        except Exception as exc:
            self.parameter_jobs.pop(target, None)
            self.parameter_errors[target] = str(exc)

    def publish_status(self, stopped=False):
        self.status_pub.publish(String(data=json.dumps(self.writer.status(stopped))))

    def finish(self):
        self.record('session_stop', '', {'topics_received': dict(self.counts),
                                       'intentionally_throttled': dict(self.throttled)})
        status = self.writer.close()
        status.update(schema_version=SCHEMA_VERSION, topics_received=dict(self.counts),
                      intentionally_throttled=dict(self.throttled), goals=self.goals.goals,
                      unobserved_topics=[self.topic(t) for t in self.expected_topics if not self.counts[t]],
                      goals_index_scope='latest 128 goals, latest 16 events each; full history in records.jsonl',
                      parameters_completed=sorted(self.parameters_completed),
                      parameter_session=self.parameter_session, parameters_failed=self.parameters_failed,
                      parameters_pending=sorted(set(self.parameter_clients)-self.parameters_completed-set(self.parameters_failed)),
                      finished_utc=datetime.now(timezone.utc).isoformat())
        try:
            atomic_json(self.writer.directory / 'summary.json', status)
        except OSError as exc:
            self.writer.fail('final summary write failed: ' + str(exc))
        self.publish_status(stopped=True)
        return self.writer.status(stopped=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='수동 관측 경로 디버깅 기록기')
    parser.add_argument('--directory', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--real-time', action='store_true')
    args = parser.parse_args(argv)
    writer = RecordWriter(args.directory)
    node = None
    stopped = threading.Event()
    rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    result = 1
    try:
        atomic_json(writer.directory / 'metadata.json', metadata(args.workspace, not args.real_time, args.namespace))
        node = PathDebugNode(writer, args.workspace, not args.real_time, args.namespace)
        print('경로 디버깅 기록: ' + str(writer.directory), flush=True)
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while rclpy.ok() and not stopped.is_set():
                executor.spin_once(timeout_sec=.1)
        finally:
            executor.remove_node(node)
            executor.shutdown()
        result = 0
    except Exception as exc:
        writer.fail(str(exc))
        print('경로 기록 오류: ' + str(exc), flush=True)
    finally:
        if node is not None:
            status = node.finish()
            node.destroy_node()
        else:
            status = writer.close()
        rclpy.try_shutdown()
        if status['error']:
            print(status['error'], flush=True)
            result = 1
    return result


if __name__ == '__main__':
    raise SystemExit(main())
