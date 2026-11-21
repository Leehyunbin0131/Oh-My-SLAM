"""Own only launched processes; serve map operations independently of the GUI."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import tempfile
import time
import uuid
from contextlib import contextmanager
from datetime import datetime
from copy import deepcopy

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from omslam_operator_msgs.msg import KeepoutZones, OperatorState
from omslam_operator_msgs.srv import Control, MapCommand


class Processes:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.items = {}
        self.history = []
        self.lock = threading.RLock()
        self.closing = False

    def start(self, key, argv):
        with self.lock:
            if self.closing:
                raise RuntimeError('세션이 종료 중입니다.')
            if key in self.items:
                raise RuntimeError(f'{key} already owned')
            out = (self.directory / f'{key}-{len(self.history)}.log').open('w')
            try:
                proc = subprocess.Popen(argv, stdout=out, stderr=subprocess.STDOUT,
                                        start_new_session=True, cwd='/tmp')
            except Exception:
                out.close()
                raise
            self.items[key] = (proc, out)
            self.history.append({'key': key, 'pid': proc.pid, 'argv': argv})
            (self.directory / 'processes.json').write_text(json.dumps(self.history, indent=2))
            return proc

    def stop(self, key):
        with self.lock:
            item = self.items.pop(key, None)
        if item is None:
            return
        proc, out = item
        try:
            for sig, seconds in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    proc.poll()
                    try:
                        os.killpg(proc.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(.1)
                else:
                    continue
                break
            proc.wait(timeout=3)
        finally:
            out.close()

    def close(self):
        with self.lock:
            self.closing = True
            keys = list(self.items)
        errors = []
        for key in reversed(keys):
            try:
                self.stop(key)
            except (OSError, subprocess.TimeoutExpired) as error:
                errors.append(f'{key}: {error}')
        if errors:
            raise RuntimeError('Process shutdown failed: ' + ', '.join(errors))

    def exited(self):
        """Poll only currently owned children, atomically with stop/start.

        stop() removes ownership before signalling a process. Holding this lock
        through poll prevents a stale monitor snapshot from reporting that
        intentional shutdown as a crash during a navigation mode change.
        """
        with self.lock:
            return [(key, proc.returncode) for key, (proc, _) in self.items.items()
                    if key != 'map_save' and proc.poll() is not None]


def map_prefix(root, value):
    """Resolve without touching files; reject suffixes and existing bundle members."""
    p = Path(value).expanduser()
    if not p.is_absolute():
        raise ValueError('저장 경로는 이 호스트의 절대 경로여야 합니다.')
    p = p.resolve()
    if p.suffix in ('.yaml', '.pgm', '.posegraph', '.data', '.json'):
        raise ValueError('확장자를 제외한 지도 이름을 입력하세요.')
    for suffix in ('.yaml', '.pgm', '.posegraph', '.data', '.metadata.json', '.keepout.json', '.keepout.pending'):
        if Path(str(p) + suffix).exists() or Path(str(p) + suffix).is_symlink():
            raise ValueError('기존 지도 이름입니다. 새 이름을 입력하세요.')
    return p


def write_new_file(target, contents):
    """Publish a complete file without replacing a concurrent writer's file."""
    target = Path(target)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='wb',
                dir=target.parent, prefix='.' + target.name, suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        # rename/replace would overwrite a sidecar created concurrently.
        os.link(temporary, target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return target


def save_keepout_sidecar(map_path, polygons):
    """Install a complete sidecar atomically without overwriting any file."""
    from omslam_navigation.keepouts import validate_polygons
    from omslam_navigation.launch_support import map_image_path, validate_map_bundle
    path = validate_map_bundle(map_path, allow_pending_keepouts=True)
    data = {'schema_version': 1, 'frame_id': 'map',
            'map_sha256': {name: hashlib.sha256(actual.read_bytes()).hexdigest()
                           for name, actual in [('yaml', path), ('pgm', map_image_path(path))]},
            'polygons': validate_polygons(polygons)}
    return write_new_file(path.with_suffix('.keepout.json'),
                          (json.dumps(data, ensure_ascii=False, indent=2) + '\n').encode())


def copy_localization_map(source, prefix, reserved=False):
    """Save As a loaded raster map, without claiming to create a SLAM graph."""
    from omslam_navigation.launch_support import map_image_path, validate_map_bundle
    source = validate_map_bundle(source)
    prefix = Path(prefix) if reserved else map_prefix(None, str(prefix))
    prefix.parent.mkdir(parents=True, exist_ok=True)
    image = map_image_path(source).read_bytes()
    config = yaml.safe_load(source.read_text())
    config['image'] = prefix.name + '.pgm'
    raster = yaml.safe_dump(config, sort_keys=False).encode()
    metadata = deepcopy(json.loads(source.with_suffix('.metadata.json').read_text()))
    metadata['operation'] = 'copy_saved_raster'
    metadata['source_map'] = {'path': str(source),
                              'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    # The selected saved raster is reused. No online graph was produced in
    # localization mode, and old graph file paths must not appear as new output.
    metadata['files'] = {}
    for suffix, contents in [('pgm', image), ('yaml', raster)]:
        target = Path(str(prefix) + '.' + suffix)
        metadata['files'][suffix] = {'path': str(target),
            'sha256': hashlib.sha256(contents).hexdigest(), 'size_bytes': len(contents)}
        write_new_file(target, contents)
    write_new_file(Path(str(prefix) + '.metadata.json'),
                   (json.dumps(metadata, ensure_ascii=False, indent=2) + '\n').encode())
    return Path(str(prefix) + '.yaml')


class MapSupport(Node):
    def __init__(self, args, processes):
        super().__init__('operator_session', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=not args.real)])
        self.args, self.processes = args, processes
        self.root = Path(args.workspace).resolve()
        self.map_directory = self.root / 'maps' / 'operator'
        self.state = None
        self.state_at = 0.
        self.map_stamp = 0
        self.map_received_at = 0.
        self.amcl_estimate = (0, 0.)
        self.keepout_lock = threading.Lock()
        self.keepout_sessions = {}
        self.loaded_map_path = None
        self.busy = threading.Lock()
        group = ReentrantCallbackGroup()
        self.map_group = group
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OperatorState, '/operator/state', self.receive_state, qos,
                                 callback_group=group)
        self.keepout_sub = self.create_subscription(KeepoutZones, '/operator/keepouts',
            self.receive_keepouts, qos, callback_group=group)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.receive_map, qos,
                                                callback_group=group)
        self.stop_client = self.create_client(Trigger, '/operator/stop', callback_group=group)
        self.control = self.create_client(Control, '/operator/control', callback_group=group)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.receive_amcl, 1,
                                 callback_group=group)
        self.create_service(MapCommand, '/operator/maps', self.command, callback_group=group)

    def receive_state(self, value):
        self.state, self.state_at = value, time.monotonic()

    def receive_amcl(self, message):
        if message.header.frame_id == 'map':
            stamp = message.header.stamp
            self.amcl_estimate = (stamp.sec * 1_000_000_000 + stamp.nanosec, time.monotonic())

    def wait_initial_pose(self, session, stamp, sent):
        deadline = sent + 8.
        while time.monotonic() < deadline:
            observed, received = self.amcl_estimate
            state = self.state
            if (state and state.session_id == session and state.localization_initialized
                    and state.stopped and not state.base_active and time.monotonic()-self.state_at < .6
                    and observed >= stamp and received >= sent):
                return
            time.sleep(.05)
        raise RuntimeError('초기 위치를 보냈지만 현재 지도에서 AMCL 위치 관측을 확인하지 못했습니다.')

    def receive_keepouts(self, message):
        if message.header.frame_id != 'map' or not message.session_id:
            return
        with self.keepout_lock:
            previous = self.keepout_sessions.get(message.session_id)
            if previous is None or message.revision >= previous.revision:
                self.keepout_sessions[message.session_id] = message
            while len(self.keepout_sessions) > 4:
                self.keepout_sessions.pop(next(iter(self.keepout_sessions)))

    def keepout_snapshot(self):
        """Require the current guard's state, even when zones are empty."""
        from omslam_navigation.keepouts import validate_polygons
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            state = self.state
            if state is not None and time.monotonic() - self.state_at < .6:
                with self.keepout_lock:
                    message = self.keepout_sessions.get(state.session_id)
                if message is not None:
                    return (message.session_id, message.revision,
                            validate_polygons([[[p.x, p.y] for p in polygon.points]
                                               for polygon in message.polygons]))
            time.sleep(.02)
        raise RuntimeError('현재 지도 세션의 진입 금지 구역 상태를 확인하지 못했습니다.')

    def receive_map(self, message):
        if (message.header.frame_id == 'map' and message.info.width > 0
                and message.info.height > 0 and message.info.resolution > 0
                and len(message.data) == message.info.width * message.info.height):
            self.map_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.map_received_at = time.monotonic()

    def localization_map_ready(self, transition_wall):
        # A static map retains its load stamp, which can be zero before the new
        # map_server receives /clock. Its new subscription receipt is the load
        # evidence; live guard/odometry freshness is checked independently.
        return (self.map_received_at > transition_wall and not self.state.base_active
                and self.count_subscribers('/initialpose') > 0)

    def call(self, client, request, timeout=12.):
        if not client.wait_for_service(timeout_sec=1.):
            raise RuntimeError('로봇 제어 서비스에 연결되지 않았습니다.')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError('로봇 응답 시간 초과. 정지 여부를 확인할 수 없습니다.')
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        return result

    def authorize(self, request):
        self.call(self.control, Control.Request(command='validate', owner_id=request.session_id,
                  token=request.token, epoch=request.epoch, request_id='map-validate'))
        if self.args.real:
            raise RuntimeError('실물 지도 변경은 검증된 P5 backend 연결 후 지원합니다.')

    def stop_and_confirm(self):
        self.call(self.stop_client, Trigger.Request())
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            state = self.state
            if (state is not None and time.monotonic() - self.state_at < .6 and
                    state.stopped and not state.base_active):
                return
            time.sleep(.05)
        raise RuntimeError('최신 실제 정지·구동해제 상태를 확인할 수 없습니다.')

    @contextmanager
    def map_reservation(self, request, ending_guard=False):
        # The guard holds the operation independently of the GUI owner, which
        # is released by emergency stop. A dead support process cannot lock it forever.
        transaction = dict(owner_id=request.session_id, token=request.token, epoch=request.epoch,
                           request_id=uuid.uuid4().hex)
        def send(command):
            self.call(self.control, Control.Request(command=command, **transaction), timeout=3.)
        send('begin_map')
        done = threading.Event()
        def maintain():
            while not done.wait(2.):
                try:
                    send('hold_map')
                except RuntimeError:
                    # The pre-commit check rejects an expired/replaced lease.
                    continue
        thread = threading.Thread(target=maintain, daemon=True)
        thread.start()
        try:
            yield lambda: send('hold_map')
        finally:
            done.set()
            thread.join()
            # The old server and its reservation have both gone. Never send
            # its credentials to the replacement server.
            if not ending_guard or 'navigation' in self.processes.items:
                try:
                    send('end_map')
                except RuntimeError as exc:
                    self.get_logger().warning('지도 작업 예약 해제 응답 대기: ' + str(exc))

    def save_reserved(self, path, response, confirm_reservation):
        from omslam_mapping.bundle_transaction import staged_map
        session, revision, polygons = self.keepout_snapshot()
        suffixes = ('.yaml', '.pgm', '.metadata.json', '.keepout.json')
        if self.loaded_map_path is None:
            suffixes += ('.posegraph', '.data')
        with staged_map(path, suffixes) as stage:
            if self.loaded_map_path is not None:
                copy_localization_map(self.loaded_map_path, stage, reserved=True)
            else:
                process = self.processes.start('map_save', ['ros2', 'run', 'omslam_mapping', 'save_map',
                    '--output', str(stage), '--workspace', str(self.root), '--timeout', '20'])
                log = Path(self.processes.items['map_save'][1].name)
                try:
                    if process.wait(timeout=65):
                        raise RuntimeError(log.read_text()[-1800:])
                finally:
                    self.processes.stop('map_save')
            save_keepout_sidecar(str(stage) + '.yaml', polygons)
            if self.keepout_snapshot()[:2] != (session, revision):
                raise RuntimeError('저장 중 지도 또는 금지 구역이 변경됐습니다. 다시 저장하세요.')
            confirm_reservation()
        response.paths = [str(path) + '.yaml', str(path) + '.keepout.json']
        response.message = f'지도·진입 금지 구역 저장 완료: {os.uname().nodename}:{path}'

    def start_navigation(self, mode='slam', path='', pose=None):
        launch = 'slam_navigation.launch.py' if mode == 'slam' else 'navigation.launch.py'
        argv = ['ros2', 'launch', 'omslam_navigation', launch, 'start_sim:=false',
                'operator_enabled:=true', f'world:={self.args.world}']
        if mode == 'amcl':
            argv.append('map:=' + path)
            if pose is not None:
                argv += ['initial_pose:=true', f'initial_x:={pose[0]}',
                         f'initial_y:={pose[1]}', f'initial_yaw:={pose[2]}']
        self.processes.start('navigation', argv)
        self.loaded_map_path = Path(path) if mode == 'amcl' else None

    def command(self, req, res):
        if not self.busy.acquire(blocking=False):
            res.message = '다른 지도 작업이 진행 중입니다.'
            return res
        try:
            if req.operation == 'list':
                res.paths = sorted(str(p.resolve()) for p in self.map_directory.glob('*.yaml'))
                res.success, res.message = True, f'{os.uname().nodename}: {self.map_directory}'
                return res
            if req.operation not in ('save', 'slam', 'localize', 'initialpose'):
                raise ValueError('지원하지 않는 지도 작업입니다.')
            if not all(math.isfinite(v) for v in (req.x, req.y, req.yaw)):
                raise ValueError('위치·방향은 유한한 수여야 합니다.')
            self.authorize(req)
            if req.operation in ('slam', 'localize') and 'navigation' not in self.processes.items:
                raise RuntimeError('이 앱이 시작한 세션에서만 지도 모드를 바꿀 수 있습니다.')
            path = None
            if req.operation == 'localize':
                from omslam_navigation.launch_support import validate_map_bundle
                path = validate_map_bundle(req.path)
            if req.operation == 'save':
                path = map_prefix(self.root, req.path)
            replacing = req.operation in ('slam', 'localize')
            with self.map_reservation(req, ending_guard=replacing) as confirm_reservation:
                self.stop_and_confirm()
                if req.operation == 'save':
                    self.save_reserved(path, res, confirm_reservation)
                elif req.operation == 'initialpose':
                    if not self.count_subscribers('/initialpose'):
                        raise RuntimeError('AMCL 초기 위치 수신 노드가 없습니다.')
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.pose.position.x, msg.pose.pose.position.y = req.x, req.y
                    msg.pose.pose.orientation.z = math.sin(req.yaw / 2)
                    msg.pose.pose.orientation.w = math.cos(req.yaw / 2)
                    msg.pose.covariance[0] = msg.pose.covariance[7] = .25
                    msg.pose.covariance[35] = .0685
                    confirm_reservation()
                    session = self.state.session_id
                    sent = time.monotonic()
                    self.pose_pub.publish(msg)
                    self.wait_initial_pose(session, msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec, sent)
                    confirm_reservation()
                    res.message = '현재 지도의 AMCL 초기 위치 관측을 확인했습니다.'
                else:
                    previous_session = self.state.session_id
                    transition_stamp = self.get_clock().now().nanoseconds
                    confirm_reservation()
                    self.processes.stop('navigation')
                    self.state = None
            if replacing:
                with self.keepout_lock:
                    self.keepout_sessions.clear()
                # Drop the previous publisher's queued map before starting the
                # replacement backend. A cached old grid must not satisfy load.
                self.destroy_subscription(self.map_sub)
                self.map_stamp = 0
                self.map_received_at = 0.
                transition_wall = time.monotonic()
                self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.receive_map,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                    callback_group=self.map_group)
                # A selected destination is not an AMCL estimate of the current robot pose.
                self.start_navigation('slam' if req.operation == 'slam' else 'amcl', str(path or ''))
                # Do not claim readiness merely because Popen returned successfully.
                deadline = time.monotonic() + 45.
                while time.monotonic() < deadline:
                    proc = self.processes.items['navigation'][0]
                    if proc.poll() is not None:
                        raise RuntimeError('새 주행 모드 실행 실패. 세션 로그를 확인하세요.')
                    if (self.state and time.monotonic() - self.state_at < .6
                            and self.state.session_id != previous_session
                            and (self.state.auto_ready if req.operation == 'slam' else
                                 self.localization_map_ready(transition_wall))
                            and self.state.header.stamp.sec * 1_000_000_000
                                + self.state.header.stamp.nanosec > transition_stamp):
                        break
                    time.sleep(.1)
                else:
                    raise RuntimeError('새 지도·위치추정 준비 시간 초과. 구동은 해제되어 있습니다.')
                res.message = ('새 지도 작성 준비가 완료되었습니다.' if req.operation == 'slam' else
                               '지도 로드 완료. 지도에서 현재 로봇 위치와 방향을 지정하세요.')
            res.success = True
        except Exception as exc:
            res.success, res.message = False, str(exc)
        finally:
            self.busy.release()
        return res


def start_path_debug(processes, args, gui_args):
    """Start a passive recorder before GUI discovery; its files are session-owned."""
    if not args.debug_path:
        return list(gui_args)
    options = argparse.ArgumentParser(add_help=False)
    options.add_argument('--namespace', default='')
    namespace = options.parse_known_args(gui_args)[0].namespace
    directory = processes.directory / 'path-debug'
    command = [sys.executable, '-m', 'omslam_operator_support.path_debug',
               '--directory', str(directory), '--workspace', str(Path(args.workspace).resolve()),
               '--namespace', namespace]
    if args.real:
        command.append('--real-time')
    process = processes.start('path_debug', command)
    deadline = time.monotonic() + 10.
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'경로 기록 시작 실패: {processes.directory}')
        if (directory / 'ready.json').is_file():
            print(f'경로 자동 기록: {directory}', flush=True)
            return [*gui_args, '--path-debug-directory', str(directory)]
        time.sleep(.05)
    raise RuntimeError(f'경로 기록 준비 시간 초과: {processes.directory}')


def recorded_observer(args, processes, gui_args):
    """An extra window records independently and never owns the existing robot."""
    running = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: running.set())
    signal.signal(signal.SIGTERM, lambda *_: running.set())
    try:
        arguments = start_path_debug(processes, args, gui_args)
        gui = processes.start('gui', [sys.executable, '-m', 'omslam_operator.app',
            *(['--real-time', '--read-only'] if args.real else ['--use-sim-time']), *arguments])
        while not running.wait(.1):
            if gui.poll() is not None:
                return gui.returncode
            for key, code in processes.exited():
                if key == 'path_debug':
                    print(f'경로 기록 프로세스 종료: {code}; {processes.directory}', file=sys.stderr)
                    processes.stop(key)
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        processes.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='로봇 통합 제어 GUI')
    parser.add_argument('--start-sim', action='store_true', help='앱 소유 시뮬레이션 시작')
    parser.add_argument('--world', choices=['p4_course', 'airport'], default='p4_course')
    parser.add_argument('--gazebo-gui', action='store_true')
    parser.add_argument('--real', action='store_true', help='실물 관측 연결 (구동 미허용)')
    parser.add_argument('--no-gui', action='store_true', help='검증용 세션·지도 서비스만 실행')
    parser.add_argument('--debug-path', action='store_true',
                        help='경로·로봇 위치·목표·도착 결과를 세션 폴더에 자동 기록')
    parser.add_argument('--workspace', default=os.environ.get('OMSLAM_CONSOLE_ROOT', str(Path.cwd())))
    args, gui_args = parser.parse_known_args(argv)
    if args.real and args.start_sim:
        parser.error('--real과 --start-sim은 함께 사용할 수 없습니다.')
    root = Path(args.workspace).resolve()
    logs = root / 'reports' / 'operator' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    lock = open(f'/tmp/omslam-console-{os.getuid()}-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        if args.start_sim or args.no_gui:
            sys.exit('같은 ROS domain의 콘솔 지원 세션이 이미 실행 중입니다.')
        if args.debug_path:
            return recorded_observer(args, Processes(logs), gui_args)
        # Additional windows share the existing support service and start as observers.
        os.execv(sys.executable, [sys.executable, '-m', 'omslam_operator.app',
                 *(['--real-time', '--read-only'] if args.real else ['--use-sim-time']), *gui_args])
    processes = Processes(logs)
    rclpy.init(args=[])
    node = MapSupport(args, processes)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    running = threading.Event()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def stop_signal(*_):
        running.set()
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    code = 0
    try:
        gui_args = start_path_debug(processes, args, gui_args)
        if args.start_sim:
            time.sleep(2.)
            if node.count_publishers('/clock') or node.count_publishers('/navigation/state'):
                raise RuntimeError('같은 domain에 실행 중인 로봇이 있습니다. 관측 연결을 사용하세요.')
            processes.start('simulation', ['ros2', 'launch', 'omslam_sim', 'sim.launch.py',
                f'world:={args.world}', f'gui:={str(args.gazebo_gui).lower()}',
                'pedestrians:=false'])
            node.start_navigation()
        monitor_config = Path(get_package_share_directory('omslam_system_monitor')) / 'config/monitor.yaml'
        processes.start('system_monitor', ['ros2', 'run', 'omslam_system_monitor', 'system_monitor',
                                          '--ros-args', '--params-file', str(monitor_config), '-p', 'host_role:=' + ('simulation_host' if args.start_sim else 'operator_pc')])
        if not args.no_gui:
            gui = processes.start('gui', [sys.executable, '-m', 'omslam_operator.app',
                                  *(['--real-time', '--read-only'] if args.real else ['--use-sim-time']), *gui_args])
        print(f'콘솔 세션 로그: {logs}', flush=True)
        while not running.wait(.1):
            for key, returncode in processes.exited():
                if key == 'gui':
                    code = returncode
                    running.set()
                elif key in ('path_debug', 'system_monitor'):
                    # Recording failure must not tear down a navigation session.
                    # The GUI independently detects missing status heartbeats.
                    print(f'{key} 프로세스 종료: {returncode}; {logs}', file=sys.stderr)
                    processes.stop(key)
                else:
                    raise RuntimeError(f'{key} 종료: {returncode}; {logs}')
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        code = 1
    finally:
        # An observer has no right to stop somebody else's externally owned session.
        if args.start_sim:
            try:
                node.stop_and_confirm()
            except Exception as exc:
                print(f'종료 정지 확인: {exc}', file=sys.stderr)
        processes.close()
        executor.shutdown(timeout_sec=3.)
        node.destroy_node()
        rclpy.try_shutdown()
        lock.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
