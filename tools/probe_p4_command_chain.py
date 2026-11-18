#!/usr/bin/env python3
"""Exercise the installed P4 command chain without Gazebo or physical devices.

The fixture supplies only time, wheel feedback, TF, scan and obstacle messages.
Actual installed velocity_smoother, collision_monitor and sim_base processes
produce the observations. A transparent monitored-to-final relay permits a
separate, explicitly labelled injection phase at the final command boundary.
Every required branch must be observed; missing evidence is NOT_EXERCISED.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import defaultdict
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[1]
LEASE = 0.30
TICK = 0.020
REQUIRED = (
    'installed_artifacts', 'isolated_domain', 'lifecycle_active',
    'source_headers_preserved', 'repeated_original_cannot_renew',
    'final_before_original_rejected', 'original_without_new_final_stays_zero',
    'matching_final_accepted', 'older_original_and_final_rejected',
    'fresh_header_zero_priority', 'old_intent_cannot_restart_after_zero',
    'new_original_can_restart', 'timeout_synthesized_zero_exercised',
    'timeout_cannot_extend_lease', 'monitor_stop_exercised',
    'monitor_approach_exercised', 'monitor_clear_with_new_intent',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def moving(event):
    return abs(event['vx']) > 1e-7 or abs(event['wz']) > 1e-7


class Probe:
    def __init__(self, args):
        # Set DDS isolation before importing/initializing ROS. No daemon, launch
        # stack, Gazebo transport or hardware port is used by this program.
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                               qos_profile_sensor_data)
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
        from lifecycle_msgs.srv import ChangeState, GetState
        from nav_msgs.msg import Odometry
        from nav2_msgs.msg import CollisionMonitorState
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        self.args = args
        self.directory = args.output.resolve()
        if self.directory.exists() and any(self.directory.iterdir()):
            raise FileExistsError('Use a new, empty output directory')
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / 'logs').mkdir()
        self.events_file = (self.directory / 'events.jsonl').open('w')
        self.result = {'status': 'RUNNING', 'checks': {
            name: {'status': 'NOT_EXERCISED'} for name in REQUIRED},
            'manifest': {}, 'metrics': {},
            'scope': 'Installed software with synthetic inputs; no robot motion or Gazebo physics.'}
        self.commands = []
        self.processes = []
        self.streams = defaultdict(list)
        self.started = time.monotonic()
        self.deadline = self.started + args.timeout
        self.phase = 'startup'
        self.relay = False
        self.auto_source = None
        self.source_stamps = set()
        self.next_clock = self.next_sensors = self.next_source = 0.0
        self.clock_times = []
        self.clock_stamps = []
        self.last_clock_ns = 100_000_000_000
        self.sensor_mode = 'clear'
        self.rclpy, self.Time = rclpy, Time
        self.TwistStamped, self.Clock = TwistStamped, Clock
        self.Odometry, self.LaserScan = Odometry, LaserScan
        self.PointCloud2, self.PointField = PointCloud2, PointField
        self.ChangeState, self.GetState = ChangeState, GetState
        rclpy.init(args=[])
        self.node = Node('p4_command_chain_probe')
        self.clock_pub = self.node.create_publisher(
            Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.wheel_pub = self.node.create_publisher(
            Odometry, '/sim/wheel/odometry_raw', qos_profile_sensor_data)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, '/perception/obstacles', qos_profile_sensor_data)
        self.source_pub = self.node.create_publisher(TwistStamped, '/cmd_vel/requested', 1)
        self.final_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.node.create_subscription(
            TwistStamped, '/cmd_vel/smoothed', lambda msg: self.velocity('smoothed', msg), 100)
        self.node.create_subscription(TwistStamped, '/cmd_vel/monitored', self.monitored, 100)
        self.node.create_subscription(Twist, '/sim/cmd_vel', self.output, 100)
        self.node.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.state, 100)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        transforms = []
        for parent, child in (('odom', 'base_footprint'), ('base_footprint', 'laser_frame')):
            msg = TransformStamped()
            msg.header.stamp = self.to_time(self.last_clock_ns)
            msg.header.frame_id, msg.child_frame_id = parent, child
            msg.transform.rotation.w = 1.0
            transforms.append(msg)
        self.transforms = transforms
        self.write_result()

    def sim_now(self):
        return 100_000_000_000 + int((time.monotonic() - self.started) * 1e9)

    def to_time(self, ns):
        return self.Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))

    def write_result(self):
        (self.directory / 'results.json').write_text(json.dumps(self.result, indent=2) + '\n')
        (self.directory / 'commands.json').write_text(json.dumps(self.commands, indent=2) + '\n')

    def event(self, kind, **values):
        now = time.monotonic()
        event = {'kind': kind, 'phase': self.phase, 'wall_monotonic': now,
                 'elapsed': now - self.started, 'sim_ns': self.sim_now(),
                 'published_clock_ns': self.last_clock_ns, **values}
        self.events_file.write(json.dumps(event, allow_nan=False) + '\n')
        self.streams[kind].append(event)
        return event

    def check(self, name, passed, *, exercised=True, **evidence):
        status = 'PASS' if passed and exercised else ('FAIL' if exercised else 'NOT_EXERCISED')
        self.result['checks'][name] = {'status': status, **evidence}
        self.write_result()

    def velocity(self, kind, msg, **metadata):
        return self.event(kind, stamp_ns=stamp_ns(msg.header.stamp),
                          frame_id=msg.header.frame_id,
                          vx=msg.twist.linear.x, wz=msg.twist.angular.z, **metadata)

    def monitored(self, msg):
        self.velocity('monitored', msg)
        if self.relay:
            self.final_pub.publish(msg)
            self.velocity('relayed_final', msg)

    def output(self, msg, info):
        # RMW timestamps describe this installed base publisher and transport.
        # This subscriber may run later when several sensor callbacks are ready.
        wall_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        published_ns = info['source_timestamp']
        if not 0 < published_ns <= wall_ns:
            raise RuntimeError('Base output has no usable RMW publication timestamp')
        published_mono_ns = mono_ns - (wall_ns - published_ns)
        i = bisect_right(self.clock_times, published_mono_ns) - 1
        self.event('output', vx=msg.linear.x, wz=msg.angular.z,
                   rmw_source_ns=published_ns, rmw_received_ns=info['received_timestamp'],
                   publisher_monotonic=published_mono_ns / 1e9,
                   clock_at_publication_ns=self.clock_stamps[i] if i >= 0 else None,
                   callback_delay_from_publication_s=(wall_ns - published_ns) / 1e9)

    def state(self, msg):
        self.event('monitor_state', action_type=msg.action_type, polygon_name=msg.polygon_name)

    def command(self, vx, stamp=None):
        msg = self.TwistStamped()
        # The small, explicit age avoids a future stamp if command and /clock
        # callbacks are scheduled in the opposite order in a receiving node.
        msg.header.stamp = self.to_time(stamp if stamp is not None else self.last_clock_ns - 10_000_000)
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(vx)
        return msg

    def source(self, msg):
        self.source_stamps.add(stamp_ns(msg.header.stamp))
        self.source_pub.publish(msg)
        return self.velocity('source', msg)

    def final(self, msg):
        publish_start = time.monotonic()
        self.final_pub.publish(msg)
        return self.velocity('injected_final', msg, publish_start_monotonic=publish_start)

    def fixture(self):
        now = time.monotonic()
        if now >= self.next_clock:
            self.last_clock_ns = self.sim_now()
            self.clock_times.append(time.monotonic_ns())
            self.clock_stamps.append(self.last_clock_ns)
            self.clock_pub.publish(self.Clock(clock=self.to_time(self.last_clock_ns)))
            self.next_clock = now + 0.002
        if now >= self.next_sensors:
            stamp = self.to_time(self.last_clock_ns - 10_000_000)
            wheel = self.Odometry()
            wheel.header.stamp = stamp
            wheel.header.frame_id, wheel.child_frame_id = 'odom', 'base_footprint'
            wheel.pose.pose.orientation.w = 1.0
            self.wheel_pub.publish(wheel)
            scan = self.LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser_frame'
            scan.angle_min, scan.angle_max, scan.angle_increment = -0.01, 0.01, 0.01
            scan.range_min, scan.range_max, scan.scan_time = 0.05, 10.0, 0.02
            scan.ranges = [math.inf, 0.20 if self.sensor_mode == 'stop' else math.inf, math.inf]
            self.scan_pub.publish(scan)
            cloud = self.PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, 'base_footprint'
            cloud.height, cloud.width = 1, 1
            cloud.fields = [self.PointField(name=name, offset=index * 4,
                            datatype=self.PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian, cloud.is_dense = False, True
            cloud.point_step, cloud.row_step = 12, 12
            cloud.data = struct.pack('<fff', 0.35 if self.sensor_mode == 'approach' else 5.0, 0.0, 0.10)
            self.cloud_pub.publish(cloud)
            self.next_sensors = now + 0.02
        if self.auto_source is not None and now >= self.next_source:
            self.source(self.command(self.auto_source['vx'], self.auto_source.get('stamp')))
            self.next_source = now + self.auto_source.get('period', 0.02)

    def pump(self, seconds, *, predicate=None, fixture=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if time.monotonic() > self.deadline:
                raise TimeoutError(f'Overall {self.args.timeout:g}s probe timeout')
            for process, record, _log in self.processes:
                if process.poll() is not None:
                    raise RuntimeError(f"{record['node']} exited unexpectedly: {process.returncode}")
            if fixture:
                self.fixture()
            self.rclpy.spin_once(self.node, timeout_sec=0.001)
            if predicate is not None and predicate():
                return True
        return predicate is None or predicate()

    def install_and_start(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        import yaml
        from omslam_navigation.launch_support import load_configuration

        self.pump(0.7, fixture=False)
        existing = self.node.get_node_names_and_namespaces()
        foreign = [list(item) for item in existing if item[0] != self.node.get_name()]
        self.check('isolated_domain', not foreign, domain=self.args.domain, existing_nodes=existing)
        if foreign:
            raise RuntimeError(f'Probe domain is in use: {foreign}')
        package_names = ('omslam_base', 'omslam_sim', 'omslam_navigation',
                         'nav2_velocity_smoother', 'nav2_collision_monitor')
        prefixes = {name: Path(get_package_prefix(name)).absolute() for name in package_names}
        modules = {
            'omslam_base.core': 'src/omslam_base/omslam_base/core.py',
            'omslam_sim.control': 'src/omslam_sim/omslam_sim/control.py',
            'omslam_sim.sim_base': 'src/omslam_sim/omslam_sim/sim_base.py',
            'omslam_navigation.launch_support': 'src/omslam_navigation/omslam_navigation/launch_support.py',
        }
        artifacts = {}
        for name, source in modules.items():
            installed = Path(importlib.import_module(name).__file__)
            artifacts[name] = {'installed': str(installed), 'source': str(self.args.workspace / source),
                               'installed_sha256': digest(installed),
                               'source_sha256': digest(self.args.workspace / source)}
        nav_share = Path(get_package_share_directory('omslam_navigation'))
        sim_share = Path(get_package_share_directory('omslam_sim'))
        config = load_configuration(nav_share)
        parameters = {name: config[name] for name in ('velocity_smoother', 'collision_monitor')}
        parameters.update(yaml.safe_load((sim_share / 'config/sim_base.yaml').read_text()))
        param_file = self.directory / 'installed_parameters.yaml'
        param_file.write_text(yaml.safe_dump(parameters, sort_keys=False))
        manifests = {name: Path(get_package_share_directory(name)) / 'package.xml'
                     for name in package_names}
        versions = {name: ET.parse(path).findtext('version') for name, path in manifests.items()}
        provenance = self.args.workspace / 'src/nav2_smac_planner/provenance.json'
        nav2_baseline = json.loads(provenance.read_text())['package_version']
        family = nav2_baseline.split('.')[:2]
        self.result['manifest'] = {
            'workspace': str(self.args.workspace), 'ros_domain_id': self.args.domain,
            'prefixes': {key: str(value) for key, value in prefixes.items()},
            'versions': versions, 'python_artifacts': artifacts,
            'package_manifests': {name: {'path': str(path), 'sha256': digest(path)}
                                  for name, path in manifests.items()},
            'nav2_release': {'ros_distro': os.environ.get('ROS_DISTRO'), 'baseline': nav2_baseline,
                             'family': '.'.join(family), 'provenance': str(provenance),
                             'provenance_sha256': digest(provenance)},
            'parameter_file': str(param_file), 'parameter_sha256': digest(param_file),
            'clock': {'start_ns': 100_000_000_000, 'rate': 1.0,
                      'publication_period_s': 0.002, 'stamp_age_offset_s': 0.01},
            'sensor_fixture': {'period_s': 0.02, 'wheel_velocity': 0.0,
                               'stop_scan_range_m': 0.20, 'approach_point_m': [0.35, 0.0, 0.10]},
            'sink': '/sim/cmd_vel', 'physical_devices_opened': False,
            'probe_sha256': digest(Path(__file__)),
        }
        wrong_prefixes = [name for name in ('omslam_base', 'omslam_sim', 'omslam_navigation')
                          if prefixes[name] != self.args.workspace / 'install' / name]
        mismatched_modules = [name for name, item in artifacts.items()
                              if item['installed_sha256'] != item['source_sha256']]
        errors = []
        if wrong_prefixes:
            errors.append('workspace prefix mismatch: ' + ', '.join(wrong_prefixes))
        if mismatched_modules:
            errors.append('source hash mismatch: ' + ', '.join(mismatched_modules))
        if os.environ.get('ROS_DISTRO') != 'jazzy':
            errors.append(f"expected the Jazzy underlay, found {os.environ.get('ROS_DISTRO')!r}")
        # The workspace pins its patched planner exactly. The underlay provides
        # smoother and monitor; test the installed patch's complete behavior.
        for name in ('nav2_velocity_smoother', 'nav2_collision_monitor'):
            parts = (versions[name] or '').split('.')
            if len(parts) != 3 or not parts[2].isdigit() or parts[:2] != family:
                errors.append(f"{name}: expected Nav2 {'.'.join(family)}.x, found {versions[name]}")
        self.check('installed_artifacts', not errors, manifest='manifest', errors=errors)
        if errors:
            raise RuntimeError('; '.join(errors))
        self.broadcaster.sendTransform(self.transforms)
        nodes = (
            ('nav2_velocity_smoother', 'velocity_smoother',
             ['-r', 'cmd_vel:=/cmd_vel/requested', '-r', 'cmd_vel_smoothed:=/cmd_vel/smoothed']),
            ('nav2_collision_monitor', 'collision_monitor', []),
            ('omslam_sim', 'sim_base', []),
        )
        for package, node, remaps in nodes:
            executable = prefixes[package] / 'lib' / package / node
            command = [str(executable), '--ros-args', '--params-file', str(param_file), *remaps]
            log = (self.directory / 'logs' / f'{node}.log').open('w')
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
            record = {'node': node, 'command': command, 'pid': process.pid,
                      'executable_sha256': digest(executable), 'started_monotonic': time.monotonic()}
            self.commands.append(record)
            self.processes.append((process, record, log))
        self.write_result()
        self.pump(0.5)
        states = {}
        for node in ('collision_monitor', 'velocity_smoother', 'sim_base'):
            self.transition(node, 1, 2)
        self.pump(0.10)
        for node in ('collision_monitor', 'velocity_smoother', 'sim_base'):
            states[node] = self.transition(node, 3, 3)
        self.relay = True
        self.check('lifecycle_active', all(value == 3 for value in states.values()), states=states)

    def transition(self, name, transition, expected):
        change = self.node.create_client(self.ChangeState, f'/{name}/change_state')
        state = self.node.create_client(self.GetState, f'/{name}/get_state')
        try:
            if not self.pump(5.0, predicate=lambda: change.service_is_ready() and state.service_is_ready()):
                raise TimeoutError(f'{name}: lifecycle services unavailable')
            request = self.ChangeState.Request()
            request.transition.id = transition
            future = change.call_async(request)
            if not self.pump(5.0, predicate=future.done) or not future.result().success:
                raise RuntimeError(f'{name}: transition {transition} failed')
            future = state.call_async(self.GetState.Request())
            if not self.pump(2.0, predicate=future.done):
                raise TimeoutError(f'{name}: state query timed out')
            observed = future.result().current_state.id
            self.event('lifecycle', node=name, transition=transition, state=observed)
            if observed != expected:
                raise RuntimeError(f'{name}: expected lifecycle state {expected}, observed {observed}')
            return observed
        finally:
            self.node.destroy_client(change)
            self.node.destroy_client(state)

    def select_phase(self, name):
        self.phase = name
        self.event('phase')
        return time.monotonic()

    def since(self, kind, start):
        return [event for event in self.streams[kind] if event['wall_monotonic'] >= start]

    def settle(self):
        self.select_phase('settle')
        self.sensor_mode, self.relay = 'clear', True
        self.auto_source = {'vx': 0.0}
        self.pump(0.70)
        self.auto_source, self.relay = None, False
        self.pump(0.04)

    def warm(self, phase):
        self.sensor_mode, self.relay = 'clear', True
        start = self.select_phase(phase)
        self.auto_source = {'vx': 0.20}
        self.pump(1.10)
        return start

    def expiry(self, start, source_event):
        outputs = [e for e in self.streams['output'] if e['publisher_monotonic'] >= start]
        nonzero = [event for event in outputs if moving(event)]
        stamp = source_event['stamp_ns']
        # The fixture maps 1:1 from monotonic time to ROS time, but /clock
        # advances in discrete messages. Keep both clocks at publication.
        wall_origin = self.started + (stamp - 100_000_000_000) / 1e9
        sim_deadline_wall = wall_origin + LEASE
        last_nonzero = max((e['publisher_monotonic'] for e in nonzero), default=start)
        zeros = [e for e in outputs if not moving(e) and e['publisher_monotonic'] > last_nonzero]
        first_zero = min(zeros, key=lambda e: e['publisher_monotonic'], default=None)
        after_deadline = [e for e in outputs if e['publisher_monotonic'] >= sim_deadline_wall + TICK or
                          (e['clock_at_publication_ns'] - stamp) / 1e9 >= LEASE + TICK]
        live_finals_after_expiry = [event for event in self.since('relayed_final', start)
                                   if event['wall_monotonic'] >= sim_deadline_wall + TICK and moving(event)]
        metrics = {
            'original_stamp_ns': stamp, 'source_publish_monotonic': source_event['wall_monotonic'],
            'original_wall_origin': wall_origin, 'deadline_monotonic': sim_deadline_wall,
            'allowance_s': TICK, 'nonzero_samples': len(nonzero), 'post_deadline_samples': len(after_deadline),
            'first_persistent_zero_monotonic': first_zero['publisher_monotonic'] if first_zero else None,
            'zero_wall_age_s': first_zero['publisher_monotonic'] - wall_origin if first_zero else None,
            'zero_sim_age_s': (first_zero['clock_at_publication_ns'] - stamp) / 1e9 if first_zero else None,
            'zero_callback_age_s': first_zero['wall_monotonic'] - wall_origin if first_zero else None,
            'zero_callback_delay_s': first_zero['callback_delay_from_publication_s'] if first_zero else None,
            'last_nonzero_monotonic': last_nonzero,
            'nonzero_final_samples_after_expiry': len(live_finals_after_expiry),
        }
        exercised = bool(nonzero and first_zero and len(after_deadline) >= 5
                         and len(live_finals_after_expiry) >= 3)
        passed = (exercised and metrics['zero_wall_age_s'] <= LEASE + TICK
                  and metrics['zero_sim_age_s'] <= LEASE + TICK
                  and all(not moving(event) for event in after_deadline))
        return passed, exercised, metrics

    def exercise(self):
        start = self.warm('normal_chain')
        smooth = self.since('smoothed', start)
        monitored = self.since('monitored', start)
        outputs = self.since('output', start)
        headers = {(event['stamp_ns'], event['frame_id']) for event in smooth}
        preserved = (all(event['stamp_ns'] in self.source_stamps and event['frame_id'] == 'base_footprint'
                         for event in smooth)
                     and all((event['stamp_ns'], event['frame_id']) in headers for event in monitored))
        self.check('source_headers_preserved', preserved,
                   exercised=len(smooth) >= 20 and len(monitored) >= 20 and any(event['vx'] > 0.15 for event in outputs),
                   smoothed_count=len(smooth), monitored_count=len(monitored),
                   maximum_output_vx=max((event['vx'] for event in outputs), default=0.0))

        start = self.select_phase('repeat_same_original')
        original = self.command(0.20)
        original_event = self.source(original)
        self.auto_source = {'vx': 0.20, 'stamp': stamp_ns(original.header.stamp), 'period': 0.01}
        self.pump(0.72)
        passed, exercised, metrics = self.expiry(start, original_event)
        metrics['repeat_count'] = len(self.since('source', start))
        self.result['metrics']['repeated_original_expiry'] = metrics
        self.check('repeated_original_cannot_renew', passed, exercised=exercised, **metrics)

        self.settle()
        start = self.select_phase('final_before_original')
        candidate = self.command(0.16)
        self.final(candidate)
        self.pump(0.045)
        samples = self.since('output', start)
        self.check('final_before_original_rejected', all(not moving(e) for e in samples),
                   exercised=len(samples) >= 2, output_count=len(samples))
        start = self.select_phase('original_without_new_final')
        self.source(candidate)
        self.pump(0.045)
        samples = self.since('output', start)
        self.check('original_without_new_final_stays_zero', all(not moving(e) for e in samples),
                   exercised=len(samples) >= 2, output_count=len(samples))
        start = self.select_phase('matching_final')
        self.final(candidate)
        self.pump(0.045)
        samples = self.since('output', start)
        self.check('matching_final_accepted', any(abs(e['vx'] - 0.16) < 1e-6 for e in samples),
                   exercised=len(samples) >= 2, output_count=len(samples))
        start = self.select_phase('older_original_and_final')
        older = self.command(0.04, stamp_ns(candidate.header.stamp) - 20_000_000)
        self.source(older)
        self.pump(0.01)
        self.final(older)
        self.pump(0.045)
        samples = self.since('output', start)
        self.check('older_original_and_final_rejected',
                   all(abs(e['vx'] - 0.16) < 1e-6 and abs(e['wz']) < 1e-7 for e in samples),
                   exercised=len(samples) >= 2, output_count=len(samples))

        self.settle()
        self.select_phase('zero_priority_setup')
        original = self.command(0.16)
        self.source(original)
        self.pump(0.025)
        self.final(original)
        start = time.monotonic()
        self.pump(0.045)
        moved_before_zero = any(moving(e) for e in self.since('output', start))
        self.select_phase('fresh_header_zero_priority')
        stop = self.command(0.0)
        stop_event = self.final(stop)
        self.pump(0.045)
        injected = stop_event['publish_start_monotonic']
        samples = [e for e in self.streams['output'] if e['publisher_monotonic'] >= injected]
        first_zero = next((e for e in samples if not moving(e)), None)
        first_zero_latency = first_zero['publisher_monotonic'] - injected if first_zero else None
        self.check('fresh_header_zero_priority',
                   first_zero_latency is not None and first_zero_latency <= TICK
                   and all(not moving(e) for e in samples if e['publisher_monotonic'] >= injected + TICK),
                   exercised=moved_before_zero and len(samples) >= 2,
                   zero_stamp_ns=stamp_ns(stop.header.stamp), original_stamp_ns=stamp_ns(original.header.stamp),
                   first_zero_latency_s=first_zero_latency, output_count=len(samples),
                   zero_callback_delay_s=first_zero['callback_delay_from_publication_s'] if first_zero else None)
        start = self.select_phase('old_intent_after_zero')
        for _ in range(5):
            self.source(original)
            self.final(original)
            self.pump(0.02)
        samples = self.since('output', start)
        self.check('old_intent_cannot_restart_after_zero', all(not moving(e) for e in samples),
                   exercised=len(samples) >= 4,
                   final_original_age_s=(self.last_clock_ns - stamp_ns(original.header.stamp)) / 1e9,
                   output_count=len(samples))
        start = self.select_phase('new_original_after_zero')
        new = self.command(0.16)
        self.source(new)
        self.pump(0.025)
        self.final(new)
        self.pump(0.05)
        samples = self.since('output', start)
        self.check('new_original_can_restart', any(abs(e['vx'] - 0.16) < 1e-6 for e in samples),
                   exercised=len(samples) >= 2, output_count=len(samples))

        self.settle()
        self.warm('timeout_setup')
        original_event = self.streams['source'][-1]
        self.auto_source = None
        start = self.select_phase('smoother_timeout')
        self.pump(1.05)
        smooth = self.since('smoothed', start)
        monitor = self.since('monitored', start)
        regenerated = [e for e in smooth if e['stamp_ns'] > original_event['stamp_ns']
                       and e['stamp_ns'] not in self.source_stamps]
        fresh_zeros = [e for e in regenerated if not moving(e)]
        monitor_headers = {(e['stamp_ns'], e['frame_id']) for e in monitor if not moving(e)}
        forwarded_zeros = [e for e in fresh_zeros if (e['stamp_ns'], e['frame_id']) in monitor_headers]
        self.check('timeout_synthesized_zero_exercised', bool(forwarded_zeros),
                   exercised=bool(regenerated), regenerated_count=len(regenerated),
                   fresh_zero_count=len(fresh_zeros), forwarded_fresh_zero_count=len(forwarded_zeros),
                   regenerated_frames=sorted({e['frame_id'] for e in regenerated}))
        passed, exercised, metrics = self.expiry(start, original_event)
        self.result['metrics']['timeout_expiry'] = metrics
        self.check('timeout_cannot_extend_lease', passed, exercised=exercised, **metrics)

        self.settle()
        self.warm('monitor_stop_setup')
        start = self.select_phase('monitor_stop')
        self.sensor_mode = 'stop'
        self.event('sensor_change', mode='stop', point=[0.20, 0.0])
        self.pump(0.35)
        states = self.since('monitor_state', start)
        zeros = [e for e in self.since('monitored', start) if not moving(e)]
        outputs = self.since('output', start + 0.15)
        stop_states = [e for e in states if e['action_type'] == 1 and e['polygon_name'] == 'Stop']
        self.check('monitor_stop_exercised', bool(zeros) and all(not moving(e) for e in outputs),
                   exercised=bool(stop_states) and len(outputs) >= 5,
                   stop_state_count=len(stop_states), monitored_zero_count=len(zeros), output_count=len(outputs))

        start = self.select_phase('monitor_approach')
        self.sensor_mode = 'approach'
        self.event('sensor_change', mode='approach', point=[0.35, 0.0, 0.10])
        self.pump(0.45)
        states = self.since('monitor_state', start)
        inputs = self.since('smoothed', start)
        input_by_stamp = defaultdict(list)
        for event in inputs:
            input_by_stamp[(event['stamp_ns'], event['frame_id'])].append(event)
        reduced = [event for event in self.since('monitored', start)
                   if event['vx'] > 0.0 and any(other['vx'] > event['vx'] + 0.005
                       for other in input_by_stamp[(event['stamp_ns'], event['frame_id'])])]
        approach_states = [e for e in states if e['action_type'] == 3 and e['polygon_name'] == 'Approach']
        outputs = self.since('output', start + 0.15)
        reduced_outputs = [e for e in outputs if 0.0 < e['vx'] < 0.19]
        self.check('monitor_approach_exercised', len(reduced) >= 5 and len(reduced_outputs) >= 5,
                   exercised=bool(approach_states) and len(inputs) >= 5,
                   approach_state_count=len(approach_states), reduced_monitor_count=len(reduced),
                   reduced_output_count=len(reduced_outputs),
                   observed_output_vx=sorted({round(e['vx'], 5) for e in outputs}))

        start = self.select_phase('monitor_clear')
        self.sensor_mode = 'clear'
        self.event('sensor_change', mode='clear')
        self.pump(0.30)
        states = self.since('monitor_state', start)
        outputs = self.since('output', start + 0.10)
        self.check('monitor_clear_with_new_intent', any(e['vx'] > 0.19 for e in outputs),
                   exercised=any(e['action_type'] == 0 for e in states) and len(outputs) >= 5,
                   output_count=len(outputs))

    def cleanup(self):
        self.auto_source, self.relay = None, False
        try:
            self.final(self.command(0.0))
            self.pump(0.04)
        except Exception:
            pass
        # A process group is used only for a child created above. Never inspect
        # or terminate an unrelated ROS or simulator process by name.
        for process, record, _log in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for process, record, log in reversed(self.processes):
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=1.0)
            record['exit_code'] = process.returncode
            record['finished_monotonic'] = time.monotonic()
            log.close()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
        self.events_file.close()

    def run(self):
        try:
            self.install_and_start()
            self.exercise()
            self.result['status'] = ('PASS' if all(check['status'] == 'PASS'
                                     for check in self.result['checks'].values()) else 'FAIL')
        except Exception as exc:
            self.result['status'] = 'FAIL'
            self.result['error'] = {'message': str(exc), 'traceback': traceback.format_exc()}
        finally:
            try:
                self.cleanup()
            except Exception as exc:
                self.result['status'] = 'FAIL'
                self.result['cleanup_error'] = {'message': str(exc), 'traceback': traceback.format_exc()}
            self.result['elapsed_s'] = time.monotonic() - self.started
            self.result['metrics']['observed_counts'] = {name: len(values) for name, values in self.streams.items()}
            self.write_result()
        print(json.dumps({'status': self.result['status'], 'results': str(self.directory / 'results.json'),
                          'checks': {name: check['status'] for name, check in self.result['checks'].items()}}, indent=2))
        return 0 if self.result['status'] == 'PASS' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--workspace', type=Path, default=WORKSPACE)
    parser.add_argument('--domain', type=int, default=190)
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args()
    args.workspace = args.workspace.resolve()
    if not 1 <= args.domain <= 232:
        parser.error('--domain must be an isolated domain in 1..232')
    if not 20 <= args.timeout <= 180:
        parser.error('--timeout must be between 20 and 180 seconds')
    try:
        return Probe(args).run()
    except Exception as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc), 'traceback': traceback.format_exc()}, indent=2))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
