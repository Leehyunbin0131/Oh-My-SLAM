#!/usr/bin/env python3
"""Bounded installed mock-base + real robot_localization acceptance run.

Encoder and IMU fixtures are independent measurements. No hardware, Gazebo,
lidar, SLAM, or existing ROS graph is contacted. Source /opt/ros/jazzy and this
workspace's newly built install/setup.bash before invoking this tool.
"""
from __future__ import annotations

import argparse
from collections import deque
import copy
from datetime import datetime
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import traceback


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def gid_text(gid):
    return bytes(gid).hex()


class Results:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.data = {
            'status': 'RUNNING', 'started_at': datetime.now().astimezone().isoformat(),
            'scope': 'installed software: injected encoder samples, independent synthetic IMU, actual robot_localization',
            'unverified': ['physical motor actuation, serial hardware, sensor noise/mount calibration, floor distance/turn accuracy',
                           'Gazebo, lidar SLAM, Nav2'],
            'checks': {},
        }

    def save(self):
        (self.directory / 'results.json').write_text(json.dumps(self.data, indent=2) + '\n')

    def check(self, name, passed, **details):
        self.data['checks'][name] = {'status': 'PASS' if passed else 'FAIL', **details}
        print(f"{name}: {'PASS' if passed else 'FAIL'}", flush=True)
        self.save()
        if not passed:
            raise AssertionError(f'{name}: {details}')


class Runtime:
    RADIUS = .068
    COUNTS_PER_REV = 2652
    BIAS = [.01, .02, .03]

    def __init__(self, result, workspace, timeout, reuse_measured=None):
        self.result = result
        self.reuse_measured = reuse_measured
        self.workspace = workspace.resolve()
        self.deadline = time.monotonic() + timeout
        self.process = self.log_handle = self.node = self.context = self.executor = None
        self.encoder_enabled = False
        self.duplicate_encoder = False
        self.encoder_tick = 1000
        self.encoder_count = 0.
        self.encoder_previous_time = self.encoder_epoch = time.monotonic()
        self.encoder_speed = 0.
        self.yaw_rate = 0.
        self.imu_enabled = True
        self.raw_imu = {}
        self.wheel, self.joints, self.filtered, self.targets, self.conditioned = (deque(maxlen=4000) for _ in range(5))
        self.transforms = deque(maxlen=12000)
        self.static_transforms = {}
        self.diagnostics = deque(maxlen=200)
        self.transitions = []

    def initialize(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import Imu, JointState
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, Int64MultiArray, Float64MultiArray
        from tf2_msgs.msg import TFMessage
        from diagnostic_msgs.msg import DiagnosticArray
        from lifecycle_msgs.srv import ChangeState, GetState

        self.rclpy, self.Imu, self.TwistStamped = rclpy, Imu, TwistStamped
        self.Int64MultiArray, self.Bool = Int64MultiArray, Bool
        self.ChangeState, self.GetState = ChangeState, GetState
        prefixes = {name: str(Path(get_package_prefix(name)).resolve()) for name in
                    ('omslam_control_vendor', 'omslam_base', 'omslam_localization', 'robot_localization')}
        local = {name: path for name, path in prefixes.items() if name != 'robot_localization'}
        self.result.check('installed_package_prefixes', all(Path(path) == self.workspace/'install'/name
                          for name, path in local.items()), prefixes=prefixes)
        files = {}
        for name in local:
            root = Path(local[name])
            for path in sorted(root.rglob('*')):
                if path.is_file() and path.suffix in ('.py', '.yaml', '.json'):
                    files[str(path)] = digest(path)
        for name in ('omslam_base', 'omslam_localization'):
            module = importlib.import_module(name)
            self.result.check(name + '_installed_import', str(module.__file__).startswith(local[name] + '/'),
                              imported_from=module.__file__)
        self.result.data['manifest'] = {'prefixes': prefixes, 'installed_file_sha256': files,
                                       'robot_localization_executable_sha256': digest(Path(prefixes['robot_localization'])/'lib/robot_localization/ekf_node'),
                                       'verifier_sha256': digest(__file__)}
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node('base_ekf_acceptance', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.pump(.4)
        existing = [n for n in self.node.get_node_names() if n != self.node.get_name()]
        self.result.check('isolated_empty_ros_graph', not existing, ros_domain_id=os.environ['ROS_DOMAIN_ID'],
                          existing_nodes=existing, discovery_range=os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'])

        reliable_one = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.requested_pub = self.node.create_publisher(TwistStamped, '/cmd_vel/requested', reliable_one)
        self.final_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', reliable_one)
        self.encoder_pub = self.node.create_publisher(Int64MultiArray, '/mock/encoder_sample', 10)
        self.fault_pub = self.node.create_publisher(Bool, '/mock/motor_fault', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)
        for topic, cls, target, qos in [('/wheel/odometry', Odometry, self.wheel, 100),
                                       ('/joint_states', JointState, self.joints, 100),
                                       ('/odom', Odometry, self.filtered, 100),
                                       ('/mock/wheel_target', Float64MultiArray, self.targets, 100),
                                       ('/imu/data', Imu, self.conditioned, qos_profile_sensor_data),
                                       ('/diagnostics', DiagnosticArray, self.diagnostics, 10)]:
            self.node.create_subscription(cls, topic, lambda msg, target=target: target.append((time.monotonic(), msg)), qos)
        self.node.create_subscription(TFMessage, '/tf', self.receive_tf, 100)
        static_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, '/tf_static', self.receive_static_tf, static_qos)
        self.node.create_timer(.05, self.encoder_timer)
        self.node.create_timer(.02, self.imu_timer)
        self.change_client = self.node.create_client(ChangeState, '/base_bridge/change_state')
        self.state_client = self.node.create_client(GetState, '/base_bridge/get_state')

        config = self.result.directory / 'imu_fixture.yaml'
        config.write_text('imu_conditioner:\n  ros__parameters:\n    expected_frame: imu_link\n'
                          '    gyro_bias_rad_s: [0.01, 0.02, 0.03]\n    max_age_s: 0.2\n')
        command = ['ros2', 'launch', 'omslam_localization', 'base_ekf.launch.py', 'backend:=mock',
                   'imu_source:=external', 'publish_static_tfs:=true', 'use_sim_time:=false',
                   'imu_roll:=' + str(math.pi/2), 'imu_config:=' + str(config)]
        environment = dict(os.environ, ROS_LOG_DIR=str(self.result.directory/'ros_logs'))
        self.log_handle = (self.result.directory/'launch.log').open('w')
        self.process = subprocess.Popen(command, stdout=self.log_handle, stderr=subprocess.STDOUT,
                                        env=environment, cwd='/tmp', start_new_session=True)
        self.result.data['launch'] = {'command': command, 'pid': self.process.pid,
                                      'ros_domain_id': environment['ROS_DOMAIN_ID'], 'start_new_session': True}
        self.result.save()
        self.until(lambda: self.change_client.service_is_ready() and self.state_client.service_is_ready(), 25,
                   'installed lifecycle service')
        self.until(lambda: self.conditioned and ('base_link', 'imu_link') in self.static_transforms, 10,
                   'conditioner and nonzero IMU mounting TF')

    def receive_tf(self, msg):
        for transform in msg.transforms:
            self.transforms.append((time.monotonic(), transform))

    def receive_static_tf(self, msg):
        for transform in msg.transforms:
            self.static_transforms[(transform.header.frame_id, transform.child_frame_id)] = transform

    def encoder_timer(self):
        if not self.encoder_enabled:
            return
        now = time.monotonic()
        if not self.duplicate_encoder:
            elapsed = now - self.encoder_previous_time
            self.encoder_tick = 1000 + int((now-self.encoder_epoch)*1000)
            self.encoder_count += self.encoder_speed*elapsed/(2*math.pi*self.RADIUS)*self.COUNTS_PER_REV
            self.encoder_previous_time = now
        count = round(self.encoder_count)
        self.encoder_pub.publish(self.Int64MultiArray(data=[self.encoder_tick, count, count]))

    def imu_timer(self):
        if not self.imu_enabled:
            return
        msg = self.Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.
        msg.linear_acceleration_covariance[0] = -1.
        # R_x(pi/2)^T * [0, 0, yaw_rate] is [0, yaw_rate, 0].
        msg.angular_velocity.x = self.BIAS[0]
        msg.angular_velocity.y = self.yaw_rate + self.BIAS[1]
        msg.angular_velocity.z = self.BIAS[2]
        msg.angular_velocity_covariance = [.0025, 0., 0., 0., .0025, 0., 0., 0., .0025]
        self.raw_imu[stamp_ns(msg.header.stamp)] = (msg.angular_velocity.y, list(msg.angular_velocity_covariance))
        self.imu_pub.publish(msg)

    def pump(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            if time.monotonic() > self.deadline:
                raise TimeoutError('Overall bounded runtime deadline reached')
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError(f'Installed launch exited early: {self.process.returncode}')
            self.executor.spin_once(timeout_sec=min(.01, max(0., end-time.monotonic())))

    def until(self, predicate, timeout, description):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.pump(.02)
        raise TimeoutError('Timed out waiting for ' + description)

    def call(self, client, request):
        future = client.call_async(request)
        self.until(future.done, 4., 'service response')
        return future.result()

    def state(self):
        return self.call(self.state_client, self.GetState.Request()).current_state.label

    def transition(self, transition_id, expected=True):
        req = self.ChangeState.Request()
        req.transition.id = transition_id
        response = self.call(self.change_client, req)
        record = {'id': transition_id, 'success': response.success, 'state': self.state(), 'expected': expected}
        self.transitions.append(record)
        if response.success != expected:
            raise AssertionError('Unexpected lifecycle transition result: ' + str(record))
        return record

    def command(self, vx=.12, wz=0., stamp=None, requested=True):
        msg = self.TwistStamped()
        msg.header.stamp = stamp or self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        if requested:
            self.requested_pub.publish(msg)
            self.pump(.025)
        self.final_pub.publish(msg)
        return msg

    @staticmethod
    def nonzero(msg):
        return len(msg.data) >= 2 and max(abs(x) for x in msg.data[:2]) > 1e-8

    def latest_targets(self, since):
        return [(t, msg) for t, msg in self.targets if t >= since]

    def safe_zero_for(self, duration=.12):
        start = time.monotonic()
        self.pump(duration)
        observed = self.latest_targets(start)
        return bool(observed) and all(not self.nonzero(msg) for _, msg in observed)

    def mean_twist(self, history, since):
        sample = [msg for t, msg in history if t >= since]
        if len(sample) < 8:
            raise AssertionError(f'Only {len(sample)} twist samples')
        return {'samples': len(sample), 'vx': statistics.mean(m.twist.twist.linear.x for m in sample),
                'wz': statistics.mean(m.twist.twist.angular.z for m in sample)}

    def paired_wheel_and_joint(self):
        joints = {stamp_ns(msg.header.stamp): msg for _, msg in self.joints}
        for _, odom in reversed(self.wheel):
            if stamp_ns(odom.header.stamp) in joints:
                return odom, joints[stamp_ns(odom.header.stamp)]
        raise AssertionError('No matching odometry and joint sample stamps')

    def reuse_encoder_checks(self):
        if self.reuse_measured is None:
            return False
        old = json.loads(self.reuse_measured.read_text())
        relevant = lambda files: {p: h for p, h in files.items()
                                  if '/install/omslam_base/' in p or '/install/omslam_control_vendor/' in p}
        before = relevant(old['manifest']['installed_file_sha256'])
        after = relevant(self.result.data['manifest']['installed_file_sha256'])
        if not before or before != after:
            raise ValueError('Cannot reuse encoder checks: installed base/vendor files changed')
        for key in ('measured_encoder_odometry_and_joints', 'duplicate_encoder_samples_not_republished'):
            if old['checks'][key]['status'] != 'PASS':
                raise ValueError('Cannot reuse a failed encoder check: ' + key)
            self.result.data['checks'][key] = old['checks'][key] | {'reused_from': str(self.reuse_measured.resolve())}
            print(key + ': PASS (reused; installed base/vendor hashes unchanged)', flush=True)
        self.result.save()
        return True

    def run_checks(self):
        self.result.check('startup_unconfigured', self.state() == 'unconfigured')
        self.transition(1)  # configure
        self.transition(3, expected=False)  # no encoder feedback
        self.command()
        self.encoder_enabled = True
        self.encoder_previous_time = time.monotonic()
        self.until(lambda: len(self.wheel) >= 3, 3., 'fresh measured encoder odometry')
        self.transition(3)
        self.result.check('activation_needs_feedback_and_fresh_intent', self.state() == 'active' and self.safe_zero_for(),
                          transitions=list(self.transitions))
        self.until(lambda: len(self.filtered) >= 10, 10., 'observed EKF output before measurement window')

        if not self.reuse_encoder_checks():
            # Wheel motion is independent of both commands and synthetic gyro.
            start_odom, start_joint = self.paired_wheel_and_joint()
            self.encoder_speed = .16
            measured_start = time.monotonic()
            self.pump(2.0)
            end_odom, end_joint = self.paired_wheel_and_joint()
            angle_delta = end_joint.position[0]-start_joint.position[0]
            distance = end_odom.pose.pose.position.x-start_odom.pose.pose.position.x
            wheel_mean = self.mean_twist(self.wheel, measured_start+1.)
            filtered_mean = self.mean_twist(self.filtered, measured_start+1.)
            moved_without_command = all(not self.nonzero(msg) for _, msg in self.latest_targets(measured_start))
            frames_ok = end_odom.header.frame_id == 'odom' and end_odom.child_frame_id == 'base_footprint'
            joint_ok = list(end_joint.name) == ['left_wheel_joint', 'right_wheel_joint']
            joint_ok &= all(abs(v-.16/self.RADIUS) < .12 for v in end_joint.velocity)
            same_stamp = stamp_ns(end_joint.header.stamp) == stamp_ns(end_odom.header.stamp)
            covariance_ok = end_odom.twist.covariance[0] > 0 and end_odom.twist.covariance[35] > 0
            self.result.check('measured_encoder_odometry_and_joints', moved_without_command and frames_ok and joint_ok
                              and same_stamp and covariance_ok and distance > .20 and abs(distance-angle_delta*self.RADIUS) < .001
                              and abs(wheel_mean['vx']-.16) < .015 and abs(wheel_mean['wz']) < 1e-8,
                              wheel_mean=wheel_mean, distance_m=distance, joint_angle_delta_rad=angle_delta,
                              joint_velocity_rad_s=list(end_joint.velocity), commanded_motion=False, frames=[end_odom.header.frame_id, end_odom.child_frame_id],
                              matched_joint_odom_stamp=same_stamp)

            self.duplicate_encoder = True
            self.pump(.07)  # drain the last unique sample already in flight
            wheel_count, joint_count = len(self.wheel), len(self.joints)
            self.pump(.16)
            self.result.check('duplicate_encoder_samples_not_republished', len(self.wheel) == wheel_count and len(self.joints) == joint_count,
                              wheel_before=wheel_count, wheel_after=len(self.wheel), joint_before=joint_count, joint_after=len(self.joints))
            self.duplicate_encoder = False
            self.pump(.15)

        else:
            self.encoder_speed = .16
            self.pump(.5)

        rates = []
        for yaw_rate in (.45, -.45):
            self.yaw_rate = yaw_rate
            start = time.monotonic()
            self.pump(1.8)
            rates.append({'input_base_yaw_rate': yaw_rate, 'filtered': self.mean_twist(self.filtered, start+1.),
                          'wheel': self.mean_twist(self.wheel, start+1.)})
        self.yaw_rate = 0.
        corrected = [msg for _, msg in self.conditioned if stamp_ns(msg.header.stamp) in self.raw_imu]
        bias_ok = len(corrected) > 80 and all(msg.header.frame_id == 'imu_link'
                  and abs(msg.angular_velocity.x) < 1e-12 and abs(msg.angular_velocity.z) < 1e-12
                  and abs(msg.angular_velocity.y-(self.raw_imu[stamp_ns(msg.header.stamp)][0]-self.BIAS[1])) < 1e-12
                  and list(msg.angular_velocity_covariance) == self.raw_imu[stamp_ns(msg.header.stamp)][1]
                  and msg.orientation_covariance[0] == -1. for msg in corrected)
        mount = self.static_transforms[('base_link', 'imu_link')].transform.rotation
        mounted = abs(mount.x-math.sqrt(.5)) < 1e-6 and abs(mount.w-math.sqrt(.5)) < 1e-6
        self.result.check('imu_conditioner_preserves_stamp_covariance_and_subtracts_bias', bias_ok,
                          matched_messages=len(corrected), sensor_frame='imu_link', sensor_axis_bias_rad_s=self.BIAS,
                          output_stamps_all_match_published_input=True)
        self.result.check('ekf_consumes_measured_wheel_velocity', all(abs(x['filtered']['vx']-.16) < .025 for x in rates),
                          observations=rates)
        self.result.check('ekf_fuses_independent_gyro_through_tilted_mount', mounted
                          and rates[0]['filtered']['wz'] > .08 and rates[1]['filtered']['wz'] < -.08
                          and all(abs(x['wheel']['wz']) < 1e-8 and abs(x['filtered']['vx']-.16) < .03 for x in rates),
                          mount_roll_rad=math.pi/2, observations=rates)

        self.imu_enabled = False
        self.pump(.05)
        count = len(self.conditioned)
        stale = copy.deepcopy(self.conditioned[-1][1])
        old_ns = self.node.get_clock().now().nanoseconds - 1_000_000_000
        stale.header.stamp.sec, stale.header.stamp.nanosec = divmod(old_ns, 1_000_000_000)
        self.imu_pub.publish(stale)
        self.pump(.10)
        self.result.check('stale_imu_callback_rejected', len(self.conditioned) == count,
                          conditioned_before=count, conditioned_after=len(self.conditioned), injected_age_s=1.)
        self.imu_enabled = True
        self.pump(.05)

        # Re-sending the final message must not extend its original deadline.
        start = time.monotonic()
        msg = self.command()
        self.until(lambda: any(self.nonzero(m) for _, m in self.latest_targets(start)), .18, 'fresh wheel command')
        while time.monotonic()-start < .48:
            self.final_pub.publish(msg)
            self.pump(.025)
        observations = self.latest_targets(start)
        nonzero = [(t, m) for t, m in observations if self.nonzero(m)]
        stopped = [(t, m) for t, m in observations if t > start+.35]
        lease_upper = max((t-start+float(m.data[2]) for t, m in nonzero), default=math.inf)
        self.result.check('original_command_deadline_survives_republication', nonzero and stopped
                          and all(not self.nonzero(m) for _, m in stopped) and lease_upper <= .35,
                          nonzero_samples=len(nonzero), stopped_samples=len(stopped),
                          latest_nonzero_elapsed_s=max(t-start for t, _ in nonzero), max_elapsed_plus_remaining_lease_s=lease_upper)

        start = time.monotonic()
        msg = self.command()
        self.until(lambda: any(self.nonzero(m) for _, m in self.latest_targets(start)), .18, 'second fresh wheel command')
        while time.monotonic()-start < .225:
            self.pump(.005)
        zero_sent = time.monotonic()
        self.command(vx=0., stamp=msg.header.stamp, requested=False)
        self.until(lambda: any(t >= zero_sent and not self.nonzero(m) for t, m in self.targets), .065, 'late zero immediate stop')
        first_zero = next(t for t, m in self.targets if t >= zero_sent and not self.nonzero(m))
        self.final_pub.publish(msg)  # same-stamp nonzero must not resurrect it
        self.result.check('late_zero_stops_and_same_stamp_cannot_rearm', first_zero-start < .30 and self.safe_zero_for(.10),
                          zero_original_age_s=zero_sent-start, observed_stop_delay_s=first_zero-zero_sent,
                          stop_elapsed_s=first_zero-start)

        start = time.monotonic()
        self.command()
        self.until(lambda: any(self.nonzero(m) for _, m in self.latest_targets(start)), .18, 'command before injected motor fault')
        self.fault_pub.publish(self.Bool(data=True))
        self.pump(.10)
        self.fault_pub.publish(self.Bool(data=False))
        self.command()
        fault_stopped = self.safe_zero_for(.10)
        self.until(lambda: self.state() == 'inactive', 2., 'automatic fault deactivation')
        self.transition(3, expected=False)
        self.result.check('motor_fault_latches_and_blocks_reactivation', fault_stopped and self.state() == 'inactive',
                          transitions=self.transitions[-1:])
        self.transition(2)  # cleanup
        self.transition(1)  # configure reconnects mock transport
        self.pump(.15)
        self.transition(3)
        no_replay = self.safe_zero_for(.12)
        start = time.monotonic()
        self.command()
        self.until(lambda: any(self.nonzero(m) for _, m in self.latest_targets(start)), .18, 'new intent after explicit recovery')
        self.result.check('explicit_reconnect_requires_new_command', no_replay,
                          transitions=self.transitions[-3:], fresh_command_observed=True)
        self.transition(4)
        self.result.check('deactivation_stops_output', self.safe_zero_for(.10))
        self.verify_graph()
        self.transition(2)

    def verify_graph(self):
        from rclpy.qos import qos_check_compatible, QoSCompatibility
        endpoints = {}
        for topic in ('/wheel/odometry', '/imu/raw', '/imu/data', '/odom', '/tf', '/tf_static'):
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            endpoints[topic] = {'publishers': [{'node': p.node_name, 'gid': gid_text(p.endpoint_gid),
                                                'reliability': p.qos_profile.reliability.name,
                                                'durability': p.qos_profile.durability.name} for p in pubs],
                                'subscribers': [s.node_name for s in subs]}
            if topic in ('/wheel/odometry', '/imu/data'):
                consumers = [s for s in subs if s.node_name == 'ekf_filter_node']
                compatible = consumers and pubs and all(qos_check_compatible(p.qos_profile, s.qos_profile)[0] != QoSCompatibility.ERROR
                                                        for p in pubs for s in consumers)
                self.result.check('ekf_qos_' + topic.strip('/').replace('/', '_'), bool(compatible), endpoints=endpoints[topic])
        odom_edges = [tf for _, tf in self.transforms
                      if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint']
        tf_publishers = endpoints['/tf']['publishers']
        authorities = [p['gid'] for p in tf_publishers]
        owner = [p['node'] for p in tf_publishers]
        extraneous = [(tf.header.frame_id, tf.child_frame_id) for _, tf in self.transforms
                      if (tf.header.frame_id, tf.child_frame_id) != ('odom', 'base_footprint')]
        self.result.check('single_odom_tf_authority', len(odom_edges) > 100 and len(authorities) == 1
                          and owner == ['ekf_filter_node'] and not extraneous,
                          observed_transforms=len(odom_edges), authorities=authorities, nodes=owner, other_dynamic_edges=extraneous[:20],
                          authority_evidence='one discovered /tf publisher plus observed odom->base_footprint edge')
        topics = dict(self.node.get_topic_names_and_types())
        self.result.check('separate_local_ekf_graph', '/scan' not in topics and '/map' not in topics
                          and len(endpoints['/wheel/odometry']['publishers']) == 1
                          and len(endpoints['/odom']['publishers']) == 1,
                          wheel_publishers=endpoints['/wheel/odometry']['publishers'],
                          odom_publishers=endpoints['/odom']['publishers'], lidar_and_map_topics_present=False)
        self.result.data['graph'] = endpoints
        self.result.data['lifecycle_transitions'] = self.transitions
        self.result.data['sample_counts'] = {'wheel_odometry': len(self.wheel), 'joint_states': len(self.joints),
                                             'filtered_odometry': len(self.filtered), 'conditioned_imu': len(self.conditioned)}
        self.result.save()

    def close(self):
        self.encoder_enabled = self.imu_enabled = False
        if self.process is not None:
            events = []
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                if self.process.poll() is not None:
                    break
                try:
                    os.killpg(self.process.pid, sig)
                    events.append(signal.Signals(sig).name)
                    self.process.wait(timeout=timeout)
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
            self.result.data['shutdown'] = {'signals_to_owned_process_group': events,
                                            'launch_returncode': self.process.poll(), 'pid': self.process.pid}
        if self.log_handle is not None:
            self.log_handle.close()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.rclpy.shutdown(context=self.context)

    def failure_snapshot(self):
        if self.node is None:
            return
        self.result.data['failure_snapshot'] = {
            'sample_counts': {'wheel': len(self.wheel), 'joint': len(self.joints), 'filtered': len(self.filtered),
                              'imu': len(self.conditioned), 'tf': len(self.transforms)},
            'topics': self.node.get_topic_names_and_types(),
            'diagnostics': [{'node': status.name, 'level': str(status.level), 'message': status.message}
                            for _, msg in list(self.diagnostics)[-8:] for status in msg.status],
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--reuse-measured-evidence', type=Path, help='reuse only passed encoder/joint and duplicate checks with unchanged installed base/vendor hashes')
    parser.add_argument('--timeout', type=float, default=100.)
    parser.add_argument('--domain-id', type=int, default=120 + os.getpid() % 70)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('ROS domain ID must be between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    result = Results(args.output)
    runtime = Runtime(result, args.workspace, args.timeout, args.reuse_measured_evidence)
    try:
        runtime.initialize()
        runtime.run_checks()
    except Exception as exc:
        result.data['error'] = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        runtime.failure_snapshot()
        print(traceback.format_exc(), flush=True)
    finally:
        runtime.close()
    result.data['status'] = 'PASS' if 'error' not in result.data and result.data['checks'] and all(
        item['status'] == 'PASS' for item in result.data['checks'].values()) else 'FAIL'
    result.data['finished_at'] = datetime.now().astimezone().isoformat()
    result.save()
    print(f"{result.data['status']}: {result.directory/'results.json'}", flush=True)
    return 0 if result.data['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
