#!/usr/bin/env python3
"""One isolated P2/V2 airport drive, sensor, TF, and simulated-time check.

Requires the latest installed workspace and ROS 2 Jazzy underlay. Starts only
its own Gazebo partition/ROS domain. No serial hardware or existing preview is
used. Physical calibration and actual MCU/PID behavior are outside this test.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PillowImage


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_s(value):
    return float(value.sec) + float(value.nanosec if hasattr(value, 'nanosec') else value.nsec) * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def rotate(q, vector):
    xyz = np.array([q.x, q.y, q.z])
    vector = np.asarray(vector, dtype=float)
    return vector + 2*np.cross(xyz, np.cross(xyz, vector)+q.w*vector)


class Results:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.data = {'status': 'RUNNING', 'started_at': datetime.now().astimezone().isoformat(),
                     'scope': 'P2/V2: Gazebo airport robot drive, LiDAR, IMU, RGB-D, EKF, TF and simulated time',
                     'checks': {}, 'unverified': ['physical hardware, MCU/PID, real sensor noise and calibration',
                                                  'SLAM mapping, AMCL, Nav2']}

    def save(self):
        (self.directory/'results.json').write_text(json.dumps(self.data, indent=2,
            default=lambda x: x.item() if isinstance(x, np.generic) else str(x))+'\n')

    def check(self, name, passed, **evidence):
        self.data['checks'][name] = {'status': 'PASS' if passed else 'FAIL', **evidence}
        print(f"{name}: {'PASS' if passed else 'FAIL'}", flush=True)
        self.save()
        if not passed:
            raise AssertionError(name + ': ' + str(evidence))


class Runtime:
    REQUIRED = ('/clock', '/sim/wheel/odometry_raw', '/sim/ground_truth', '/joint_states',
                '/scan', '/imu/raw', '/imu/data', '/camera/color/image_raw',
                '/camera/depth/image_rect_raw', '/camera/depth/camera_info', '/camera/color/camera_info')

    def __init__(self, result, workspace, timeout, pedestrians=True):
        self.result, self.workspace = result, workspace.resolve()
        self.pedestrians = pedestrians
        self.deadline = time.monotonic()+timeout
        self.process = self.log = self.context = self.node = self.executor = None
        self.gz_node = None
        self.samples = {}
        self.tf_edges = {}
        self.tf_static = {}
        self.gz_stats, self.gz_images = deque(maxlen=5000), {}
        self.robot_world_pose = None
        self.transitions = []
        self.start_wall = time.monotonic()

    @property
    def sim_time(self):
        data = self.samples.get('/clock')
        return stamp_s(data[-1][1].clock) if data else 0.

    def initialize(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, Image, CameraInfo, LaserScan, JointState
        from geometry_msgs.msg import Twist, TwistStamped
        from tf2_msgs.msg import TFMessage
        from lifecycle_msgs.srv import ChangeState, GetState
        from rcl_interfaces.srv import GetParameters
        from diagnostic_msgs.msg import DiagnosticArray
        from tf2_ros import Buffer, TransformListener
        from gz.transport13 import Node as GzNode
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.msgs10.pose_v_pb2 import Pose_V
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from omslam_description.model import simulation_urdf

        self.rclpy, self.ChangeState, self.GetState, self.GetParameters = rclpy, ChangeState, GetState, GetParameters
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.TwistStamped = TwistStamped
        names = ('omslam_description', 'omslam_sim', 'omslam_base', 'omslam_localization')
        prefixes = {name: str(Path(get_package_prefix(name)).resolve()) for name in names}
        imports = {name: importlib.import_module(name).__file__ for name in names}
        self.result.check('installed_artifacts', all(Path(prefixes[name]) == self.workspace/'install'/name
                          and imports[name].startswith(prefixes[name]+'/') for name in names),
                          prefixes=prefixes, imports=imports)
        manifest = {}
        for name in names:
            for path in sorted(Path(prefixes[name]).rglob('*')):
                if path.is_file() and path.suffix in ('.py', '.yaml', '.json', '.sdf', '.urdf', '.so'):
                    manifest[str(path)] = digest(path)
        urdf = simulation_urdf()
        (self.result.directory/'installed_robot.urdf').write_text(urdf)
        root = ET.fromstring(urdf)
        self.expected_tf = {(j.find('parent').get('link'), j.find('child').get('link')): j.get('type')
                            for j in root.findall('joint')}
        self.result.data['manifest'] = {'prefixes': prefixes, 'installed_file_sha256': manifest,
                                        'robot_urdf_sha256': hashlib.sha256(urdf.encode()).hexdigest(),
                                        'verifier_sha256': digest(__file__)}
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node('sim_robot_acceptance', context=self.context,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.pump(.4)
        existing = [name for name in self.node.get_node_names() if name != self.node.get_name()]
        self.result.check('isolated_empty_ros_graph', not existing, existing_nodes=existing,
                          ros_domain_id=os.environ['ROS_DOMAIN_ID'], gz_partition=os.environ['GZ_PARTITION'])
        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        topics = {'/clock': Clock, '/sim/wheel/odometry_raw': Odometry, '/wheel/odometry': Odometry,
                  '/odom': Odometry, '/sim/ground_truth': Odometry, '/joint_states': JointState,
                  '/imu/raw': Imu, '/imu/data': Imu, '/scan': LaserScan,
                  '/camera/color/image_raw': Image, '/camera/depth/image_rect_raw': Image,
                  '/camera/depth/camera_info': CameraInfo, '/camera/color/camera_info': CameraInfo,
                  '/sim/cmd_vel': Twist, '/diagnostics': DiagnosticArray}
        for topic, message_type in topics.items():
            self.samples[topic] = deque(maxlen=6 if message_type is Image else 3000)
            def receive(msg, topic=topic):
                self.samples[topic].append((time.monotonic(), msg))
            qos = clock_qos if topic == '/clock' else qos_profile_sensor_data
            self.node.create_subscription(message_type, topic, receive, qos)
        self.node.create_subscription(TFMessage, '/tf', self.receive_tf, 100)
        self.node.create_subscription(TFMessage, '/tf_static', self.receive_static,
             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.requested_pub = self.node.create_publisher(TwistStamped, '/cmd_vel/requested', qos)
        self.final_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.change_client = self.node.create_client(ChangeState, '/sim_base/change_state')
        self.state_client = self.node.create_client(GetState, '/sim_base/get_state')
        self.gz_node = GzNode()
        self.gz_node.subscribe(WorldStatistics, '/world/airport/stats', lambda msg: self.gz_stats.append(
            (time.monotonic(), stamp_s(msg.sim_time), float(msg.real_time_factor))))
        self.gz_node.subscribe(Pose_V, '/world/airport/pose/info', self.receive_world_poses)
        self.gz_node.subscribe(Pose_V, '/world/airport/dynamic_pose/info', self.receive_world_poses)
        command = ['ros2', 'launch', 'omslam_sim', 'sim.launch.py', 'gui:=false',
                   'pedestrians:=' + str(self.pedestrians).lower()]
        self.log = (self.result.directory/'launch.log').open('w')
        self.process = subprocess.Popen(command, cwd='/tmp', stdout=self.log, stderr=subprocess.STDOUT,
            env=dict(os.environ, ROS_LOG_DIR=str(self.result.directory/'ros_logs')), start_new_session=True)
        self.result.data['launch'] = {'command': command, 'pid': self.process.pid, 'start_new_session': True,
                                      'ROS_DOMAIN_ID': os.environ['ROS_DOMAIN_ID'], 'GZ_PARTITION': os.environ['GZ_PARTITION']}
        self.result.save()
        self.until(lambda: self.change_client.service_is_ready(), 30., 'sim lifecycle service')
        self.until(lambda: all(len(self.samples[key]) >= 2 for key in self.REQUIRED), 100., 'actual clock, wheel, truth and all sensor samples')
        self.result.check('startup_unconfigured_and_sensors_ready', self.state() == 'unconfigured',
                          counts={key: len(self.samples[key]) for key in self.REQUIRED}, sim_time=self.sim_time)
        self.transition(1)
        self.until(lambda: len(self.samples['/wheel/odometry']) >= 5 and len(self.samples['/odom']) >= 10, 15.,
                   'configured measured wheel and EKF odometry')
        self.transition(3)
        self.wait_sim(.6)

    def receive_tf(self, msg):
        for transform in msg.transforms:
            self.tf_edges.setdefault((transform.header.frame_id, transform.child_frame_id), deque(maxlen=2000)).append(transform)

    def receive_static(self, msg):
        for transform in msg.transforms:
            self.tf_static[(transform.header.frame_id, transform.child_frame_id)] = transform

    def receive_world_poses(self, msg):
        for pose in msg.pose:
            if pose.name == 'oh_my_mobilerobot':
                self.robot_world_pose = pose

    def pump(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            if time.monotonic() > self.deadline:
                raise TimeoutError('Overall bounded simulation deadline exceeded')
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError(f'Simulation launch exited {self.process.returncode}')
            self.executor.spin_once(timeout_sec=min(.01, max(0., end-time.monotonic())))

    def until(self, predicate, timeout, reason):
        end, next_note = time.monotonic()+timeout, time.monotonic()+15
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError(reason)
            if time.monotonic() >= next_note:
                print(f'Waiting for {reason}; sim_time={self.sim_time:.3f}', flush=True)
                next_note += 15
            self.pump(.02)

    def wait_sim(self, duration, timeout=40.):
        target = self.sim_time+duration
        self.until(lambda: self.sim_time >= target, timeout, f'{duration:.2f}s simulated interval')

    def call(self, client, request):
        future = client.call_async(request)
        self.until(future.done, 5., 'service response')
        return future.result()

    def state(self):
        return self.call(self.state_client, self.GetState.Request()).current_state.label

    def transition(self, ident):
        req = self.ChangeState.Request()
        req.transition.id = ident
        result = self.call(self.change_client, req)
        entry = {'id': ident, 'success': result.success, 'state': self.state(), 'sim_time': self.sim_time}
        self.transitions.append(entry)
        if not result.success:
            raise AssertionError('Lifecycle transition failed: ' + str(entry))

    def command(self, vx=0., wz=0., old=None, requested=True):
        msg = self.TwistStamped()
        msg.header.stamp = old.header.stamp if old else self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        if requested:
            self.requested_pub.publish(msg)
            self.pump(.015)
        self.final_pub.publish(msg)
        return msg

    def drive(self, vx, wz, duration):
        start = self.sim_time
        last = None
        deadline = time.monotonic()+max(40., duration*15.)
        while self.sim_time < start+duration:
            if time.monotonic() > deadline:
                raise TimeoutError('Drive interval: simulation too slow or stalled')
            last = self.command(vx, wz)
            self.pump(.035)
        return last

    def pose(self, topic='/sim/ground_truth'):
        msg = self.samples[topic][-1][1]
        p = msg.pose.pose
        return np.array([p.position.x, p.position.y, yaw(p.orientation)])

    def joints(self):
        msg = self.samples['/joint_states'][-1][1]
        return {name: float(msg.position[index]) for index, name in enumerate(msg.name)}

    def twist(self, topic, duration=.3):
        cutoff = self.sim_time-duration
        values = [[msg.twist.twist.linear.x, msg.twist.twist.angular.z] for _, msg in self.samples[topic]
                  if stamp_s(msg.header.stamp) >= cutoff]
        if len(values) < 3:
            raise AssertionError('Insufficient fresh twist samples for '+topic)
        return np.mean(values, axis=0).tolist()

    def transform(self, frame):
        from rclpy.time import Time
        return self.buffer.lookup_transform('base_footprint', frame, Time()).transform

    def imu_vector(self, field, duration=.3):
        cutoff = self.sim_time-duration
        values = []
        for _, msg in self.samples['/imu/data']:
            if stamp_s(msg.header.stamp) >= cutoff:
                v = getattr(msg, field)
                values.append(rotate(self.transform(msg.header.frame_id).rotation, [v.x, v.y, v.z]))
        if len(values) < 3:
            raise AssertionError('Insufficient fresh IMU samples')
        return np.mean(values, axis=0).tolist()

    def stationary(self, name):
        self.wait_sim(.7)
        start = self.pose()
        self.wait_sim(.3)
        end = self.pose()
        wheel = self.twist('/wheel/odometry')
        self.result.check(name, np.linalg.norm(end[:2]-start[:2]) < .015
                          and abs(angle_delta(end[2], start[2])) < .035 and abs(wheel[0]) < .025 and abs(wheel[1]) < .06,
                          truth_drift_m=float(np.linalg.norm(end[:2]-start[:2])),
                          truth_yaw_drift_rad=angle_delta(end[2], start[2]), measured_wheel_twist=wheel)

    def run(self):
        gravity = self.imu_vector('linear_acceleration')
        self.result.check('stationary_imu_gravity_in_base', abs(gravity[0]) < 1.2 and abs(gravity[1]) < 1.2
                          and 8.5 < gravity[2] < 10.8, acceleration_m_s2=gravity)
        self.sensor_checks()
        start, wheel_start, ekf_start, joints_start = self.pose(), self.pose('/wheel/odometry'), self.pose('/odom'), self.joints()
        self.drive(.16, 0., 2.0)
        end, wheel_end, ekf_end, joints_end = self.pose(), self.pose('/wheel/odometry'), self.pose('/odom'), self.joints()
        forward = float(np.dot(end[:2]-start[:2], [math.cos(start[2]), math.sin(start[2])]))
        lateral = float(np.dot(end[:2]-start[:2], [-math.sin(start[2]), math.cos(start[2])]))
        wheel_dx = float(wheel_end[0]-wheel_start[0]); ekf_dx = float(ekf_end[0]-ekf_start[0])
        joint_deltas = {key: joints_end[key]-joints_start[key] for key in ('left_wheel_joint', 'right_wheel_joint')}
        self.result.check('forward_truth_wheel_ekf_and_joint_signs', .12 < forward < .45 and abs(lateral) < .08
                          and abs(angle_delta(end[2], start[2])) < .12 and wheel_dx > .12 and ekf_dx > .12
                          and all(value > .5 for value in joint_deltas.values()),
                          truth_forward_m=forward, truth_lateral_m=lateral, truth_yaw_delta_rad=angle_delta(end[2], start[2]),
                          wheel_delta_x_m=wheel_dx, ekf_delta_x_m=ekf_dx, joint_delta_rad=joint_deltas)
        self.command()
        self.stationary('explicit_zero_stops_physics')

        start, joints_start = self.pose(), self.joints()
        self.drive(0., .45, 2.0)
        end, joints_end = self.pose(), self.joints()
        wheel, filtered = self.twist('/wheel/odometry'), self.twist('/odom')
        gyro = self.imu_vector('angular_velocity')
        joint_deltas = {key: joints_end[key]-joints_start[key] for key in ('left_wheel_joint', 'right_wheel_joint')}
        turn = angle_delta(end[2], start[2])
        self.result.check('left_turn_truth_imu_ekf_and_joint_signs', .25 < turn < 1.2 and wheel[1] > .15
                          and filtered[1] > .15 and gyro[2] > .15
                          and joint_deltas['left_wheel_joint'] < -.3 and joint_deltas['right_wheel_joint'] > .3,
                          truth_yaw_delta_rad=turn, wheel_twist=wheel, ekf_twist=filtered,
                          base_gyro_rad_s=gyro, joint_delta_rad=joint_deltas)
        # Keep republishing the same final message after its original source stops.
        original = self.command(0., .45)
        stopped_source = time.monotonic()
        while time.monotonic()-stopped_source < .5:
            self.command(0., .45, old=original, requested=False)
            self.pump(.025)
        target = [(t, m) for t, m in self.samples['/sim/cmd_vel'] if t >= stopped_source+.36]
        self.result.check('original_command_timeout_not_extended_by_final_replay', target
                          and all(abs(m.linear.x)+abs(m.angular.z) < 1e-9 for _, m in target),
                          output_samples_after_deadline=len(target), observation_after_wall_s=.36)
        self.stationary('command_timeout_stops_physics')

        old = self.drive(.12, 0., .65)
        self.pause(True)
        self.pump(.5)
        paused_clock = self.sim_time
        cutoff = time.monotonic()
        self.pump(.15)
        targets = [m for t, m in self.samples['/sim/cmd_vel'] if t >= cutoff]
        self.result.check('pause_stops_output_with_frozen_sim_clock', self.sim_time == paused_clock and targets
                          and all(abs(m.linear.x)+abs(m.angular.z) < 1e-9 for m in targets),
                          frozen_sim_time=paused_clock, zero_outputs_while_paused=len(targets))
        self.pause(False)
        self.wait_sim(.1)
        self.command(.12, 0., old=old)
        no_rearm_start = time.monotonic()
        self.wait_sim(.55)
        targets = [m for t, m in self.samples['/sim/cmd_vel'] if t >= no_rearm_start]
        self.result.check('resume_rejects_pre_pause_intent', targets
                          and all(abs(m.linear.x)+abs(m.angular.z) < 1e-9 for m in targets),
                          observed_zero_outputs=len(targets), replayed_original_stamp_s=stamp_s(old.header.stamp))
        self.stationary('resume_remains_stationary_until_fresh_intent')
        start = self.pose()
        self.drive(.12, 0., 1.0)
        end = self.pose()
        displacement = float(np.dot(end[:2]-start[:2], [math.cos(start[2]), math.sin(start[2])]))
        self.result.check('new_post_resume_intent_moves_robot', displacement > .04, forward_displacement_m=displacement)
        self.command()
        self.stationary('final_zero_stops_physics')
        self.graph_checks()
        self.capture_robot()
        self.transition(4)
        self.transition(2)
        self.result.data['lifecycle_transitions'] = self.transitions

    def sensor_checks(self):
        scan = self.samples['/scan'][-1][1]
        ranges = np.asarray(scan.ranges)
        finite = ranges[np.isfinite(ranges)]
        self.result.check('lidar_returns_and_frame', scan.header.frame_id == 'laser_frame' and len(ranges) == 720
                          and len(finite) >= 20 and finite.min() >= .14 and finite.max() <= 12.02 and not np.isnan(ranges).any(),
                          frame=scan.header.frame_id, rays=len(ranges), finite_returns=len(finite),
                          min_m=float(finite.min()) if len(finite) else None, max_m=float(finite.max()) if len(finite) else None)
        rgb = self.samples['/camera/color/image_raw'][-1][1]
        depth = self.samples['/camera/depth/image_rect_raw'][-1][1]
        info = self.samples['/camera/depth/camera_info'][-1][1]
        color_info = self.samples['/camera/color/camera_info'][-1][1]
        self.result.check('rgbd_dimensions_encoding_intrinsics', rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
                          and (rgb.width, rgb.height) == (depth.width, depth.height) == (info.width, info.height) == (320, 240)
                          and info.k[0] > 100 and info.k[4] > 100 and 0 < info.k[2] < info.width and 0 < info.k[5] < info.height
                          and list(color_info.k) == list(info.k),
                          rgb_encoding=rgb.encoding, depth_encoding=depth.encoding, size=[rgb.width, rgb.height],
                          camera_matrix=list(info.k), depth_unit='metres (32FC1)')
        dtype = np.dtype('>f4' if depth.is_bigendian else '<f4')
        values = np.frombuffer(depth.data, dtype=dtype).reshape(depth.height, depth.step//4)[:, :depth.width]
        valid = values[np.isfinite(values) & (values > 0)]
        image = PillowImage.frombytes('RGB', (rgb.width, rgb.height), bytes(rgb.data), 'raw', 'RGB', rgb.step)
        image.save(self.result.directory/'onboard_rgb.png')
        np.save(self.result.directory/'depth_metres.npy', values)
        self.result.check('rgbd_valid_measured_buffers', len(valid) > values.size*.05 and valid.min() >= .1
                          and valid.max() <= 5.01 and float(np.asarray(image).std()) > 5.,
                          finite_depth_fraction=float(len(valid)/values.size), min_depth_m=float(valid.min()) if len(valid) else None,
                          max_depth_m=float(valid.max()) if len(valid) else None, rgb_pixel_std=float(np.asarray(image).std()),
                          rgb_image=str(self.result.directory/'onboard_rgb.png'), depth_array=str(self.result.directory/'depth_metres.npy'))
        optical = self.transform('camera_optical_frame').rotation
        basis = [rotate(optical, axis).tolist() for axis in np.eye(3)]
        self.result.check('sensor_frames_and_optical_axes', rgb.header.frame_id == depth.header.frame_id == info.header.frame_id
                          == color_info.header.frame_id == 'camera_optical_frame'
                          and np.allclose(basis, [[0, -1, 0], [0, 0, -1], [1, 0, 0]], atol=1e-6),
                          frames=[rgb.header.frame_id, depth.header.frame_id, info.header.frame_id],
                          optical_xyz_axes_in_base=basis)
        self.stamp_check()

    def stamp_check(self):
        observed = {}
        for topic in self.REQUIRED[1:]+('/wheel/odometry', '/odom'):
            data = list(self.samples[topic])
            stamps = [stamp_s(msg.header.stamp) for _, msg in data]
            observed[topic] = {'samples': len(stamps), 'first': stamps[0], 'last': stamps[-1],
                               'age_sim_s': self.sim_time-stamps[-1],
                               'monotonic': all(b >= a for a, b in zip(stamps, stamps[1:]))}
        self.result.check('sensor_and_odometry_stamps_follow_clock', all(v['samples'] >= 2 and v['first'] > 0
                          and v['last'] > v['first'] and -.2 <= v['age_sim_s'] < .8 and v['monotonic'] for v in observed.values()),
                          sim_time=self.sim_time, topics=observed)

    def graph_checks(self):
        from rclpy.qos import qos_check_compatible, QoSCompatibility
        nodes = ('sim_base', 'robot_state_publisher', 'robot_sensor_bridge', 'airport_clock_bridge', 'imu_conditioner', 'ekf_filter_node')
        params = {}
        for name in nodes:
            client = self.node.create_client(self.GetParameters, '/'+name+'/get_parameters')
            self.until(client.service_is_ready, 3., name+' parameters')
            response = self.call(client, self.GetParameters.Request(names=['use_sim_time']))
            params[name] = response.values[0].bool_value
            self.node.destroy_client(client)
        self.result.check('all_runtime_nodes_use_sim_time', all(params.values()), nodes=params)
        endpoints = {}
        for topic in ('/clock', '/tf', '/tf_static', '/wheel/odometry', '/odom', '/imu/raw', '/imu/data', '/scan',
                      '/sim/ground_truth', '/camera/color/image_raw', '/camera/depth/image_rect_raw'):
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            endpoints[topic] = {'publishers': [p.node_name for p in pubs], 'subscribers': [s.node_name for s in subs],
                                'publisher_qos': [{'reliability': p.qos_profile.reliability.name,
                                                   'durability': p.qos_profile.durability.name} for p in pubs]}
            if topic in ('/wheel/odometry', '/imu/data'):
                consumers = [s for s in subs if s.node_name == 'ekf_filter_node']
                matches = [qos_check_compatible(p.qos_profile, s.qos_profile)[0] for p in pubs for s in consumers]
                self.result.check('ekf_qos_'+topic.strip('/').replace('/', '_'), matches
                                  and all(x == QoSCompatibility.OK for x in matches), endpoints=endpoints[topic])
        expected_dynamic = {edge for edge, kind in self.expected_tf.items() if kind != 'fixed'} | {('odom', 'base_footprint')}
        expected_static = {edge for edge, kind in self.expected_tf.items() if kind == 'fixed'}
        parents = {}
        for parent, child in set(self.tf_edges) | set(self.tf_static):
            parents.setdefault(child, set()).add(parent)
        self.result.check('single_tf_authorities_and_complete_robot_tree', set(self.tf_edges) == expected_dynamic
                          and set(self.tf_static) == expected_static and all(len(p) == 1 for p in parents.values())
                          and sorted(endpoints['/tf']['publishers']) == ['ekf_filter_node', 'robot_state_publisher']
                          and endpoints['/tf_static']['publishers'] == ['robot_state_publisher'],
                          dynamic_edges=[list(x) for x in sorted(self.tf_edges)], static_edges=[list(x) for x in sorted(self.tf_static)],
                          tf_publishers=endpoints['/tf']['publishers'], static_publishers=endpoints['/tf_static']['publishers'],
                          evidence='Installed URDF owns robot joints; EKF owns odom edge; no Gazebo TF is bridged')
        self.result.check('one_clock_and_separate_truth', len(endpoints['/clock']['publishers']) == 1
                          and endpoints['/wheel/odometry']['publishers'] == ['sim_base']
                          and endpoints['/odom']['publishers'] == ['ekf_filter_node']
                          and endpoints['/sim/ground_truth']['subscribers'] == [self.node.get_name()], endpoints=endpoints)
        self.result.data['graph'] = endpoints

    def pause(self, paused):
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.boolean_pb2 import Boolean
        ok, response = self.gz_node.request('/world/airport/control', WorldControl(pause=paused), WorldControl, Boolean, 3000)
        if not ok or not response.data:
            raise RuntimeError('Gazebo pause service failed')

    def capture_robot(self):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image
        self.until(lambda: self.robot_world_pose is not None, 10., 'Gazebo robot world pose for rendered capture')
        robot = self.robot_world_pose
        heading = yaw(robot.orientation)
        dx, dy = 1.2*math.cos(heading)-1.0*math.sin(heading), 1.2*math.sin(heading)+1.0*math.cos(heading)
        pose = [robot.position.x+dx, robot.position.y+dy, .95, 0., .43, math.atan2(-dy, -dx)]
        topic = '/airport/verification/robot/image'
        self.gz_images[topic] = deque(maxlen=3)
        self.gz_node.subscribe(Image, topic, lambda msg: self.gz_images[topic].append(msg))
        sdf = f'''<sdf version="1.9"><model name="verification_robot_camera"><static>true</static>
          <pose>{' '.join(map(str, pose))}</pose><link name="camera_link"><sensor name="camera" type="camera">
          <always_on>true</always_on><update_rate>1</update_rate><topic>{topic}</topic><camera><horizontal_fov>0.9</horizontal_fov>
          <image><width>960</width><height>720</height><format>R8G8B8</format></image><clip><near>0.05</near><far>100</far></clip>
          </camera></sensor></link></model></sdf>'''
        ok, response = self.gz_node.request('/world/airport/create', EntityFactory(sdf=sdf, name='verification_robot_camera', allow_renaming=False),
                                            EntityFactory, Boolean, 5000)
        if not ok or not response.data:
            raise RuntimeError('Could not create rendered robot evidence camera')
        self.until(lambda: self.gz_images[topic] and np.frombuffer(self.gz_images[topic][-1].data, dtype=np.uint8).std() > 5,
                   40., 'rendered robot camera frame')
        msg = self.gz_images[topic][-1]
        path = self.result.directory/'robot_in_airport.png'
        PillowImage.frombytes('RGB', (msg.width, msg.height), msg.data, 'raw', 'RGB', msg.step).save(path)
        self.result.check('rendered_robot_evidence', path.is_file(), image=str(path), camera_pose=pose,
                          robot_world_pose=[robot.position.x, robot.position.y, robot.position.z, heading],
                          sim_stamp_s=stamp_s(msg.header.stamp))
        self.result.data['visual_review'] = {'status': 'PENDING', 'required': 'Inspect robot CAD body, wheels, lidar and front camera in the actual rendered image'}

    def snapshot(self):
        self.result.data['sample_counts'] = {key: len(value) for key, value in self.samples.items()}
        self.result.data['observed_sim_time_s'] = self.sim_time
        self.result.data['wall_duration_s'] = time.monotonic()-self.start_wall
        if self.gz_stats:
            first, last = self.gz_stats[0], self.gz_stats[-1]
            self.result.data['observed_real_time_factor'] = (last[1]-first[1])/max(last[0]-first[0], 1e-9)
        if self.node is not None:
            self.result.data['discovered_topics'] = self.node.get_topic_names_and_types()
        diagnostics = self.samples.get('/diagnostics', [])
        self.result.data['last_diagnostics'] = [{'name': s.name, 'level': str(s.level), 'message': s.message}
                                               for _, msg in list(diagnostics)[-6:] for s in msg.status]

    def close(self):
        if self.process is not None:
            events = []
            if self.process.poll() is None:
                try:
                    from gz.msgs10.server_control_pb2 import ServerControl
                    from gz.msgs10.boolean_pb2 import Boolean
                    self.pause(True)
                    time.sleep(1.)  # settle render work before destroying skinned actors/cameras
                    ok, response = self.gz_node.request('/server_control', ServerControl(stop=True), ServerControl, Boolean, 3000)
                    events.append({'transport_stop': bool(ok and response.data)})
                    self.process.wait(timeout=15.)
                except Exception as exc:
                    events.append({'transport_shutdown_fallback': str(exc)})
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
                if self.process.poll() is not None:
                    break
                try:
                    os.killpg(self.process.pid, sig)
                    events.append({'signal_to_owned_group': signal.Signals(sig).name})
                    self.process.wait(timeout=timeout)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
            self.result.data['shutdown'] = {'events': events, 'launch_returncode': self.process.poll(), 'pid': self.process.pid}
        if self.log is not None:
            self.log.close()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.rclpy.shutdown(context=self.context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--domain-id', type=int, default=173)
    parser.add_argument('--timeout', type=float, default=240.)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    os.environ['GZ_PARTITION'] = 'omslam_sim_robot_verify_'+str(os.getpid())
    os.environ['OMSLAM_GZ_PARTITION'] = os.environ['GZ_PARTITION']
    result = Results(args.output)
    runtime = Runtime(result, args.workspace, args.timeout)
    try:
        runtime.initialize()
        runtime.run()
    except Exception as exc:
        result.data['error'] = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        print(traceback.format_exc(), flush=True)
    finally:
        runtime.snapshot()
        runtime.close()
    result.data['status'] = 'AUTOMATED_PASS_VISUAL_REVIEW_REQUIRED' if 'error' not in result.data and all(
        entry['status'] == 'PASS' for entry in result.data['checks'].values()) else 'FAIL'
    result.data['finished_at'] = datetime.now().astimezone().isoformat()
    result.save()
    print(result.data['status']+': '+str(result.directory/'results.json'), flush=True)
    return 0 if result.data['status'].startswith('AUTOMATED_PASS') else 1


if __name__ == '__main__':
    sys.exit(main())
