"""Fail-closed Nav2 command chain and explicit simulation driving permission."""

from dataclasses import dataclass
from collections import OrderedDict, deque
from copy import deepcopy
from functools import partial
import json
import math
import signal
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from omm_perception_msgs.msg import MemoryStatus, PerceptionHealth
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from omslam_sim.control import SIM_MAX_LINEAR_VELOCITY

from .guard_core import Limits, NavigationGate
from .operator_core import OperatorGate


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def twist_values(message):
    linear, angular = message.twist.linear, message.twist.angular
    return (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)


def quaternion_valid(q):
    values = (q.x, q.y, q.z, q.w)
    return all(math.isfinite(x) for x in values) and abs(sum(x*x for x in values) - 1.0) < 0.02


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))


@dataclass
class Observation:
    stamp: int
    received: float
    valid: bool


class NavigationGuard(Node):
    def __init__(self):
        super().__init__('navigation_guard')
        defaults = {
            'localization_mode': 'slam', 'base_frame': 'base_footprint',
            'odom_frame': 'odom', 'map_frame': 'map', 'scan_frame': 'laser_frame',
            'scan_timeout': 0.30, 'depth_timeout': 0.30, 'health_timeout': 0.30,
            'odom_timeout': 0.20, 'wheel_timeout': 0.40, 'map_tf_timeout': 0.50,
            'clock_stall_timeout': 0.30, 'recovery_healthy_time': 1.0,
            'service_timeout': 2.0, 'readiness_timeout': 30.0,
            'map_jump_translation': 0.10, 'map_jump_rotation': 0.10,
            'max_linear_velocity': SIM_MAX_LINEAR_VELOCITY, 'max_angular_velocity': 0.60,
            'future_tf_tolerance': 1.0,
            'operator_enabled': False,
            'keepout_file': '',
            'manual_reverse_enabled': False,
            'rear_observation_verified': False,
            'lifecycle_nodes': ['collision_monitor', 'velocity_smoother', 'planner_server',
                                'controller_server', 'behavior_server', 'bt_navigator'],
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('P4 navigation_guard requires use_sim_time=true')
        if self.p['localization_mode'] not in ('slam', 'amcl'):
            raise ValueError('localization_mode must be slam or amcl')
        for key, value in self.p.items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        if (self.p['max_linear_velocity'] > SIM_MAX_LINEAR_VELOCITY or
                self.p['max_angular_velocity'] > 0.60):
            raise ValueError('Simulation velocity limits exceed the configured simulation wheel limit')
        if self.p['manual_reverse_enabled'] and not (
                self.p['operator_enabled'] and self.p['rear_observation_verified']):
            raise ValueError('manual reverse requires an explicitly verified rear-observation profile')
        limits = Limits(
            clock_timeout=self.p['clock_stall_timeout'],
            recovery_time=self.p['recovery_healthy_time'],
            map_translation=self.p['map_jump_translation'],
            map_rotation=self.p['map_jump_rotation'],
            max_vx=self.p['max_linear_velocity'], max_wz=self.p['max_angular_velocity'],
            frame=self.p['base_frame'])
        self.core = (OperatorGate(limits, reverse_allowed=self.p['manual_reverse_enabled'])
                     if self.p['operator_enabled'] else NavigationGate(limits))
        if self.p['operator_enabled']:
            self.core.stop_deadlines = {k:self.p[k + '_timeout'] for k in ('wheel','odom')}
        self.operator = None
        self.observations = {}
        self.future_observations = {}
        self.depth_history = OrderedDict()
        self.pending_perception = {}
        self.depth_geometry = None
        self.camera_geometry = None
        self.scan_pending = deque(maxlen=16)
        self.scan_tf = None
        self.map_valid = False
        self.map_grid = None
        self.map_tf = None
        self.tf_received = {'map': None, 'odom': None}
        self.tf_publishers = []
        self.graph_missing = ['ROS graph has not been inspected']
        self.graph_missing_by_mode = {'MANUAL': list(self.graph_missing), 'AUTO': list(self.graph_missing)}
        self.missing = ['initializing']
        self.memory_observation = None
        self.health_observation = None
        self.follow_started = None
        self.last_original_stamp = 0
        self.started = time.monotonic()
        self.last_graph_check = self.last_poll = self.last_report = 0.0
        self.last_state_signature = None
        self.startup_timeout_reported = False
        self.operation = None
        self.base_transition = None
        self.cancel_call = None
        self.cancel_retry_at = 0.
        self.cancel_attempted = None
        self.stop_error = ''
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)

        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.requested_pub = self.create_publisher(TwistStamped, '/cmd_vel/requested', 1)
        self.final_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.state_pub = self.create_publisher(String, '/navigation/state', transient)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(TwistStamped, '/cmd_vel/nav', self.on_requested, 1)
        self.create_subscription(TwistStamped, '/cmd_vel/monitored', self.on_final, 1)
        self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # Match the simulation publisher and perception: dropped best-effort
        # frames otherwise break original-stamp correlation between consumers.
        depth_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.on_depth, depth_qos)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_camera, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda msg: self.on_odom('odom', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odometry', lambda msg: self.on_odom('wheel', msg), qos_profile_sensor_data)
        self.create_subscription(PerceptionHealth, '/perception/health', self.on_health, transient)
        self.create_subscription(MemoryStatus, '/perception/memory_status', self.on_memory, transient)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, transient)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self.on_static_tf, static_qos)
        self.create_subscription(GoalStatusArray, '/follow_path/_action/status', self.on_follow_status, transient)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, transient)

        backend = ['slam_toolbox'] if self.p['localization_mode'] == 'slam' else ['map_server', 'amcl']
        self.servers = list(dict.fromkeys(self.p['lifecycle_nodes'] + backend))
        self.states = {name: (State.PRIMARY_STATE_UNKNOWN, 0.0) for name in [*self.servers, 'sim_base']}
        self.state_epochs = {name: 0 for name in self.states}
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state') for name in self.states}
        self.state_calls = {}
        for name in self.states:
            self.create_subscription(TransitionEvent, f'/{name}/transition_event',
                                     partial(self.on_transition, name), 10)
        self.base_client = self.create_client(ChangeState, '/sim_base/change_state')
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        # A suspended service callback must not hold the sensor/timer group.
        self.service_group = ReentrantCallbackGroup()
        self.enable_service = self.create_service(SetBool, '/navigation/enable', self.on_enable,
                                                   callback_group=self.service_group)
        if self.p['operator_enabled']:
            from .operator_control import OperatorControl
            self.operator = OperatorControl(self)
        from .keepouts import KeepoutManager
        self.keepouts = KeepoutManager(self)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.reject_parameter_updates)

    def reject_parameter_updates(self, parameters):
        return SetParametersResult(successful=False, reason='restart navigation to change its contracts')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def remember(self, key, stamp, valid, received=None):
        received = time.monotonic() if received is None else received
        if valid and stamp > self.now_ns():
            # /clock and sensor messages also use independent DDS streams.
            # A future sample cannot replace a still-fresh verified sample.
            self.future_observations[key] = Observation(stamp, received, True)
            return
        self.future_observations.pop(key, None)
        old = self.observations.get(key)
        if old is not None and stamp <= old.stamp:
            if not valid:
                old.valid = False
            return
        self.observations[key] = Observation(stamp, received, bool(valid))

    def fresh(self, key, limit, ros_ns, wall):
        observation = self.observations.get(key)
        return (observation is not None and observation.valid and observation.stamp > 0 and
                0 <= (ros_ns - observation.stamp) * 1e-9 < limit and
                0 <= wall - observation.received < limit)

    def on_clock(self, message):
        if self.core.clock(stamp_ns(message.clock), time.monotonic()):
            # tf2's default callback only covers jumps of at least one second.
            # Explicitly clear dynamic history for every observed rewind.
            self.follow_started = None
            self.tf_buffer.clear()
            self.observations.clear()
            self.future_observations.clear()
            self.depth_history.clear()
            self.pending_perception.clear()
            self.scan_pending.clear()
            self.scan_tf = self.map_tf = None
            self.memory_observation = self.health_observation = None
            self.tf_received = {'map': None, 'odom': None}
            self.map_valid = False
            self.map_grid = None
            if self.operator is not None:
                self.operator.localization_initialized = False
                self.operator.session_start_ns = stamp_ns(message.clock)
            self.last_original_stamp = 0
            self.publish_zero()

    def on_scan(self, message):
        valid = (message.header.frame_id == self.p['scan_frame'] and len(message.ranges) > 0 and
                 all(math.isfinite(x) for x in (message.angle_min, message.angle_max,
                     message.angle_increment, message.range_min, message.range_max)) and
                 message.angle_increment > 0 and 0 < message.range_min < message.range_max and
                 any((math.isfinite(x) and message.range_min <= x <= message.range_max) or
                     x == math.inf for x in message.ranges))
        stamp = stamp_ns(message.header.stamp)
        self.remember('scan', stamp, valid)
        if valid and (not self.scan_pending or stamp > self.scan_pending[-1][0]):
            self.scan_pending.append((stamp, message.header.frame_id, time.monotonic()))

    def on_depth(self, message):
        bytes_per_pixel = {'32FC1': 4, '16UC1': 2}.get(message.encoding, 0)
        valid = (message.header.frame_id == 'camera_optical_frame' and bytes_per_pixel > 0 and
                 message.width > 0 and message.height > 0 and
                 message.step >= message.width * bytes_per_pixel and
                 len(message.data) == message.step * message.height)
        self.depth_geometry = (message.width, message.height, message.header.frame_id)
        stamp, received = stamp_ns(message.header.stamp), time.monotonic()
        self.remember('depth', stamp, valid, received)
        if valid and stamp not in self.depth_history:
            self.depth_history[stamp] = received
            while len(self.depth_history) > 32:
                self.depth_history.popitem(last=False)
            self.match_perception()

    def match_perception(self):
        for key, stamp in list(self.pending_perception.items()):
            received = self.depth_history.get(stamp)
            if received is not None:
                self.remember(key + '_observation', stamp, True, received)
                del self.pending_perception[key]

    def perception_observed(self, key, stamp, valid):
        if valid:
            self.pending_perception[key] = stamp
            self.match_perception()
        else:
            self.pending_perception.pop(key, None)
            self.future_observations.pop(key + '_observation', None)
            if key + '_observation' in self.observations:
                self.observations[key + '_observation'].valid = False

    def on_camera(self, message):
        valid = (message.header.frame_id == 'camera_optical_frame' and
                 message.width > 0 and message.height > 0 and
                 all(math.isfinite(x) for x in message.k) and message.k[0] > 0 and message.k[4] > 0)
        self.camera_geometry = (message.width, message.height, message.header.frame_id)
        self.remember('camera', stamp_ns(message.header.stamp), valid)

    def on_imu(self, message):
        valid = (message.header.frame_id == 'imu_link' and message.orientation_covariance[0] >= 0 and
                 quaternion_valid(message.orientation) and
                 all(math.isfinite(x) for x in (message.angular_velocity.x,
                     message.angular_velocity.y, message.angular_velocity.z)))
        self.remember('imu', stamp_ns(message.header.stamp), valid)

    def on_odom(self, key, message):
        pose, twist = message.pose.pose, message.twist.twist
        valid = (message.header.frame_id == self.p['odom_frame'] and
                 message.child_frame_id == self.p['base_frame'] and quaternion_valid(pose.orientation) and
                 all(math.isfinite(x) for x in (pose.position.x, pose.position.y,
                     twist.linear.x, twist.linear.y, twist.angular.z)))
        self.remember(key, stamp_ns(message.header.stamp), valid)
        if self.operator is not None:
            self.core.sample_stop(key, stamp_ns(message.header.stamp), math.hypot(twist.linear.x, twist.linear.y),
                                  twist.angular.z, valid, time.monotonic())

    def on_health(self, message):
        snapshot, observed = stamp_ns(message.stamp), stamp_ns(message.observation_stamp)
        valid = (message.sensor_healthy and message.geometry_healthy and message.memory_healthy and
                 0 < observed <= snapshot)
        self.health_observation = observed
        self.remember('health', snapshot, valid)
        self.perception_observed('health', observed, valid)

    def on_memory(self, message):
        snapshot, observed = stamp_ns(message.header.stamp), stamp_ns(message.observation_stamp)
        valid = (message.header.frame_id == self.p['odom_frame'] and message.sensor_healthy and
                 message.geometry_healthy and message.memory_healthy and 0 < observed <= snapshot and
                 math.isfinite(message.voxel_size) and message.voxel_size > 0)
        self.memory_observation = observed
        self.remember('memory', snapshot, valid)
        self.perception_observed('memory', observed, valid)

    def on_map(self, message):
        self.map_valid = (message.header.frame_id == self.p['map_frame'] and
                          message.info.width > 0 and message.info.height > 0 and
                          math.isfinite(message.info.resolution) and message.info.resolution > 0 and
                          len(message.data) == message.info.width * message.info.height and
                          quaternion_valid(message.info.origin.orientation))
        self.map_grid = message if self.map_valid else None

    def on_static_tf(self, message):
        for transform in message.transforms:
            try:
                self.tf_buffer.set_transform_static(transform, 'robot_state_publisher')
            except (TransformException, ValueError) as error:
                self.get_logger().error(f'invalid static transform: {error}', throttle_duration_sec=2.0)

    def on_tf(self, message):
        now, ros_ns = time.monotonic(), self.now_ns()
        for transform in message.transforms:
            try:
                # Jazzy rclpy does not expose a sender GID in MessageInfo.
                # inspect_graph checks the fixed launch's allowed endpoints;
                # this callback independently checks edge data and freshness.
                self.tf_buffer.set_transform(transform, 'configured_tf_graph')
            except (TransformException, ValueError):
                continue
            parent, child = transform.header.frame_id, transform.child_frame_id
            if parent == self.p['odom_frame'] and child == self.p['base_frame']:
                self.tf_received['odom'] = now
            if parent != self.p['map_frame'] or child != self.p['odom_frame']:
                continue
            translation, rotation = transform.transform.translation, transform.transform.rotation
            stamp = stamp_ns(transform.header.stamp)
            if (not quaternion_valid(rotation) or
                    not all(math.isfinite(x) for x in (translation.x, translation.y, translation.z)) or
                    not -self.p['map_tf_timeout'] <= (stamp - ros_ns) * 1e-9 <= self.p['future_tf_tolerance'] + 0.02):
                continue
            self.map_tf = (stamp, now)
            self.tf_received['map'] = now
            if not self.core.map_transform(translation.x, translation.y, quaternion_yaw(rotation)):
                self.publish_zero()

    def on_status(self, message):
        old_state, old_goal = self.core.state, self.core.goal_id
        self.core.goals([(bytes(item.goal_info.goal_id.uuid).hex(),
                          stamp_ns(item.goal_info.stamp), item.status)
                         for item in message.status_list], self.now_ns(), time.monotonic())
        if old_goal != self.core.goal_id or (old_state == 'RUNNING' and self.core.goal_status != 2):
            self.publish_zero()

    def on_follow_status(self, message):
        # The outer goal remains EXECUTING during CancelControl and Wait.
        # Only FollowPath can supply motion in this navigation tree.
        executing = [stamp_ns(item.goal_info.stamp) for item in message.status_list
                     if item.status == GoalStatus.STATUS_EXECUTING]
        started = max(executing, default=None)
        if started != self.follow_started:
            self.follow_started = started
            if self.core.goal_id is not None:
                self.core._discard_commands()
                self.publish_zero()

    def on_requested(self, message):
        stamp = stamp_ns(message.header.stamp)
        if (self.follow_started is None or self.follow_started < self.core.goal_stamp
                or stamp < self.follow_started):
            self.core.rejected_sources += 1
            return
        wall, ros_ns = time.monotonic(), self.now_ns()
        if self.core.requested(stamp_ns(message.header.stamp), message.header.frame_id,
                               twist_values(message), ros_ns, wall):
            self.last_original_stamp = stamp_ns(message.header.stamp)
            output = message
            if self.operator is not None:
                output = deepcopy(message)
                limited = self.core.limit_values(twist_values(message))
                output.twist.linear.x, output.twist.angular.z = limited[0], limited[5]
            self.requested_pub.publish(output)
        elif not self.core.armed:
            self.publish_zero()

    def on_final(self, message):
        # Keep original frame/axis validation. Only valid commands can be capped;
        # min/max operations must never turn NaN or an invalid axis into input.
        if self.operator is not None and self.core._valid_twist(message.header.frame_id, twist_values(message)):
            message = deepcopy(message)
            limited = self.core.limit_final(stamp_ns(message.header.stamp), twist_values(message))
            message.twist.linear.x, message.twist.angular.z = limited[0], limited[5]
        if self.core.final(stamp_ns(message.header.stamp), message.header.frame_id,
                           twist_values(message), self.now_ns(), time.monotonic()):
            if message.header.frame_id == self.p['base_frame']:
                if (hasattr(self, 'keepouts') and not self.keepouts.command_allowed(
                        message.twist.linear.x, message.twist.angular.z)):
                    self.core.fault('진입 금지 구역 접근 차단 또는 지도 위치 확인 불가')
                    self.publish_zero()
                    return
                self.final_pub.publish(message)
            else:
                # Only an all-zero timeout can pass without the base frame.
                # Emit the existing stop form; never relabel a moving command.
                self.publish_zero()
        else:
            # Dropping an unmatched nonzero cannot resurrect a previous intent.
            if not self.core.armed:
                self.publish_zero()

    def publish_zero(self):
        message = TwistStamped()
        stamp = self.last_original_stamp or max(0, self.now_ns())
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = self.p['base_frame']
        self.final_pub.publish(message)

    def on_transition(self, name, message):
        self.state_epochs[name] += 1
        self.states[name] = (message.goal_state.id, time.monotonic())

    def poll_states(self, wall):
        for name, (future, started, epoch) in list(self.state_calls.items()):
            if future.done():
                try:
                    result = future.result()
                    if self.state_epochs[name] == epoch:
                        self.states[name] = (result.current_state.id, wall)
                except Exception:
                    if self.state_epochs[name] == epoch:
                        self.states[name] = (State.PRIMARY_STATE_UNKNOWN, wall)
                del self.state_calls[name]
            elif wall - started > self.p['service_timeout']:
                self.state_clients[name].remove_pending_request(future)
                if self.state_epochs[name] == epoch:
                    self.states[name] = (State.PRIMARY_STATE_UNKNOWN, wall)
                del self.state_calls[name]
        if wall - self.last_poll < 0.5:
            return
        self.last_poll = wall
        for name, client in self.state_clients.items():
            if name not in self.state_calls and client.service_is_ready():
                self.state_calls[name] = (client.call_async(GetState.Request()), wall, self.state_epochs[name])

    def request_base_transition(self, transition, destination, wall):
        if self.base_transition is not None or not self.base_client.service_is_ready():
            return False
        request = ChangeState.Request()
        request.transition.id = transition
        self.base_transition = (self.base_client.call_async(request), wall, destination)
        return True

    def manage_base(self, wall):
        if self.base_transition is not None:
            future, started, destination = self.base_transition
            if future.done():
                self.base_transition = None
                try:
                    success = future.result().success
                except Exception:
                    success = False
                if success:
                    self.state_epochs['sim_base'] += 1
                    self.states['sim_base'] = (destination, wall)
                else:
                    self.stop_error = 'base lifecycle transition failed'
                    self.core.fault(self.stop_error)
                    self.finish_operation(False, self.stop_error)
            elif wall - started >= self.p['service_timeout']:
                self.base_client.remove_pending_request(future)
                self.base_transition = None
                self.stop_error = 'base lifecycle transition timed out'
                self.core.fault(self.stop_error)
                self.finish_operation(False, self.stop_error)
        base_state = self.states['sim_base'][0]
        enabling = ((self.operation is not None and self.operation['kind'] == 'enable') or
                    (self.operator is not None and self.operator.activating))
        if base_state == State.PRIMARY_STATE_UNCONFIGURED:
            self.request_base_transition(Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE, wall)
        elif not self.core.armed and not enabling and base_state == State.PRIMARY_STATE_ACTIVE:
            self.request_base_transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE, wall)

    def manage_cancel(self, wall):
        if self.cancel_call is not None:
            future, started = self.cancel_call
            if future.done():
                self.cancel_call = None
                try:
                    result = future.result()
                    rejected = result.return_code not in (CancelGoal.Response.ERROR_NONE,
                                                         CancelGoal.Response.ERROR_GOAL_TERMINATED)
                except Exception:
                    rejected = True
                if rejected and self.core.active_goals:
                    self.cancel_attempted = None
                    self.cancel_retry_at = wall + .5
                    self.stop_error = 'navigation action rejected cancellation'
                    self.core.fault(self.stop_error)
                    self.finish_operation(False, self.stop_error)
            elif wall - started >= self.p['service_timeout']:
                self.cancel_client.remove_pending_request(future)
                self.cancel_call = None
                self.cancel_retry_at = wall + .5
                self.cancel_attempted = None
                self.stop_error = 'navigation cancellation response timed out'
                self.core.fault(self.stop_error)
                self.finish_operation(False, self.stop_error)
        if not self.core.cancel_required:
            self.cancel_attempted = None
            return
        key = tuple(sorted(self.core.active_goals))
        if (self.cancel_call is None and self.cancel_attempted != key and
                wall >= self.cancel_retry_at and self.cancel_client.service_is_ready()):
            self.cancel_attempted = key
            self.cancel_call = (self.cancel_client.call_async(CancelGoal.Request()), wall)

    def inspect_graph(self, wall):
        if wall - self.last_graph_check < 1.0:
            return
        self.last_graph_check = wall
        expected = {'/cmd_vel': 'navigation_guard'}
        missing = []
        for topic in ('/cmd_vel', '/scan', '/camera/depth/image_rect_raw',
                      '/imu/data', '/odom', '/wheel/odometry', '/perception/health',
                      '/perception/memory', '/perception/memory_status'):
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) != 1:
                missing.append(f'{topic} requires one publisher, found {len(publishers)}')
            elif topic in expected and publishers[0].node_name != expected[topic]:
                missing.append(f'{topic} publisher is not {expected[topic]}')
        backend = 'slam_toolbox' if self.p['localization_mode'] == 'slam' else 'amcl'
        expected_tf = sorted([('ekf_filter_node', '/'), ('robot_state_publisher', '/'),
                              (backend, '/')])
        publishers = self.get_publishers_info_by_topic('/tf')
        # slam_toolbox has more than one TF endpoint in a single node. Count
        # graph nodes for uniqueness, not the broadcaster objects they own.
        observed_tf = sorted({(item.node_name, item.node_namespace) for item in publishers})
        graph_nodes = [tuple(item) for item in self.get_node_names_and_namespaces()]
        duplicate_nodes = [node for node in expected_tf if graph_nodes.count(node) != 1]
        self.tf_publishers = [
            {'node': item.node_name, 'namespace': item.node_namespace,
             'endpoint_gid': bytes(item.endpoint_gid).hex()} for item in publishers]
        static_tf = self.get_publishers_info_by_topic('/tf_static')
        if [(item.node_name, item.node_namespace) for item in static_tf] != [('robot_state_publisher', '/')]:
            missing.append('/tf_static requires only the configured robot_state_publisher')
        auto_missing = list(missing)
        if observed_tf != expected_tf or duplicate_nodes:
            auto_missing.append(f'/tf requires unique configured publisher nodes {expected_tf}; '
                                f'found {observed_tf}, missing/duplicate nodes {duplicate_nodes}')
        map_publishers = self.get_publishers_info_by_topic('/map')
        map_authority = 'slam_toolbox' if self.p['localization_mode'] == 'slam' else 'map_server'
        if len(map_publishers) != 1 or map_publishers[0].node_name != map_authority:
            auto_missing.append('/map requires the configured single map publisher')
        manual_tf = [('ekf_filter_node', '/'), ('robot_state_publisher', '/')]
        if (not set(manual_tf).issubset(observed_tf) or not set(observed_tf).issubset(expected_tf) or
                any(graph_nodes.count(node) != 1 for node in manual_tf)):
            missing.append('/tf requires unique EKF and robot_state_publisher nodes for manual control')
        self.graph_missing_by_mode = {'MANUAL': missing, 'AUTO': auto_missing}
        self.graph_missing = auto_missing

    def readiness(self, ros_ns=None, wall=None, mode=None):
        ros_ns = self.now_ns() if ros_ns is None else ros_ns
        wall = time.monotonic() if wall is None else wall
        mode = mode or (self.core.desired_mode() if self.operator is not None else 'AUTO')
        # Stopping into DISABLED has no separate moving-mode graph contract.
        # Keep observing the manual chain while the stop/transition handshake
        # independently requires measured stop and an inactive base.
        if mode == 'DISABLED':
            mode = 'MANUAL'
        missing = list(self.graph_missing_by_mode[mode])
        if not self.core.clock_fresh(wall):
            missing.append('clock absent, paused or stalled')
        for key, limit in [('scan', self.p['scan_timeout']), ('depth', self.p['depth_timeout']),
                           ('imu', self.p['odom_timeout']), ('odom', self.p['odom_timeout']),
                           ('wheel', self.p['wheel_timeout']), ('health', self.p['health_timeout']),
                           ('memory', self.p['health_timeout']),
                           ('health_observation', self.p['depth_timeout']),
                           ('memory_observation', self.p['depth_timeout'])]:
            if not self.fresh(key, limit, ros_ns, wall):
                missing.append(f'{key} invalid or stale')
        camera = self.observations.get('camera')
        if camera is None or not camera.valid or self.depth_geometry != self.camera_geometry:
            missing.append('depth and CameraInfo geometry mismatch')
        keepout_localization = (mode == 'MANUAL' and self.keepouts.polygons and
                                self.p['localization_mode'] == 'amcl')
        if keepout_localization and not self.operator.localization_initialized:
            missing.append('금지 구역 확인을 위해 초기 위치 지정이 필요합니다')
        if (mode == 'AUTO' or keepout_localization) and not self.map_valid:
            missing.append('valid map missing')
        if (mode == 'AUTO' or keepout_localization) and (self.map_tf is None or not 0 <= wall - self.map_tf[1] < self.p['map_tf_timeout']):
            missing.append('map to odom transport stale')
        for name, received in self.tf_received.items():
            if mode == 'MANUAL' and not keepout_localization and name == 'map':
                continue
            if received is None or not 0 <= wall - received < 0.5:
                missing.append(f'{name} transform transport stale')
        # A scan may precede its corresponding EKF TF on DDS. Keep the last
        # successfully transformed observation fresh while the new one waits.
        for stamp, frame, received in reversed(self.scan_pending):
            if not (0 <= (ros_ns - stamp) * 1e-9 < self.p['scan_timeout'] and
                    0 <= wall - received < self.p['scan_timeout']):
                continue
            try:
                self.tf_buffer.lookup_transform(self.p['odom_frame'], frame, Time(nanoseconds=stamp))
                self.scan_tf = Observation(stamp, received, True)
                break
            except TransformException:
                pass
        while self.scan_pending and (wall - self.scan_pending[0][2] >= self.p['scan_timeout'] or
                                      (self.scan_tf and self.scan_pending[0][0] <= self.scan_tf.stamp)):
            self.scan_pending.popleft()
        if (self.scan_tf is None or not 0 <= (ros_ns - self.scan_tf.stamp) * 1e-9 < self.p['scan_timeout'] or
                not 0 <= wall - self.scan_tf.received < self.p['scan_timeout']):
            missing.append('scan-time odom transform unavailable')
        required_servers = self.servers if mode == 'AUTO' else ['collision_monitor', 'velocity_smoother']
        for name in required_servers:
            state, received = self.states[name]
            if state != State.PRIMARY_STATE_ACTIVE or wall - received >= self.p['service_timeout']:
                missing.append(f'{name} not confirmed active')
        base_state, base_seen = self.states['sim_base']
        allowed_base_states = (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE)
        if ((self.operation is not None and self.operation['kind'] == 'enable') or
                (self.operator is not None and self.operator.activating)):
            allowed_base_states += (State.TRANSITION_STATE_ACTIVATING,)
        if base_state not in allowed_base_states or wall - base_seen >= self.p['service_timeout']:
            missing.append('sim_base not configured')
        if self.core.armed and base_state != State.PRIMARY_STATE_ACTIVE:
            missing.append('sim_base not active')
        return missing

    async def on_enable(self, request, response):
        if self.operator is not None:
            if request.data:
                response.success, response.message = False, 'operator mode requires /operator/control ownership and mode selection'
                return response
            return await self.operator.stop(request, response)
        wall = time.monotonic()
        if not request.data:
            self.finish_operation(False, 'superseded by disable request')
            self.core.disable()
            self.publish_zero()
            self.cancel_attempted = None
            self.stop_error = ''
            future = Future()
            self.operation = {'kind': 'disable', 'deadline': wall + self.p['service_timeout'], 'future': future}
        else:
            if self.operation is not None:
                response.success, response.message = False, 'a lifecycle request is already pending'
                return response
            if self.core.armed:
                response.success, response.message = True, 'already enabled; activation epoch unchanged'
                return response
            self.missing = self.readiness(self.now_ns(), wall)
            self.core.health(not self.missing, '; '.join(self.missing), wall)
            error = self.core.arm_error(wall)
            if error or self.states['sim_base'][0] != State.PRIMARY_STATE_INACTIVE:
                response.success = False
                response.message = error or 'base must be configured and inactive before enable'
                if self.missing:
                    response.message += ': ' + '; '.join(self.missing)
                return response
            self.stop_error = ''
            future = Future()
            self.operation = {'kind': 'enable', 'deadline': wall + self.p['service_timeout'], 'future': future}
            self.request_base_transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, wall)
        response.success, response.message = await future
        return response

    def finish_operation(self, success, message):
        operation, self.operation = self.operation, None
        if operation is not None and not operation['future'].done():
            operation['future'].set_result((success, message))

    def manage_operation(self, ros_ns, wall):
        if self.operation is None:
            return
        if wall >= self.operation['deadline']:
            self.core.fault('enable/disable lifecycle operation timed out')
            self.publish_zero()
            self.finish_operation(False, self.core.reason)
            return
        state = self.states['sim_base'][0]
        if self.operation['kind'] == 'disable':
            if not self.core.active_goals and state in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE):
                self.finish_operation(True, 'output stopped, goals terminal and base inactive')
        elif self.missing or self.core.restart_required:
            self.core.fault('readiness lost while activating base')
            self.publish_zero()
            self.finish_operation(False, '; '.join(self.missing) or self.core.reason)
        elif state == State.PRIMARY_STATE_ACTIVE and self.base_transition is None:
            success, message = self.core.arm(ros_ns, wall)
            self.finish_operation(success, message)
        elif state == State.PRIMARY_STATE_INACTIVE:
            self.request_base_transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, wall)

    def report(self, wall):
        signature = (self.core.state, self.core.reason, tuple(self.missing), self.core.goal_id)
        if wall - self.last_report < 0.10 and signature == self.last_state_signature:
            return
        self.last_state_signature, self.last_report = signature, wall
        snapshot = {
            'state': self.core.state, 'reason': self.core.reason,
            'ready': not self.missing and (self.core.armed or not self.core.arm_error(wall)), 'healthy': self.core.healthy,
            'epoch_ns': self.core.epoch_ns, 'goal_id': self.core.goal_id or '',
            'missing': self.missing, 'base_state': self.states['sim_base'][0],
            'restart_required': self.core.restart_required,
            'fault_count': self.core.fault_count,
            'accepted_sources': self.core.accepted_sources, 'rejected_sources': self.core.rejected_sources,
            'accepted_finals': self.core.accepted_finals, 'rejected_finals': self.core.rejected_finals,
            'tf_authority_check': 'configured_launch_graph',
            'tf_publishers': self.tf_publishers,
        }
        self.state_pub.publish(String(data=json.dumps(snapshot, separators=(',', ':'))))
        status = DiagnosticStatus(name='navigation_guard', hardware_id='simulation',
                                  level=DiagnosticStatus.ERROR if self.core.state == 'FAULT_LATCHED' else
                                  DiagnosticStatus.OK if not self.missing else DiagnosticStatus.WARN,
                                  message=self.core.reason,
                                  values=[KeyValue(key='state', value=self.core.state),
                                          KeyValue(key='missing', value='; '.join(self.missing))])
        diagnostic = DiagnosticArray(status=[status])
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_pub.publish(diagnostic)

    def tick(self):
        wall, ros_ns = time.monotonic(), self.now_ns()
        for key, observation in list(self.future_observations.items()):
            if observation.stamp <= ros_ns:
                self.remember(key, observation.stamp, observation.valid, observation.received)
        self.poll_states(wall)
        self.manage_base(wall)
        self.inspect_graph(wall)
        self.missing = self.readiness(ros_ns, wall)
        self.core.health(not self.missing, '; '.join(self.missing), wall)
        self.core.tick(ros_ns, wall)
        self.manage_cancel(wall)
        self.manage_operation(ros_ns, wall)
        if self.operator is not None:
            self.operator.tick(ros_ns, wall)
        if (not self.core.motion_allowed() or
                (self.core.output == (0.0, 0.0) and not self.core.waiting_for_final(ros_ns, wall))):
            self.publish_zero()
        if (self.missing and wall - self.started > self.p['readiness_timeout'] and
                not self.startup_timeout_reported):
            self.startup_timeout_reported = True
            self.get_logger().error('Readiness deadline exceeded: ' + '; '.join(self.missing))
        self.report(wall)


def main(args=None):
    # rclpy's asynchronous signal handler can invalidate the context while a
    # callback is taking a message. Finish that callback before tearing it down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    node = None
    try:
        node = NavigationGuard()
        executor.add_node(node)
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if node is not None and rclpy.ok():
                node.core.disable()
                node.publish_zero()
        finally:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
