"""Lifecycle sim command adapter and measured wheel odometry covariance."""
from copy import deepcopy
import math
import signal
import time

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock as ClockMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omslam_base.core import Geometry
from omslam_base.telemetry import JointTelemetry, with_sim_target, write_motor_message
from omslam_operator_msgs.msg import MotorTelemetry
from .control import SIM_MAX_WHEEL_RPM, SimCommandGuard


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


class SimBase(LifecycleNode):
    def __init__(self):
        super().__init__('sim_base')
        for key, value in {'left_radius': .068, 'right_radius': .068, 'wheel_separation': .480,
                           'max_rpm': SIM_MAX_WHEEL_RPM, 'wheel_vx_variance': .0025, 'wheel_wz_variance': .01,
                           'clock_stall_timeout': .3}.items():
            self.declare_parameter(key, value)
        self.configured = False
        self.wheel_vx_variance = self.wheel_wz_variance = None
        self.guard = SimCommandGuard()
        self.output_moving = False
        self.last_odom_stamp = None
        self.last_odom_at = None
        self.latest_clock = None
        self.clock_advanced_at = None
        self.last_diag = 0.
        self.joint_telemetry = JointTelemetry()
        self.last_motor_publish = 0.
        self.output = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.wheel = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.motor = self.create_publisher(MotorTelemetry, '/base/motor_state', 10)
        self.create_subscription(ClockMessage, '/clock', self.clock_sample,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(Odometry, '/sim/wheel/odometry_raw', self.odometry, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel/requested', self.requested, 1)
        self.create_subscription(TwistStamped, '/cmd_vel', self.final, 1)
        # Wall/monotonic timer continues to publish zero while /clock is paused.
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.parameters)

    def parameters(self, params):
        return SetParametersResult(successful=not self.configured,
                                   reason='cleanup before changing sim base parameters' if self.configured else '')

    def val(self, name): return self.get_parameter(name).value

    def on_configure(self, state):
        try:
            if not self.val('use_sim_time'):
                raise ValueError('sim_base requires use_sim_time=true')
            geometry = Geometry(left_radius=self.val('left_radius'), right_radius=self.val('right_radius'),
                                separation=self.val('wheel_separation'), max_rpm=self.val('max_rpm'))
            stall, vx_variance, wz_variance = (self.val(k) for k in
                                               ('clock_stall_timeout', 'wheel_vx_variance', 'wheel_wz_variance'))
            if not all(math.isfinite(v) and v > 0 for v in (stall, vx_variance, wz_variance)):
                raise ValueError('Positive finite watchdog and covariance values required')
            self.guard = SimCommandGuard(geometry, stall)
            self.wheel_vx_variance, self.wheel_wz_variance = float(vx_variance), float(wz_variance)
            if self.latest_clock is not None:
                self.guard.clock(self.latest_clock, self.clock_advanced_at)
            self.last_odom_stamp = self.last_odom_at = None
            self.joint_telemetry = JointTelemetry()
            self.configured = True
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        now = time.monotonic()
        if (self.last_odom_at is None or not 0 <= now-self.last_odom_at < .4 or
                self.last_odom_stamp is None or self.latest_clock is None or
                not -20_000_000 <= self.latest_clock-self.last_odom_stamp < 300_000_000):
            self.get_logger().error('Activation requires a fresh measured wheel odometry sample')
            return TransitionCallbackReturn.FAILURE
        try:
            self.guard.activate(now)
            return TransitionCallbackReturn.SUCCESS
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        self.guard.deactivate(); self.publish_velocity(0., 0.)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.on_deactivate(state); self.configured = False
        self.last_odom_stamp = self.last_odom_at = None
        self.joint_telemetry = JointTelemetry()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state): return self.on_cleanup(state)
    def on_error(self, state): return self.on_cleanup(state)

    def clock_sample(self, msg):
        ns = stamp_ns(msg.clock); now = time.monotonic()
        rewound = self.latest_clock is not None and ns < self.latest_clock
        was_active = self.guard.active
        if self.latest_clock is None or ns != self.latest_clock:
            if rewound:
                self.last_odom_stamp = self.last_odom_at = None
                self.joint_telemetry = JointTelemetry()
            self.latest_clock = ns; self.clock_advanced_at = now
        self.guard.clock(ns, now)
        if rewound and was_active:
            self.trigger_deactivate()
            self.guard.reason = 'clock rewound; fresh wheels and reactivation required'
        elif self.output_moving:
            self.check_expiry(now)

    def joints(self, msg):
        if not self.configured or self.latest_clock is None:
            return
        ns = stamp_ns(msg.header.stamp)
        if abs(self.latest_clock-ns) > 300_000_000:
            return
        self.joint_telemetry.accept(list(msg.name), msg.position, msg.velocity, ns, time.monotonic())

    def odometry(self, msg):
        if not self.configured or self.latest_clock is None: return
        ns = stamp_ns(msg.header.stamp)
        if ns <= 0 or (self.last_odom_stamp is not None and ns <= self.last_odom_stamp): return
        if abs(self.latest_clock-ns) > 300_000_000: return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            self.get_logger().error('Gazebo odometry frame must be odom -> base_footprint', throttle_duration_sec=2.)
            return
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x,
                  msg.twist.twist.angular.z, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        if not all(math.isfinite(v) for v in values): return
        output = deepcopy(msg)
        # Explicit initial uncertainty, not ground-truth covariance. EKF fuses
        # measured wheel twist only; never consume /sim/ground_truth here.
        output.pose.covariance = [0.]*36
        output.twist.covariance = [0.]*36
        for i in (0, 7, 14, 21, 28, 35): output.pose.covariance[i] = 1e3
        for i in (7, 14, 21, 28): output.twist.covariance[i] = 1e3
        output.twist.covariance[0] = self.wheel_vx_variance
        output.twist.covariance[35] = self.wheel_wz_variance
        self.last_odom_stamp = ns; self.last_odom_at = time.monotonic()
        self.wheel.publish(output)

    def valid(self, msg):
        t = msg.twist
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        return msg.header.frame_id == 'base_footprint' and all(math.isfinite(v) for v in values) and all(v == 0 for v in (t.linear.y, t.linear.z, t.angular.x, t.angular.y))

    def publish_velocity(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self.output.publish(msg)
        self.output_moving = bool(vx or wz)

    def check_expiry(self, now):
        # Release the last moving output as soon as either clock reports expiry.
        # Only the steady 50 Hz control timer publishes nonzero commands.
        if not any(self.guard.sample(now)) and self.output_moving:
            self.publish_velocity(0., 0.)

    def requested(self, msg):
        now = time.monotonic(); self.check_expiry(now)
        if not self.configured or not self.guard.active or self.guard.suspended: return
        if not self.valid(msg): self.guard.gate.target = None; return
        self.guard.requested(stamp_ns(msg.header.stamp), now)

    def final(self, msg):
        now = time.monotonic(); self.check_expiry(now)
        if not self.configured or not self.guard.active or self.guard.suspended: return
        if not self.valid(msg): self.guard.gate.target = None; return
        self.guard.final(stamp_ns(msg.header.stamp), msg.twist.linear.x, msg.twist.angular.z, now)
        if not msg.twist.linear.x and not msg.twist.angular.z:
            self.publish_velocity(0., 0.)

    def tick(self):
        now = time.monotonic()
        if (self.configured and self.guard.active and (self.last_odom_stamp is None
                or self.latest_clock-self.last_odom_stamp > 400_000_000)):
            self.trigger_deactivate()
            self.guard.reason = 'wheel feedback stale; reactivation required'
        vx, wz = self.guard.sample(now) if self.configured else (0., 0.)
        self.publish_velocity(vx, wz)
        if self.joint_telemetry.snapshot is not None and now-self.last_motor_publish >= .05:
            published_ns = self.get_clock().now().nanoseconds
            snapshot = with_sim_target(self.joint_telemetry.snapshot,
                                       self.guard.geometry.rpm(vx, wz), published_ns)
            self.motor.publish(write_motor_message(MotorTelemetry(), snapshot, now, published_ns, KeyValue))
            self.last_motor_publish = now
        if now-self.last_diag >= 1.:
            status = DiagnosticStatus(name='sim_base', hardware_id='Gazebo wheel feedback',
                                      level=DiagnosticStatus.OK if self.configured and self.guard.clock_fresh(now) else DiagnosticStatus.WARN,
                                      message=self.guard.reason)
            status.values = [KeyValue(key='active', value=str(self.guard.active)),
                             KeyValue(key='configured', value=str(self.configured)),
                             KeyValue(key='clock_suspended', value=str(self.guard.suspended)),
                             KeyValue(key='odom_source', value='/sim/wheel/odometry_raw')]
            diag = DiagnosticArray(status=[status]); diag.header.stamp = self.get_clock().now().to_msg()
            self.diag.publish(diag); self.last_diag = now


def main(args=None):
    # Keep the context alive until the current callback and final stop finish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    node = None
    try:
        node = SimBase()
        executor.add_node(node)
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if node is not None and rclpy.ok():
                node.guard.deactivate()
                node.output.publish(Twist())
        finally:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
