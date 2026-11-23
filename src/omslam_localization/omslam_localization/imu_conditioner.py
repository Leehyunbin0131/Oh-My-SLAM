"""Validate fresh IMU measurements and subtract sensor-frame gyro bias."""
import time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omslam_base.runner import spin_node
from .conditioning import Conditioner, load_bias


class ImuConditioner(Node):
    def __init__(self, **kwargs):
        super().__init__('imu_conditioner', **kwargs)
        defaults = {'expected_frame': 'imu_link', 'gyro_bias_rad_s': [0., 0., 0.],
                    'fallback_gyro_variance': [1e-4, 1e-4, 1e-4], 'max_age_s': .2,
                    'future_tolerance_s': .02, 'calibration_file': '', 'expected_identity': ''}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        bias = params['gyro_bias_rad_s']
        if params['calibration_file']:
            bias = load_bias(params['calibration_file'], params['expected_identity'])
            self.get_logger().info('Loaded sensor-frame bias only; mounting/dynamic acceptance is separate')
        self.filter = Conditioner(params['expected_frame'], bias, params['fallback_gyro_variance'],
                                  params['max_age_s'], params['future_tolerance_s'])
        self.output = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.subscription = self.create_subscription(Imu, '/imu/raw', self.receive, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last_received = None
        self.reason = 'No accepted IMU measurement'
        self.accepted = self.rejected = 0
        # Report absence while /clock is paused. Acceptance and published
        # stamps stay on node time; receipt age stays monotonic.
        self.create_timer(1., self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        try:
            output = self.filter.accept(msg, self.get_clock().now().nanoseconds)
        except ValueError as exc:
            self.rejected += 1
            self.reason = str(exc)
            return
        self.output.publish(output)
        self.last_received = time.monotonic()
        self.accepted += 1
        self.reason = 'Fresh gyro measurement'

    def report(self):
        fresh = self.last_received is not None and time.monotonic() - self.last_received <= self.filter.max_age_ns / 1e9
        status = DiagnosticStatus(name='imu_conditioner', hardware_id=self.filter.frame,
                                  level=DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN,
                                  message=self.reason if fresh else 'IMU absent or stale; EKF may predict without it')
        status.values = [KeyValue(key='accepted', value=str(self.accepted)),
                         KeyValue(key='rejected', value=str(self.rejected)),
                         KeyValue(key='last_observation', value=self.reason)]
        msg = DiagnosticArray(status=[status]); msg.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(msg)


def main(args=None):
    spin_node(ImuConditioner, args=args)
