"""Explicit synthetic gyro source for software checks, never wheel-derived."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class MockImu(Node):
    def __init__(self):
        super().__init__('mock_imu')
        self.declare_parameter('yaw_rate', 0.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('variance', 0.0001)
        self.publisher = self.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)
        self.create_timer(.02, self.sample)
        self.get_logger().warn('SYNTHETIC IMU: independent constant gyro fixture, not a physical sensor')

    def sample(self):
        msg = Imu(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.angular_velocity.z = float(self.get_parameter('yaw_rate').value)
        v = float(self.get_parameter('variance').value)
        msg.angular_velocity_covariance = [v, 0., 0., 0., v, 0., 0., 0., v]
        msg.orientation_covariance[0] = -1.
        msg.linear_acceleration_covariance[0] = -1.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = MockImu()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
