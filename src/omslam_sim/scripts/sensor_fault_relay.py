#!/usr/bin/env python3
"""Opt-in simulation fault relay. A blocked stream is dropped, never restamped."""
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import SetBool


class Relay(Node):
    def __init__(self):
        super().__init__('sensor_fault_relay')
        self.blocked = {'scan': False, 'depth': False}
        for name, message, source, target in (
                ('scan', LaserScan, '/sim/sensors/scan', '/scan'),
                ('depth', Image, '/sim/sensors/depth', '/camera/depth/image_rect_raw')):
            qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                   if name == 'depth' else qos_profile_sensor_data)
            publisher = self.create_publisher(message, target, qos)
            self.create_subscription(message, source,
                partial(self.forward, name, publisher),
                qos)
            self.create_service(SetBool, '/simulation/fault/'+name,
                partial(self.set_block, name))

    def forward(self, key, publisher, msg):
        if not self.blocked[key]:
            publisher.publish(msg)

    def set_block(self, key, request, response):
        self.blocked[key] = request.data
        response.success = True
        response.message = key+(' blocked' if request.data else ' forwarding original samples')
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init(); node = Relay()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
