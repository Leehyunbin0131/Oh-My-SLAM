"""One Hz host diagnostics, separately identified from simulated robot sensors."""
import math
import socket

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .sampling import HostSampler


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.declare_parameter('host_role', 'host')
        self.declare_parameter('host_id', socket.gethostname())
        self.declare_parameter('disk_path', '/')
        self.declare_parameter('sample_period', 1.)
        self.host_id = self.get_parameter('host_id').value
        self.role = self.get_parameter('host_role').value
        period = self.get_parameter('sample_period').value
        if not self.host_id or '/' in self.host_id:
            raise ValueError('host_id must be a nonempty hostname without slashes')
        if self.role not in ('host', 'robot', 'simulation_host', 'operator_pc'):
            raise ValueError('host_role must identify host, robot, simulation_host or operator_pc')
        if not math.isfinite(period) or not .5 <= period <= 60.:
            raise ValueError('sample_period must be between 0.5 and 60 seconds')
        self.sampler = HostSampler(disk_path=self.get_parameter('disk_path').value)
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        # Resource utilization continues to be observed while /clock is paused.
        self.timer = self.create_timer(period, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self):
        observed_ns = self.wall_clock.now().nanoseconds
        message = DiagnosticArray()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(observed_ns, 1_000_000_000)
        for sample in self.sampler.sample():
            values = {'host_id': self.host_id, 'host_role': self.role, 'clock': 'system',
                      'observed_wall_ns': observed_ns, 'valid': str(sample.valid).lower(),
                      'source': sample.source, **sample.values}
            message.status.append(DiagnosticStatus(
                name='system_monitor/' + self.host_id + '/' + sample.category,
                hardware_id=self.host_id,
                level=DiagnosticStatus.OK if sample.valid else DiagnosticStatus.WARN,
                message=sample.message,
                values=[KeyValue(key=key, value=str(value)) for key, value in values.items()]))
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
