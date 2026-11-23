"""Opt-in read-only HFI-A9 gyro source; no configuration commands or reconnect."""
import math
import time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omslam_base.runner import spin_node
from omslam_base.vendor import load_control_module


class HfiA9(Node):
    def __init__(self, **kwargs):
        super().__init__('hfi_a9', **kwargs)
        self.port = None
        for key, value in {'port': '', 'baud': 921600, 'frame_id': 'imu_link',
                           'mounting_verified': False, 'gyro_variance': [1e-4, 1e-4, 1e-4]}.items():
            self.declare_parameter(key, value)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Serial HFI-A9 cannot use simulated time')
        port = self.get_parameter('port').value
        if not port or not self.get_parameter('mounting_verified').value:
            raise ValueError('HFI-A9 requires explicit port and mounting_verified:=true after physical axis/TF verification')
        self.frame = self.get_parameter('frame_id').value
        if not self.frame or self.frame.startswith('/'):
            raise ValueError('Invalid IMU frame')
        self.variance = list(self.get_parameter('gyro_variance').value)
        if len(self.variance) != 3 or not all(math.isfinite(v) and v > 0 for v in self.variance):
            raise ValueError('gyro_variance requires three positive finite values')
        self.decoder = load_control_module('hfi_a9').A9Decoder()
        import serial
        self.port = serial.Serial(port, int(self.get_parameter('baud').value), timeout=0, exclusive=True)
        try:
            self.port.reset_input_buffer()
            self.publisher = self.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)
            self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self.previous_tick = None
            self.last_received = None
            self.failed = False
            self.reason = 'Waiting for motion frames'
            # Physical polling and health use steady time; stamps use the node clock.
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.005, self.receive, clock=steady)
            self.create_timer(1., self.report, clock=steady)
            self.get_logger().warn('Gyro source uses host receive time; device tick units/latency are not calibrated. Mounting must match imu_link TF.')
        except BaseException:
            self.close_port()
            raise

    def close_port(self):
        # Receive failure and node teardown may both release the device.
        port, self.port = self.port, None
        if port is not None:
            port.close()

    def receive(self):
        if self.failed: return
        try:
            pending = self.port.in_waiting
            if not pending: return
            frames = self.decoder.feed(self.port.read(pending))
            latest = None
            for frame in frames:
                if frame['kind'] != 'motion': continue
                tick = frame['stamp']
                if self.previous_tick is not None:
                    delta = (tick - self.previous_tick) & 0xFFFFFFFF
                    if delta == 0: continue
                    if delta >= 0x80000000:
                        raise RuntimeError('HFI device tick regressed/rebooted; restart after checking sensor')
                self.previous_tick = tick
                if not all(math.isfinite(v) for v in frame['values'][:3]):
                    self.reason = 'Nonfinite gyro frame dropped'
                    continue
                latest = frame
            # Keep only latest sample from a batch: never re-stamp/replay a cached sample.
            if latest is None: return
            msg = Imu(); msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = latest['values'][:3]
            x, y, z = self.variance
            msg.angular_velocity_covariance = [x, 0., 0., 0., y, 0., 0., 0., z]
            # No orientation/acceleration is certified by this gyro-only adapter.
            msg.orientation_covariance[0] = -1.
            msg.linear_acceleration_covariance[0] = -1.
            self.publisher.publish(msg)
            self.last_received = time.monotonic()
            self.reason = 'Fresh gyro frame, host receive timestamp'
        except Exception as exc:
            self.failed = True
            self.reason = str(exc)
            self.close_port()
            self.get_logger().error(self.reason)

    def report(self):
        fresh = self.last_received is not None and time.monotonic() - self.last_received < .2
        status = DiagnosticStatus(name='hfi_a9', hardware_id=self.frame,
                                  level=DiagnosticStatus.ERROR if self.failed else (DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN),
                                  message=self.reason if fresh or self.failed else 'No fresh motion frames')
        status.values = [KeyValue(key='timestamp_source', value='host_receive'),
                         KeyValue(key='crc_errors', value=str(self.decoder.crc_errors))]
        msg = DiagnosticArray(status=[status]); msg.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(msg)

    def destroy_node(self):
        self.close_port()
        return super().destroy_node()


def main(args=None):
    spin_node(HfiA9, args=args)
