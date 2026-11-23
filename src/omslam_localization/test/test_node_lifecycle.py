"""Run the IMU nodes on real rclpy contexts; no serial device is opened."""
from types import SimpleNamespace
import sys
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu


@pytest.fixture
def context(monkeypatch):
    # Keep this graph private to the test process.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    context.try_shutdown()


def spin_until(executor, condition, timeout, action=lambda: None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        action()
        if condition():
            return True
        executor.spin_once(timeout_sec=.02)
    return condition()


def test_conditioner_reports_stale_imu_on_steady_timer_while_ros_time_is_frozen(context):
    from omslam_localization.imu_conditioner import ImuConditioner

    executor = SingleThreadedExecutor(context=context)
    node = ImuConditioner(context=context, parameter_overrides=[Parameter('use_sim_time', value=True)])
    probe = rclpy.create_node('imu_conditioner_probe', context=context)
    try:
        frozen = Time(seconds=100)
        clock = probe.create_publisher(ClockMessage, '/clock', 10)
        raw = probe.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)
        data, reports = [], []
        probe.create_subscription(Imu, '/imu/data', data.append, qos_profile_sensor_data)
        probe.create_subscription(DiagnosticArray, '/diagnostics',
                                  lambda msg: reports.extend(msg.status), 10)
        executor.add_node(node)
        executor.add_node(probe)
        assert spin_until(executor, lambda: node.get_clock().now().nanoseconds == frozen.nanoseconds, 5.,
                          lambda: clock.publish(ClockMessage(clock=frozen.to_msg())))
        assert spin_until(executor, lambda: raw.get_subscription_count() and
                          node.output.get_subscription_count() and
                          node.diagnostics.get_subscription_count(), 5.)
        sample = Imu()
        sample.header.stamp = frozen.to_msg()
        sample.header.frame_id = 'imu_link'
        sample.angular_velocity.z = .2
        raw.publish(sample)
        assert spin_until(executor, lambda: data, 5.)
        # /clock is no longer published: ROS time stays at the frozen stamp.

        def stale():
            return [status for status in reports if status.level == DiagnosticStatus.WARN
                    and {v.key: v.value for v in status.values}['accepted'] == '1']
        assert spin_until(executor, stale, 3.5)
        assert node.get_clock().now().nanoseconds == frozen.nanoseconds
        assert len(data) == 1 and Time.from_msg(data[0].header.stamp).nanoseconds == frozen.nanoseconds
        values = {v.key: v.value for v in stale()[-1].values}
        assert values['accepted'] == '1' and values['rejected'] == '0'
        assert node.accepted == 1 and node.rejected == 0
        assert stale()[-1].message == 'IMU absent or stale; EKF may predict without it'
    finally:
        executor.shutdown()
        probe.destroy_node()
        node.destroy_node()


def hfi_port(monkeypatch, *, reset_error=None):
    from omslam_localization import hfi_a9_node

    ports = []

    class Port:
        def __init__(self, name, baud, **kwargs):
            self.closes = 0
            ports.append(self)

        def reset_input_buffer(self):
            if reset_error:
                raise reset_error

        @property
        def in_waiting(self):
            raise OSError('device disconnected')

        def close(self):
            self.closes += 1

    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(Serial=Port))
    monkeypatch.setattr(hfi_a9_node, 'load_control_module', lambda name: SimpleNamespace(
        A9Decoder=lambda: SimpleNamespace(crc_errors=0, feed=lambda data: [])))
    return hfi_a9_node.HfiA9, ports


HFI_PARAMETERS = [Parameter('port', value='/dev/hfi-a9-test'), Parameter('mounting_verified', value=True)]


def test_hfi_closes_port_when_initialization_after_open_fails(monkeypatch, context):
    HfiA9, ports = hfi_port(monkeypatch, reset_error=OSError('reset failed'))
    with pytest.raises(OSError, match='reset failed'):
        HfiA9(context=context, parameter_overrides=HFI_PARAMETERS)
    assert len(ports) == 1 and ports[0].closes == 1


def test_hfi_receive_failure_and_teardown_close_port_once(monkeypatch, context):
    HfiA9, ports = hfi_port(monkeypatch)
    node = HfiA9(context=context, parameter_overrides=HFI_PARAMETERS)
    try:
        node.receive()
        assert node.failed and node.reason == 'device disconnected'
        assert node.port is None and ports[0].closes == 1
        node.receive()
        node.report()
    finally:
        node.destroy_node()
    node.close_port()
    assert len(ports) == 1 and ports[0].closes == 1
