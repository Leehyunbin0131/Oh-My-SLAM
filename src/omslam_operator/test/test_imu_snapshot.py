"""Real ROS fixed-size arrays must produce JSON-safe GUI snapshots."""
import json
from types import SimpleNamespace

from sensor_msgs.msg import Imu
from omslam_operator.worker import ConsoleNode


def test_imu_callback_snapshot_json_and_unavailable_orientation():
    snapshots = {}
    rotation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    receiver = SimpleNamespace(
        assign=lambda name, value: snapshots.__setitem__(name, value),
        tf=SimpleNamespace(lookup_transform=lambda *_: SimpleNamespace(
            transform=SimpleNamespace(rotation=rotation))),
    )
    message = Imu()
    message.header.frame_id = 'imu_link'
    message.orientation.w = 1.
    message.orientation_covariance[0] = -1.
    message.linear_acceleration_covariance[0] = -1.

    ConsoleNode.imu(receiver, message)
    payload = json.loads(json.dumps(snapshots))
    assert payload['imu']['accel_valid'] is False
    assert payload['imu']['valid'] is False
    assert payload['imu']['rpy'] == []
    assert payload['imu_base']['valid'] is False
    assert payload['imu_base']['rpy'] == []
    assert payload['imu_base']['accel'] == []

    message.orientation_covariance[0] = 0.
    message.linear_acceleration_covariance[0] = 0.
    ConsoleNode.imu(receiver, message)
    payload = json.loads(json.dumps(snapshots))
    assert payload['imu']['accel_valid'] is True
    assert payload['imu']['valid'] is True
    assert payload['imu_base']['valid'] is True
    assert payload['imu_base']['rpy'] == [0., 0., 0.]
