"""A scan arriving before its exact TF must not blink the entire overlay."""
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from omslam_operator.worker import ConsoleNode, Shared


@pytest.fixture
def receiver(monkeypatch):
    wall = [1000.]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    rclpy.init(domain_id=196)
    shared = Shared()
    node = ConsoleNode(shared, True, '')
    node.last_graph = node.last_history = wall[0]
    node.get_clock().set_ros_time_override(Time(seconds=10.))
    receive = next(s.callback for s in node.subscriptions if s.topic_name == '/scan')
    f = SimpleNamespace(node=node, shared=shared, wall=wall, receive=receive)
    transform(f, 'base_footprint', 'laser_frame', 0., static=True)
    chain(f, 10.)
    yield f
    node.destroy_node()
    rclpy.try_shutdown()


def transform(f, parent, child, stamp, x=0., static=False):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.
    setter = f.node.tf.set_transform_static if static else f.node.tf.set_transform
    setter(msg, 'scan_display_regression')


def chain(f, stamp, x=0.):
    transform(f, 'map', 'odom', stamp, x)
    transform(f, 'odom', 'base_footprint', stamp)


def scan(stamp, distance=1., frame='laser_frame'):
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.range_min, msg.range_max = .1, 12.
    msg.angle_increment = .1
    msg.ranges = [distance] * 4
    return msg


def advance(f, ros, elapsed=.01):
    f.wall[0] += elapsed
    f.node.get_clock().set_ros_time_override(Time(seconds=ros))


def first_x(f):
    return f.shared.data['scan'][0][0]


def test_delayed_tf_retains_cloud_then_retries_original_exact_stamp(receiver):
    f = receiver
    f.receive(scan(10.))
    original = list(f.shared.data['scan'])
    advance(f, 10.1, .05)
    f.receive(scan(10.1, 2.))
    received = f.shared.data['topics']['/scan']['received']
    assert f.shared.data['scan'] == original
    assert f.shared.data['scan_info']['display_status'] == 'tf_wait'
    # Later TF brackets the acquisition time. Exact interpolation gives x=3;
    # substituting latest TF would give x=4 and fails this assertion.
    chain(f, 10.2, 2.)
    advance(f, 10.14, .02)
    f.node.pump()
    assert first_x(f) == pytest.approx(3.)
    assert f.shared.data['scan_info']['display_stamp'] == pytest.approx(10.1)
    assert f.shared.data['topics']['/scan']['received'] == received
    assert f.shared.data['topics']['/scan']['count'] == 2


def test_pending_scans_progress_when_tf_lags_more_than_one_scan(receiver):
    f = receiver
    f.receive(scan(10.))
    advance(f, 10.1)
    f.receive(scan(10.1, 2.))
    advance(f, 10.2)
    f.receive(scan(10.2, 3.))
    chain(f, 10.15, 1.5)
    f.node.pump()
    assert first_x(f) == pytest.approx(3.)
    assert f.shared.data['scan_info']['display_stamp'] == pytest.approx(10.1)
    chain(f, 10.3, 3.)
    f.node.pump()
    assert first_x(f) == pytest.approx(5.)
    assert f.shared.data['scan_info']['display_stamp'] == pytest.approx(10.2)


def test_newer_projected_scan_replaces_pending_without_late_rewind(receiver):
    f = receiver
    f.receive(scan(10.))
    advance(f, 10.1)
    f.receive(scan(10.1, 2., frame='delayed_laser'))
    chain(f, 10.2, 2.)
    advance(f, 10.2)
    f.receive(scan(10.2, 3.))
    expected = list(f.shared.data['scan'])
    assert first_x(f) == pytest.approx(5.)
    transform(f, 'base_footprint', 'delayed_laser', 0., static=True)
    f.node.pump()
    assert f.shared.data['scan'] == expected
    # A late-arriving older scan also cannot replace the newer visible scan.
    f.receive(scan(10.15, 9.))
    assert f.shared.data['scan'] == expected
    assert f.shared.data['scan_info']['display_stamp'] == pytest.approx(10.2)


@pytest.mark.parametrize('expiry_clock', ['wall', 'ros'])
def test_actual_stale_cloud_expires_even_with_fresh_untransformable_scans(receiver, expiry_clock):
    f = receiver
    f.receive(scan(10.))
    for ros, wall_elapsed in ((10.1, .1), (10.2, .1), (10.29, .09)):
        advance(f, ros, wall_elapsed if expiry_clock == 'wall' else .01)
        f.receive(scan(ros, 2.))
    assert f.shared.data['scan']
    advance(f, 10.29 if expiry_clock == 'wall' else 10.31, .02)
    f.node.pump()
    assert f.shared.data['scan'] == []
    assert f.shared.data['scan_info']['display_stamp'] is None


def test_clock_and_tf_deferral_do_not_renew_original_receipt_expiry(receiver):
    f = receiver
    f.receive(scan(10.1, 2.))  # Delivered before its matching /clock.
    advance(f, 10.1, .25)
    f.node.pump()  # /clock catches up, but exact TF still lags.
    assert f.shared.data['topics']['/scan']['received'] == pytest.approx(1000.)
    advance(f, 10.1, .06)
    chain(f, 10.2, 2.)
    f.node.pump()
    assert f.shared.data['scan'] == []
    assert f.shared.data['scan_info']['display_stamp'] is None


def test_clock_reset_clears_visible_and_pending_scans_without_resurrection(receiver):
    f = receiver
    f.receive(scan(10.))
    advance(f, 10.1)
    f.receive(scan(10.1, 2.))
    advance(f, 1.)
    f.node.pump()
    assert f.shared.data['scan'] == []
    chain(f, 10.2, 2.)
    advance(f, 10.1)
    f.node.pump()
    assert f.shared.data['scan'] == []
    advance(f, 1.)
    f.node.pump()
    chain(f, 1.1, 4.)
    advance(f, 1.1)
    f.receive(scan(1.1, 2.))
    assert first_x(f) == pytest.approx(6.)


def test_zero_stamp_cannot_project_using_latest_tf(receiver):
    f = receiver
    f.receive(scan(0., 2.))
    f.node.pump()
    assert f.shared.data['scan'] == []
