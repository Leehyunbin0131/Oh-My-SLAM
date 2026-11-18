"""Camera point overlays retain fresh coordinates while exact TF catches up."""
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from omslam_operator.worker import ConsoleNode, Shared


TOPICS = {'memory_points': '/perception/memory_preview',
          'obstacles': '/perception/obstacles', 'points': '/camera/points'}


@pytest.fixture
def receiver(monkeypatch):
    wall = [1000.]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    rclpy.init(domain_id=196)
    shared = Shared()
    node = ConsoleNode(shared, True, '')
    node.last_graph = node.last_history = wall[0]
    node.frame_visible = 'perception'
    node.get_clock().set_ros_time_override(Time(seconds=10.))
    callbacks = {s.topic_name: s.callback for s in node.subscriptions}
    f = SimpleNamespace(node=node, shared=shared, wall=wall, callbacks=callbacks)
    transform(f, 10.)
    yield f
    node.destroy_node()
    rclpy.try_shutdown()


def transform(f, stamp, x=0., child='odom'):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = 'map', child
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.transform.translation.x, msg.transform.rotation.w = x, 1.
    f.node.tf.set_transform(msg, 'cloud_display_regression')


def advance(f, stamp, elapsed=.01):
    f.wall[0] += elapsed
    f.node.get_clock().set_ros_time_override(Time(seconds=stamp))


def receive(f, key, stamp, x=1., frame='odom', samples=None):
    cloud = point_cloud2.create_cloud_xyz32(
        Header(stamp=Time(seconds=stamp).to_msg(), frame_id=frame),
        [(x, 0., 0.), (x, 1., 0.)] if samples is None else samples)
    f.callbacks[TOPICS[key]](cloud)


@pytest.mark.parametrize('key', list(TOPICS))
def test_cloud_retains_then_retries_exact_stamp_without_new_message(receiver, key):
    f = receiver
    receive(f, key, 10.)
    old = list(f.shared.data[key])
    advance(f, 10.1, .05)
    receive(f, key, 10.1, 2.)
    received = f.shared.data['topics'][TOPICS[key]]['received']
    assert f.shared.data[key] == old
    transform(f, 10.2, 2.)
    f.node.pump()
    # Exact interpolation yields x=3; a latest-time substitution yields4.
    assert f.shared.data[key][0][0] == pytest.approx(3.)
    assert f.shared.data['cloud_info'][key]['display_stamp'] == pytest.approx(10.1)
    assert f.shared.data['topics'][TOPICS[key]]['received'] == received
    assert f.shared.data['topics'][TOPICS[key]]['count'] == 2


def test_cloud_backlog_progresses_and_late_older_frame_cannot_rewind(receiver):
    f = receiver
    receive(f, 'memory_points', 10.)
    advance(f, 10.1)
    receive(f, 'memory_points', 10.1, 2.)
    advance(f, 10.2)
    receive(f, 'memory_points', 10.2, 3.)
    transform(f, 10.15, 1.5)
    f.node.pump()
    assert f.shared.data['memory_points'][0][0] == pytest.approx(3.)
    transform(f, 10.3, 3.)
    f.node.pump()
    assert f.shared.data['memory_points'][0][0] == pytest.approx(5.)
    receive(f, 'memory_points', 10.15, 9.)
    assert f.shared.data['memory_points'][0][0] == pytest.approx(5.)


def test_cloud_keys_retry_and_expire_independently(receiver):
    f = receiver
    receive(f, 'memory_points', 10.)
    receive(f, 'obstacles', 10., 4.)
    advance(f, 10.1)
    receive(f, 'memory_points', 10.1, 2., frame='late_frame')
    transform(f, 10.1, 1.)
    receive(f, 'obstacles', 10.1, 5.)
    assert f.shared.data['memory_points'][0][0] == pytest.approx(1.)
    assert f.shared.data['obstacles'][0][0] == pytest.approx(6.)
    transform(f, 10.1, 3., child='late_frame')
    f.node.pump()
    assert f.shared.data['memory_points'][0][0] == pytest.approx(5.)
    assert f.shared.data['obstacles'][0][0] == pytest.approx(6.)


@pytest.mark.parametrize('clock', ['wall', 'ros'])
def test_cloud_stale_expiry_does_not_keep_old_overlay_alive(receiver, clock):
    f = receiver
    receive(f, 'memory_points', 10.)
    for stamp, elapsed in ((10.1, .1), (10.2, .1), (10.29, .09)):
        advance(f, stamp, elapsed if clock == 'wall' else .01)
        receive(f, 'memory_points', stamp, 2.)
    assert f.shared.data['memory_points']
    advance(f, 10.29 if clock == 'wall' else 10.31, .02)
    f.node.pump()
    assert f.shared.data['memory_points'] == []
    assert f.shared.data['cloud_info']['memory_points']['display_stamp'] is None


def test_cloud_clock_deferral_and_retry_keep_original_receipt_deadline(receiver):
    f = receiver
    receive(f, 'memory_points', 10.1, 2.)
    advance(f, 10.1, .25)
    f.node.pump()
    assert f.shared.data['topics'][TOPICS['memory_points']]['received'] == pytest.approx(1000.)
    transform(f, 10.2, 2.)
    advance(f, 10.1, .06)
    f.node.pump()
    assert not f.shared.data.get('memory_points')


@pytest.mark.parametrize('samples', [[], [(float('nan'), float('nan'), float('nan'))]])
def test_fresh_empty_cloud_clears_without_tf_and_discards_older_pending(receiver, samples):
    f = receiver
    receive(f, 'memory_points', 10.)
    advance(f, 10.1)
    receive(f, 'memory_points', 10.1, 2.)
    advance(f, 10.2)
    receive(f, 'memory_points', 10.2, samples=samples)
    assert f.shared.data['memory_points'] == []
    assert f.shared.data['cloud_info']['memory_points']['display_stamp'] == pytest.approx(10.2)
    transform(f, 10.2, 2.)
    f.node.pump()
    assert f.shared.data['memory_points'] == []


def test_cloud_clock_reset_clears_all_keys_and_pending_without_resurrection(receiver):
    f = receiver
    for key in TOPICS:
        receive(f, key, 10.)
    advance(f, 10.1)
    for key in TOPICS:
        receive(f, key, 10.1, 2.)
    advance(f, 1.)
    f.node.pump()
    assert all(not f.shared.data.get(key) for key in TOPICS)
    transform(f, 10.2, 2.)
    advance(f, 10.1)
    f.node.pump()
    assert all(not f.shared.data.get(key) for key in TOPICS)
    advance(f, 1.)
    f.node.pump()
    transform(f, 1.1, 4.)
    advance(f, 1.1)
    receive(f, 'memory_points', 1.1, 2.)
    assert f.shared.data['memory_points'][0][0] == pytest.approx(6.)


def test_zero_cloud_stamp_does_not_request_latest_tf(receiver):
    f = receiver
    receive(f, 'memory_points', 0., 2.)
    f.node.pump()
    assert not f.shared.data.get('memory_points')


@pytest.mark.parametrize('reset_time', [1., 99.8])
def test_clock_rewind_discards_old_tf_and_accepts_new_cloud(receiver, reset_time):
    f = receiver
    advance(f, 100.)
    transform(f, 100., 30.)
    receive(f, 'memory_points', 100.)
    assert f.shared.data['memory_points'][0][0] == pytest.approx(31.)
    advance(f, reset_time)
    f.node.pump()
    assert not f.node.tf.can_transform('map', 'odom', Time(seconds=reset_time))
    transform(f, reset_time, 4.)
    receive(f, 'memory_points', reset_time, 2.)
    assert f.shared.data['memory_points'][0][0] == pytest.approx(6.)
    # Latest must belong to the new epoch as well as exact-time queries.
    latest = f.node.tf.lookup_transform('map', 'odom', Time())
    assert latest.transform.translation.x == pytest.approx(4.)
