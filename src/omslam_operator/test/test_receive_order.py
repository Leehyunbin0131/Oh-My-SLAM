"""Separate /clock delivery must not revoke fresh UI input or repaint old frames."""
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from PySide6.QtCore import QCoreApplication
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omslam_operator_msgs.msg import OperatorState
from omslam_operator import worker
from omslam_operator.live_model import LiveConsoleModel
from omslam_operator.worker import ConsoleNode, RosWorker, Shared


@pytest.fixture
def receiver(monkeypatch):
    app = QCoreApplication.instance() or QCoreApplication([])
    wall = [1000.]
    monkeypatch.setattr('time.monotonic', lambda: wall[0])
    rclpy.init(domain_id=193)
    shared = Shared()
    node = ConsoleNode(shared, True, '')
    shared.node = node
    node.get_clock().set_ros_time_override(Time(seconds=10.))
    model = LiveConsoleModel(shared)
    model.timer.stop()
    model.display_timer.stop()
    callbacks = {s.topic_name: s.callback for s in node.subscriptions}
    yield SimpleNamespace(app=app, node=node, model=model, shared=shared,
                          wall=wall, receive=callbacks)
    model.timer.stop()
    model.display_timer.stop()
    model.deleteLater()
    shared.node = None
    node.destroy_node()
    rclpy.shutdown()


def state(f, stamp):
    msg = OperatorState()
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.session_id = 'session'
    msg.owner_id = f.model.owner
    msg.epoch = 1
    msg.mode = 'MANUAL'
    msg.phase = 'ACTIVE'
    return msg


def test_future_state_preserves_owner_then_promotes_original_receipt(receiver):
    f = receiver
    f.model.token, f.model.epoch = 'owned-token', 1
    f.receive['/operator/state'](state(f, 10.))
    f.model.tick()
    assert f.model.token
    f.wall[0] += .02
    f.receive['/operator/state'](state(f, 10.04))
    f.model.tick()
    assert f.shared.data['topics']['/operator/state']['stamp'] == 10.
    assert f.model.token and f.shared.latest_heartbeat is not None
    f.node.get_clock().set_ros_time_override(Time(seconds=10.04))
    f.wall[0] += .01
    # Continuous streams must promote the caught-up sample before replacing
    # the pending slot with another sample ahead of /clock.
    f.receive['/operator/state'](state(f, 10.06))
    assert f.shared.data['topics']['/operator/state']['stamp'] == pytest.approx(10.04)
    assert f.shared.data['topics']['/operator/state']['received'] == pytest.approx(1000.02)
    f.model.tick()
    assert f.model.token
    # A genuinely stalled ROS worker still releases input at the existing .3 s limit.
    f.wall[0] += .31
    f.model.tick()
    assert not f.model.token and f.shared.latest_heartbeat is None


def test_pending_future_state_expires_without_resurrection(receiver):
    f = receiver
    f.receive['/operator/state'](state(f, 10.2))
    assert '/operator/state' not in f.shared.data['topics']
    f.wall[0] += .31
    f.node.get_clock().set_ros_time_override(Time(seconds=10.2))
    f.node.flush_pending()
    assert '/operator/state' not in f.shared.data['topics']
    assert not f.node.pending_samples
    f.receive['/operator/state'](state(f, 10.4))
    f.node.get_clock().set_ros_time_override(Time(seconds=1.))
    f.node.flush_pending()
    assert not f.node.pending_samples  # No samples from the previous clock session.


def test_camera_uses_current_clock_and_keeps_last_coherent_frame(receiver):
    f = receiver
    image = Image(height=1, width=1, encoding='rgb8', step=3, data=[1, 2, 3])
    image.header.stamp = Time(seconds=10.).to_msg()
    f.receive['/camera/color/image_raw'](image)
    f.shared.data['ros_time'] = 9.98  # Last presentation pump is older than /clock.
    f.model.refresh()
    assert f.model.ui['rgbFresh']
    future = Image(height=1, width=1, encoding='rgb8', step=3, data=[4, 5, 6])
    future.header.stamp = Time(seconds=10.02).to_msg()
    f.wall[0] += .01
    f.receive['/camera/color/image_raw'](future)
    f.model.refresh()
    assert f.model.ui['rgbFresh'] and f.model.ui['rgbRevision'] == 1
    f.node.get_clock().set_ros_time_override(Time(seconds=10.02))
    f.wall[0] += .01
    f.node.flush_pending()
    f.model.refresh()
    assert f.model.ui['rgbFresh'] and f.model.ui['rgbRevision'] == 2
    assert f.shared.data['cameras']['rgb']['received'] == pytest.approx(1000.01)
    f.wall[0] += .81
    f.model.refresh()
    assert not f.model.ui['rgbFresh']


def test_wall_clock_host_diagnostics_are_not_deferred_by_sim_time(receiver):
    f = receiver
    message = DiagnosticArray()
    message.header.stamp = Time(seconds=1794800000.).to_msg()
    message.status = [DiagnosticStatus(name='system_monitor/pc/cpu', values=[
        KeyValue(key='host_role', value='operator_pc'),
        KeyValue(key='valid', value='true'), KeyValue(key='busy_percent', value='23')])]
    f.receive['/diagnostics'](message)
    f.model.refresh()
    assert f.model.ui['hostCpu'] == '23%'
    assert '/diagnostics' not in f.node.pending_samples


def test_history_tick_uses_one_transform_for_pose_and_freshness(receiver, monkeypatch):
    f = receiver
    looked_up = []
    transforms = []
    # At 10 s, the first pose is fresh and the second is at the stale boundary.
    for seconds, x in ((9.75, 1.25), (9.5, 99.), (9.99, 123.)):
        sample = TransformStamped()
        sample.header.stamp = Time(seconds=seconds).to_msg()
        sample.transform.translation.x = x
        sample.transform.translation.y = -2.
        sample.transform.rotation.w = 1.
        transforms.append(sample)

    def lookup(target, source, stamp):
        looked_up.append((target, source, stamp.nanoseconds))
        return transforms[len(looked_up) - 1]

    monkeypatch.setattr(f.node.tf, 'lookup_transform', lookup)
    f.node.pump()
    assert looked_up == [('map', 'base_footprint', 0)]
    assert f.shared.data['pose'] == [1.25, -2., 0.]
    assert f.shared.data['pose_received'] == f.wall[0]
    f.wall[0] += .11
    f.node.pump()
    assert looked_up == [('map', 'base_footprint', 0)] * 2
    assert f.shared.data['pose'] is None
    assert f.shared.data['pose_received'] == 0.


def test_worker_external_shutdown_has_no_error_and_destroys_action_client(receiver, monkeypatch):
    f = receiver
    calls = []

    def interrupted(timeout_sec):
        assert timeout_sec == .02
        raise ExternalShutdownException()

    destroy = f.node.navigate.destroy

    def destroy_client():
        calls.append('client')
        destroy()

    def destroy_base(node):
        assert node.navigate not in node.waitables
        calls.append('node')

    with monkeypatch.context() as patch:
        patch.setattr(worker, 'ConsoleNode', lambda *_: f.node)
        patch.setattr(worker, 'SingleThreadedExecutor', lambda **_: SimpleNamespace(
            add_node=lambda node: None, spin_once=interrupted,
            shutdown=lambda: calls.append('executor')))
        patch.setattr(f.node.navigate, 'destroy', destroy_client)
        patch.setattr(Node, 'destroy_node', destroy_base)
        RosWorker(f.shared).run()
    assert calls == ['executor', 'client', 'node']
    assert f.shared.node is None
    assert f.shared.data['events'] == []
