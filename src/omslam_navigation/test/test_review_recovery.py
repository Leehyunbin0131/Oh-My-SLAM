"""Exercise node callbacks and both command gates across timing boundaries."""
from collections import OrderedDict, deque
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer

from omslam_base.core import CommandGate
from omslam_navigation.guard_core import NavigationGate
from omslam_navigation.navigation_guard import NavigationGuard, stamp_ns
from omslam_navigation.operator_core import OperatorGate


@pytest.mark.parametrize('enabled', [True, False])
def test_guard_destroys_operator_actions_before_node(monkeypatch, enabled):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p',
                     f'operator_enabled:={str(enabled).lower()}'], domain_id=222)
    calls = []
    try:
        guard = NavigationGuard()
        assert (guard.operator is not None) == enabled
        endpoints = ((guard.operator.action, guard.operator.nav_client) if enabled else ())
        for name, endpoint in zip(('server', 'client'), endpoints):
            destroy = endpoint.destroy

            def destroy_action(name=name, destroy=destroy):
                calls.append(name)
                destroy()

            monkeypatch.setattr(endpoint, 'destroy', destroy_action)
        destroy_node = Node.destroy_node

        def destroy_base(node):
            assert all(endpoint not in node.waitables for endpoint in endpoints)
            calls.append('node')
            return destroy_node(node)

        monkeypatch.setattr(Node, 'destroy_node', destroy_base)
        guard.destroy_node()
        if enabled:
            guard.operator.destroy()
        assert calls == (['server', 'client', 'node'] if enabled else ['node'])
    finally:
        rclpy.try_shutdown()


def command(stamp, velocity):
    message = TwistStamped()
    message.header.frame_id = 'base_footprint'
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    message.twist.linear.x = velocity
    return message


def test_pending_downstream_does_not_retire_first_command(monkeypatch):
    now = [2_120_000_000, 11.12]
    monkeypatch.setattr('omslam_navigation.navigation_guard.time.monotonic', lambda: now[1])
    core = NavigationGate()
    core.clock(1_000_000_000, 10.)
    core.health(True, '', 10.)
    core.clock(now[0], now[1])
    assert core.arm(2_100_000_000, now[1])[0]
    core.goals([('goal', 2_110_000_000, 2)], now[0])
    base = CommandGate()
    base.activate(2_100_000_000)
    output = []

    def final(message):
        output.append(message.twist.linear.x)
        return base.final(stamp_ns(message.header.stamp), message.twist.linear.x,
                          message.twist.angular.z, now[0], now[1])

    guard = SimpleNamespace(core=core, operator=None, future_observations={},
        p={'base_frame': 'base_footprint'}, last_original_stamp=0, follow_started=2_110_000_000,
        now_ns=lambda: now[0], poll_states=lambda _: None,
        manage_base=lambda _: None, inspect_graph=lambda _: None,
        readiness=lambda *_: [], manage_cancel=lambda _: None,
        manage_operation=lambda *_: None, report=lambda _: None,
        final_pub=SimpleNamespace(publish=final),
        requested_pub=SimpleNamespace(publish=lambda message: base.requested(
            stamp_ns(message.header.stamp), now[0], now[1])))
    guard.publish_zero = lambda: NavigationGuard.publish_zero(guard)
    NavigationGuard.on_requested(guard, command(now[0], .1))
    NavigationGuard.tick(guard)  # source has arrived; downstream not yet available
    assert output == []
    NavigationGuard.on_final(guard, command(now[0], .1))
    assert base.current(now[1])[:2] == (.1, 0.)
    NavigationGuard.on_final(guard, command(now[0], 0.))
    assert base.current(now[1])[:2] == (0., 0.)
    NavigationGuard.on_final(guard, command(now[0], .1))
    assert base.current(now[1])[:2] == (0., 0.)  # real stop cannot be replayed away
    now[:] = [2_140_000_000, 11.14]
    core.clock(*now)
    NavigationGuard.on_requested(guard, command(now[0], .1))
    NavigationGuard.tick(guard)
    NavigationGuard.on_final(guard, command(now[0], .1))
    assert base.current(now[1])[:2] == (.1, 0.)
    # A pending response has a bounded lease; it cannot suppress stop forever.
    NavigationGuard.on_final(guard, command(now[0], 0.))
    now[:] = [2_160_000_000, 11.16]
    core.clock(*now)
    NavigationGuard.on_requested(guard, command(now[0], .1))
    now[:] = [2_470_000_000, 11.47]
    core.clock(*now)
    NavigationGuard.tick(guard)
    assert output[-1] == 0.
    assert base.current(now[1]) is None or base.current(now[1])[:2] == (0., 0.)
    assert not core.waiting_for_final(*now)


def test_subsecond_clock_rewind_clears_tf_and_stop_observations():
    core = OperatorGate()
    core.clock(100_000_000_000, 10.)
    core.sample_stop('wheel', 100_000_000_000, 0., 0., True, 10.)
    core.sample_stop('odom', 100_000_000_000, 0., 0., True, 10.)
    buffer = Buffer(cache_time=Duration(seconds=10.))

    def transform(seconds, x):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.header.stamp = Time(seconds=seconds).to_msg()
        tf.transform.translation.x = x
        tf.transform.rotation.w = 1.
        return tf

    buffer.set_transform(transform(100., 5.), 'test')
    guard = SimpleNamespace(core=core, operator=None, tf_buffer=buffer, observations={'old': 1},
        future_observations={'old': 1}, depth_history=OrderedDict(old=1),
        pending_perception={'old': 1}, scan_pending=deque([1]),
        scan_tf=1, map_tf=1, memory_observation=1, health_observation=1,
        tf_received={'map': 1, 'odom': 1}, map_valid=True,
        last_original_stamp=100_000_000_000, publish_zero=lambda: None)
    NavigationGuard.on_clock(guard, Clock(clock=Time(seconds=99.8).to_msg()))
    assert not core.stop_samples and core.stopped_since is None
    buffer.set_transform(transform(99.8, 1.), 'test')
    # Latest would still be the 100s transform if clear() had been omitted.
    latest = buffer.lookup_transform('map', 'odom', Time())
    assert stamp_ns(latest.header.stamp) == 99_800_000_000
    assert latest.transform.translation.x == 1.
    assert not guard.observations and not guard.map_valid


def test_follow_path_cancel_retires_motion_and_active_silence_still_faults(monkeypatch):
    now = [2_120_000_000, 11.12]
    monkeypatch.setattr('omslam_navigation.navigation_guard.time.monotonic', lambda: now[1])
    core = NavigationGate()
    core.clock(1_000_000_000, 10.)
    core.health(True, '', 10.)
    core.clock(*now)
    assert core.arm(2_100_000_000, now[1])[0]
    core.goals([('outer', 2_110_000_000, 2)], *now)
    forwarded, stops = [], []
    guard = SimpleNamespace(core=core, operator=None, follow_started=None, now_ns=lambda: now[0],
                            requested_pub=SimpleNamespace(publish=forwarded.append),
                            publish_zero=lambda: stops.append(now[0]))

    def status(accepted, state):
        item = SimpleNamespace(goal_info=SimpleNamespace(stamp=Time(nanoseconds=accepted).to_msg()), status=state)
        NavigationGuard.on_follow_status(guard, SimpleNamespace(status_list=[item]))

    def drive(stamp):
        NavigationGuard.on_requested(guard, command(stamp, .1))

    status(now[0], 2)
    drive(now[0])
    values = (.1, 0., 0., 0., 0., 0.)
    assert core.final(now[0], 'base_footprint', values, *now)
    status(now[0], 3)
    assert stops and core.output == (0., 0.)
    drive(now[0])
    assert len(forwarded) == 1
    assert not core.final(now[0], 'base_footprint', values, *now)
    now[:] = [2_600_000_000, 11.60]
    core.clock(*now)
    core.tick(*now)
    assert core.state == 'RUNNING' and not core.sources
    status(now[0], 2)
    drive(now[0] - 10_000_000)
    assert len(forwarded) == 1
    drive(now[0])
    assert len(forwarded) == 2 and core.final(now[0], 'base_footprint', values, *now)
    now[:] = [2_910_000_000, 11.91]
    core.clock(*now)
    core.tick(*now)
    assert core.state == 'FAULT_LATCHED' and core.reason == 'original navigation command expired'
