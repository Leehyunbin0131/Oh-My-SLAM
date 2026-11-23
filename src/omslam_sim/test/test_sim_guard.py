import math
import pytest
from omslam_sim.control import SimCommandGuard


def active_guard():
    g=SimCommandGuard();g.clock(1_000_000_000,10.);g.activate(10.)
    g.clock(1_010_000_000,10.01)
    return g


def command(g, stamp=1_010_000_000, now=10.01, vx=.1,wz=.4):
    assert g.requested(stamp,now)
    assert g.final(stamp,vx,wz,now)


def test_original_deadline_and_sim_speedup():
    g=active_guard();command(g)
    assert g.sample(10.02)==pytest.approx((.1,.4))
    g.clock(1_200_000_000,10.2)
    assert g.requested(1_010_000_000,10.2)
    assert g.final(1_010_000_000,.1,.1,10.2)
    assert g.sample(10.32)==(0.,0.)
    g=active_guard();command(g)
    g.clock(1_320_000_000,10.05)
    assert g.sample(10.05)==(0.,0.)


def test_pause_resume_discards_intent_and_rejects_frozen_stamp():
    g=active_guard();command(g)
    assert g.sample(10.4)==(0.,0.) and g.suspended
    g.clock(1_010_000_000,10.45) # duplicate paused clock is not progress
    assert g.suspended
    g.clock(1_020_000_000,10.5)
    assert not g.requested(1_010_000_000,10.5)
    assert g.sample(10.5)==(0.,0.)
    g.clock(1_030_000_000,10.51)
    command(g,1_030_000_000,10.51)
    assert g.sample(10.52)==pytest.approx((.1,.4))


def test_clock_reset_and_deactivation_clear_commands():
    g=active_guard();command(g)
    g.clock(10_000_000,10.02)
    assert g.sample(10.02)==(0.,0.)
    assert not g.requested(1_010_000_000,10.02)
    g.clock(20_000_000,10.03)
    assert not g.active and not g.requested(20_000_000,10.03)
    g.activate(10.03)
    g.clock(30_000_000,10.04);command(g,30_000_000,10.04)
    g.deactivate()
    assert g.sample(10.04)==(0.,0.)
    assert not g.requested(20_000_000,10.04)


def test_coupled_wheel_limit_and_zero_watermark():
    g=active_guard();command(g,vx=-1e308,wz=0.)
    assert g.sample(10.02) == (0., 0.) and g.gate.target is None
    g=active_guard();command(g,vx=3.,wz=2.)
    vx,wz=g.sample(10.02)
    rpms=g.geometry.rpm(vx,wz)
    assert max(abs(r) for r in rpms)==pytest.approx(g.geometry.max_rpm)
    assert wz/vx==pytest.approx(2/3)
    assert g.final(1_010_000_000,0.,0.,10.03)
    assert not g.final(1_010_000_000,.2,.4,10.04)
    assert g.sample(10.04)==(0.,0.)


def test_simulation_default_accepts_70_rpm_straight_before_expiring():
    guard = active_guard()
    maximum = 2 * math.pi * .068 * 70 / 60
    command(guard, vx=maximum, wz=0.)
    assert guard.geometry.max_rpm == 70.
    assert guard.sample(10.02) == pytest.approx((maximum, 0.))
    assert guard.geometry.rpm(*guard.sample(10.02)) == pytest.approx((70., 70.))
    assert guard.sample(10.32) == (0., 0.)


def test_rewind_requires_new_wheel_sample_and_lifecycle_activation():
    import time
    from types import SimpleNamespace
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from omslam_sim.sim_base import SimBase

    output = []
    now = time.monotonic()
    node = SimpleNamespace(guard=SimCommandGuard(), configured=True, output_moving=False, latest_clock=8_000_000_000,
                           last_odom_stamp=8_000_000_000, last_odom_at=now, last_diag=now,
                           get_logger=lambda: SimpleNamespace(error=lambda *args: None),
                           output=SimpleNamespace(publish=lambda msg: output.append(msg.linear.x)),
                           wheel=SimpleNamespace(publish=lambda msg: None),
                           val=lambda key: False if key == 'autostart' else .01,
                           wheel_vx_variance=.0025, wheel_wz_variance=.01)
    node.trigger_deactivate = lambda: SimBase.on_deactivate(node, None)
    node.publish_velocity = lambda vx, wz: SimBase.publish_velocity(node, vx, wz)
    node.check_expiry = lambda now: SimBase.check_expiry(node, now)
    node.guard.clock(node.latest_clock, now)
    assert SimBase.on_activate(node, None) == TransitionCallbackReturn.SUCCESS
    def clock(ns):
        return SimBase.clock_sample(node, SimpleNamespace(clock=Time(nanoseconds=ns).to_msg()))
    clock(1_000_000_000)
    clock(1_050_000_000)
    assert not node.guard.requested(node.latest_clock,time.monotonic())
    assert SimBase.on_activate(node, None) == TransitionCallbackReturn.FAILURE
    SimBase.tick(node)
    assert output[-1] == 0. and not node.guard.active
    wheel = Odometry()
    wheel.header.stamp = Time(nanoseconds=node.latest_clock).to_msg()
    wheel.header.frame_id, wheel.child_frame_id = 'odom', 'base_footprint'
    wheel.pose.pose.orientation.w = 1.
    SimBase.odometry(node, wheel)
    SimBase.tick(node)
    assert output[-1] == 0. and not node.guard.active
    assert SimBase.on_activate(node, None) == TransitionCallbackReturn.SUCCESS
    clock(1_100_000_000)
    command(node.guard, node.latest_clock, time.monotonic(), vx=.12, wz=0.)
    SimBase.tick(node)
    assert output[-1] == pytest.approx(.12)
    # The previous moving output is released by /clock at the original
    # deadline, with no timer tick or new final command in between.
    count = len(output)
    clock(1_399_999_999)
    assert len(output) == count
    clock(1_400_000_000)
    assert output[count:] == [0.]
    clock(1_405_000_000)
    assert output[count:] == [0.]


def test_wheel_odometry_keeps_payload_and_uses_covariance_cached_at_configure():
    from copy import deepcopy
    from types import SimpleNamespace
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from omslam_sim.sim_base import SimBase

    params = {'use_sim_time': True, 'left_radius': .068, 'right_radius': .068, 'wheel_separation': .480,
              'max_rpm': 70., 'clock_stall_timeout': .3, 'wheel_vx_variance': .0004, 'wheel_wz_variance': .0009}
    published = []
    node = SimpleNamespace(val=params.__getitem__, configured=False, latest_clock=None, clock_advanced_at=None,
                           guard=SimCommandGuard(),
                           output_moving=False, get_logger=lambda: SimpleNamespace(error=lambda *args, **kw: None),
                           output=SimpleNamespace(publish=lambda msg: None),
                           wheel=SimpleNamespace(publish=published.append))
    node.publish_velocity = lambda vx, wz: SimBase.publish_velocity(node, vx, wz)
    node.on_deactivate = lambda state: SimBase.on_deactivate(node, state)

    def wheel(ns):
        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=ns).to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
        pose, twist = msg.pose.pose, msg.twist.twist
        pose.position.x, pose.position.y, pose.position.z = 1.5, -2.25, .125
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = .01, -.02, .6, .8
        twist.linear.x, twist.linear.y, twist.linear.z = .3, .04, -.05
        twist.angular.x, twist.angular.y, twist.angular.z = .006, -.007, .8
        msg.pose.covariance = [float(i + 1) for i in range(36)]
        msg.twist.covariance = [float(100 + i) for i in range(36)]
        node.latest_clock = ns
        return msg

    def expected_covariance(diagonal):
        return [diagonal.get(i, 0.) for i in range(36)]

    assert SimBase.on_configure(node, None) == TransitionCallbackReturn.SUCCESS
    # Odometry uses the values validated at configure, not later reads.
    params['wheel_vx_variance'] = params['wheel_wz_variance'] = float('nan')
    raw = wheel(2_000_000_000)
    original = deepcopy(raw)
    SimBase.odometry(node, raw)
    output = published[-1]
    assert raw == original and output is not raw
    assert output.header == raw.header and output.child_frame_id == 'base_footprint'
    assert output.pose.pose == raw.pose.pose and output.twist.twist == raw.twist.twist
    assert list(output.pose.covariance) == expected_covariance({i: 1e3 for i in (0, 7, 14, 21, 28, 35)})
    assert list(output.twist.covariance) == expected_covariance(
        {0: .0004, 7: 1e3, 14: 1e3, 21: 1e3, 28: 1e3, 35: .0009})
    output.pose.pose.position.z = 9.
    output.twist.twist.angular.x = 9.
    assert raw == original

    assert SimBase.on_cleanup(node, None) == TransitionCallbackReturn.SUCCESS
    assert SimBase.on_configure(node, None) == TransitionCallbackReturn.FAILURE
    params['wheel_vx_variance'], params['wheel_wz_variance'] = .0016, .0036
    assert SimBase.on_configure(node, None) == TransitionCallbackReturn.SUCCESS
    SimBase.odometry(node, wheel(3_000_000_000))
    assert len(published) == 2
    assert published[-1].twist.covariance[0] == .0016 and published[-1].twist.covariance[35] == .0036
