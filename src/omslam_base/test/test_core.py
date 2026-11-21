import math
import pytest
from omslam_base.core import Geometry,EncoderOdometry,CommandGate


def test_kinematics_and_curvature_limit():
    g=Geometry()
    l,r=g.rpm(.2,0.)
    assert l==r and l==pytest.approx(.2/.068*30/math.pi)
    l,r=g.rpm(0.,.5)
    assert l<0<r and l==-r
    raw=Geometry(max_rpm=95).rpm(3.,2.)
    assert max(abs(x) for x in raw)==95
    assert raw[0]/raw[1]==pytest.approx((3.-.48)/(3.+.48))


def test_measured_encoder_se2_and_time_wrap():
    g=Geometry();o=EncoderOdometry(g)
    o.update(0xfffffff0,0,0,1000000000)
    s=o.update(34,10,10,1050000000)
    assert s['x']==pytest.approx(.068*20*math.pi/2652)
    assert s['y']==0 and s['yaw']==0
    assert s['vx']==pytest.approx(s['x']/.05)
    assert s['stamp_ns']==1050000000
    assert o.update(34,10,10,1051000000) is None
    with pytest.raises(ValueError):o.update(34,11,10,1051000000)
    with pytest.raises(ValueError):o.update(10,10,10,1060000000)
    turn=EncoderOdometry(g);turn.update(0,0,0,1000000000)
    s=turn.update(50,-10,10,1050000000)
    assert s['x']==0 and s['yaw']>0
    arc=EncoderOdometry(g);arc.update(0,0,0,1000000000)
    s=arc.update(50,0,10,1050000000)
    angle=.068*20*math.pi/2652/.48
    assert s['x']==pytest.approx(.24*math.sin(angle))
    assert s['y']==pytest.approx(.24*(1-math.cos(angle)))


def test_lease_source_final_stop_and_activation():
    gate=CommandGate();gate.activate(1000000000)
    assert not gate.requested(990000000,1000000000,10.)
    assert gate.requested(1010000000,1020000000,10.02)
    assert not gate.final(1020000000,1.,0.,1020000000,10.02)
    assert gate.final(1010000000,1.,0.,1020000000,10.02)
    issued=gate.current(10.02)[2]
    assert gate.requested(1010000000,1200000000,10.20)
    assert gate.final(1010000000,.5,0.,1200000000,10.20)
    assert gate.current(10.20)[2]==issued
    assert gate.final(1010000000,0.,0.,1250000000,10.25)
    assert not gate.final(1010000000,1.,0.,1251000000,10.251)
    assert gate.current(10.32) is None
    # An unmatched fresh stop must clear the target persistently, even when
    # its previous source is still within the 300 ms command lease.
    assert gate.requested(1350000000,1350000000,10.35)
    assert gate.final(1350000000,.2,0.,1350000000,10.35)
    assert gate.current(10.36) is not None
    assert gate.final(1380000000,0.,0.,1380000000,10.38)
    assert gate.current(10.39) == (0.,0.,10.35)
    assert gate.requested(1350000000,1400000000,10.40)
    assert not gate.final(1350000000,.2,0.,1400000000,10.40)
    # A delayed source older than the stop also cannot restart the output.
    assert gate.requested(1370000000,1410000000,10.41)
    assert not gate.final(1370000000,.2,0.,1410000000,10.41)
    assert gate.current(10.42) == (0.,0.,10.35)
    assert gate.requested(1430000000,1430000000,10.43)
    assert gate.final(1430000000,.2,0.,1430000000,10.43)
    assert gate.current(10.44)[:2] == (.2,0.)
    gate.clear()
    assert not gate.requested(1400000000,1400000000,10.4)
    gate.activate(1500000000)
    assert not gate.final(1400000000,1.,0.,1500000000,10.5)


def test_encoder_reset_and_nonfinite_geometry():
    with pytest.raises(ValueError, match='overflow'):
        Geometry().rpm(-1e308, 0.)
    with pytest.raises(ValueError):Geometry(left_radius=float('nan'))
    with pytest.raises(ValueError):Geometry(max_rpm=100.01)
    o=EncoderOdometry(Geometry());o.update(0,20000,20000,1000000000)
    with pytest.raises(ValueError):o.update(50,0,0,1050000000)


def test_explicit_100_rpm_geometry_preserves_physical_default_and_curvature():
    assert Geometry().max_rpm == 95.0
    geometry = Geometry(max_rpm=100.0)
    maximum = 2 * math.pi * .068 * 100 / 60
    assert geometry.rpm(maximum, 0.) == pytest.approx((100., 100.))
    left, right = geometry.rpm(maximum, .6)
    assert max(abs(left), abs(right)) == pytest.approx(100.)
    assert left / right == pytest.approx((maximum - .6 * .480 / 2) / (maximum + .6 * .480 / 2))


@pytest.mark.parametrize('drift', [-.002, .002])
def test_encoder_clock_tracks_hour_of_drift_without_changing_integration(drift):
    odometry = EncoderOdometry(Geometry())
    start = 0xfffffff0
    odometry.update(start, 0, 0, 1_000_000_000)
    previous_stamp = odometry.ros_stamp_ns
    max_offset = 0
    for index in range(1, 72_001):
        host_ms = index * 50
        tick = (start + round(host_ms * (1. + drift))) & 0xffffffff
        # Bounded alternating receipt jitter must not become integrated travel.
        receipt = 1_000_000_000 + host_ms * 1_000_000 + (index % 3) * 1_000_000
        old_tick = odometry.last[0]
        state = odometry.update(tick, index * 10, index * 10, receipt)
        assert state['stamp_ns'] > previous_stamp
        previous_stamp = state['stamp_ns']
        max_offset = max(max_offset, abs(state['stamp_ns'] - receipt))
        expected_vx = .068 * 20 * math.pi / 2652 / (((tick - old_tick) & 0xffffffff) / 1000)
        assert state['vx'] == pytest.approx(expected_vx)
    assert max_offset < 10_000_000
    assert state['x'] == pytest.approx(72_000 * .068 * 20 * math.pi / 2652)


def test_encoder_clock_jump_rejection_does_not_integrate_failed_sample():
    odometry = EncoderOdometry(Geometry())
    odometry.update(100, 0, 0, 1_000_000_000)
    odometry.update(150, 10, 10, 1_050_000_000)
    before = vars(odometry).copy()
    for receipt in (1_049_000_000, 1_600_000_000):
        with pytest.raises(ValueError):
            odometry.update(200, 20, 20, receipt)
        assert vars(odometry) == before
    assert odometry.update(200, 20, 20, 1_100_000_000)['x'] > before['x']


def test_requested_issue_time_cannot_reverse_or_extend_a_lease():
    gate = CommandGate()
    gate.activate(1_000_000_000)
    assert gate.requested(1_100_000_000, 1_110_000_000, 10.01)
    issued = gate.source[1]
    assert gate.final(1_100_000_000, .1, 0., 1_110_000_000, 10.01)
    # A newer ROS source maps backwards by 1 ms due to receipt sampling.
    assert not gate.requested(1_101_000_000, 1_113_000_000, 10.011)
    assert gate.source == (1_100_000_000, issued)
    assert not gate.final(1_101_000_000, .2, 0., 1_113_000_000, 10.011)
    assert gate.requested(1_150_000_000, 1_160_000_000, 10.06)
    assert gate.source[1] >= issued
    assert gate.final(1_150_000_000, .2, 0., 1_160_000_000, 10.06)
    issued = gate.source[1]
    assert gate.requested(1_150_000_000, 1_250_000_000, 10.2)
    assert gate.source[1] == issued  # repeated stamp cannot refresh the lease
    assert gate.current(issued + .301) is None


@pytest.mark.parametrize('limit,vx,wz', [(50., .34, .52), (95., .85, .47)])
def test_saturated_velocity_is_within_serial_rpm_limit(limit, vx, wz):
    from types import SimpleNamespace
    from omslam_base.vendor import load_control_module

    drive_module = load_control_module('wheel_drive')
    calibration = SimpleNamespace(wheels={})
    wire = []
    client = SimpleNamespace(calibration=calibration, info={'stream_supported': True},
                             latest={'received_at': 1., 'fault': 0}, command=lambda op: None,
                             send=wire.append, set_speed=lambda *rpm, lease_ms: wire.append(rpm))
    cfg = SimpleNamespace(calibration=calibration, max_rpm=limit, commands=lambda *rpm: rpm)
    drive = drive_module.WheelDrive(client, cfg, clock=lambda: 1.)
    for sign in (-1., 1.):
        left, right = Geometry(max_rpm=limit).rpm(vx * sign, wz * sign)
        assert abs(right) == limit and abs(left) <= limit
        drive.set_target_rpm(left, right)
        drive.tick(force=True)
        assert wire[-1] == (round(left*100), sign*int(limit*100))
    with pytest.raises(drive_module.MotorError):
        drive.set_target_rpm(0., limit+.01)


def test_out_of_order_final_preserves_original_lease_and_stop_epoch():
    gate = CommandGate()
    gate.activate(1000000000)
    one, two = 1010000000, 1030000000
    assert gate.requested(one, one-1000000, 4.01)
    assert gate.requested(two, two, 4.03)
    assert gate.final(one, .1, 0., two, 4.04)
    assert gate.current(4.311) is None
    assert gate.final(two, .1, 0., two, 4.312)
    assert not gate.final(one, .1, 0., two, 4.313)
    assert gate.final(two, 0., 0., two, 4.314)
    assert not gate.final(two, .1, 0., two, 4.315)
