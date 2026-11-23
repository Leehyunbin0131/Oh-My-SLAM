"""Arbitrary RPM settings bound each wheel without changing curvature."""
import math
from types import SimpleNamespace
import pytest
from omslam_base.core import Geometry
from omslam_navigation.guard_core import Limits
from omslam_navigation.operator_core import OperatorGate
from omslam_navigation.operator_control import OperatorControl
from omslam_operator_msgs.srv import SetSpeedLimit


# Installed simulation profile, computed from 70 RPM and the 0.068 m wheel radius.
PROFILE_VX = 2 * math.pi * .068 * 70. / 60.
PROFILE_WZ = .5


def outer_wheel_rpm(v, w):
    return (abs(v) + abs(w) * .480 / 2) / .068 * 30 / math.pi


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_rpm_limit_caps_turning_wheels_and_preserves_curvature(mode):
    g = OperatorGate(Limits(max_vx=PROFILE_VX, max_wz=PROFILE_WZ))
    assert g.geometry.max_rpm == 70.
    g.mode = mode
    g.speed_percent = 37.5
    g.linear_limit = g.limits.max_vx * .375
    g.angular_limit = g.limits.max_wz * .375
    g.wheel_rpm_limit = g.geometry.max_rpm * .375
    # The profile's own linear and angular limits overdrive the outer wheel.
    assert outer_wheel_rpm(g.linear_limit, g.angular_limit) > g.wheel_rpm_limit * 1.1
    v, _, _, _, _, w = g.limit_values((g.linear_limit,0.,0.,0.,0.,g.angular_limit))
    assert w/v == pytest.approx(g.angular_limit/g.linear_limit)
    assert max(abs(r) for r in g.geometry.rpm(v,w)) == pytest.approx(g.wheel_rpm_limit)
    g.wheel_rpm_limit = 1.
    v, _, _, _, _, w = g.limit_values((v,0.,0.,0.,0.,w))
    assert max(abs(r) for r in g.geometry.rpm(v,w)) <= 1. + 1e-9


def test_speed_service_accepts_fractional_rpm_and_rejects_invalid_without_mutation():
    core=OperatorGate()
    sent=[]
    fixture=SimpleNamespace(core=core,authorized=lambda *_:True,publish_speed=lambda:sent.append(True))
    req=SetSpeedLimit.Request(linear_limit=.0468,angular_limit=.1404,speed_percent=23.4 / core.geometry.max_rpm * 100.,wheel_rpm_limit=23.4)
    res=OperatorControl.speed(fixture,req,SetSpeedLimit.Response())
    assert res.success and res.wheel_rpm_limit==23.4 and len(sent)==1
    for rpm in [0.5,101.,-1.,float('nan'),float('inf')]:
        req.wheel_rpm_limit=rpm
        assert not OperatorControl.speed(fixture,req,SetSpeedLimit.Response()).success
        assert core.wheel_rpm_limit==23.4
    req.wheel_rpm_limit=25.
    assert not OperatorControl.speed(fixture,req,SetSpeedLimit.Response()).success
    fixture.authorized=lambda *_:False
    req.wheel_rpm_limit=23.4
    assert not OperatorControl.speed(fixture,req,SetSpeedLimit.Response()).success
    assert len(sent)==1
