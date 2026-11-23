"""RPM settings bound each wheel without raising small input commands."""
import math
from types import SimpleNamespace

import pytest

from omslam_base.core import Geometry
from omslam_navigation.guard_core import Limits
from omslam_navigation.operator_control import OperatorControl
from omslam_navigation.operator_core import OperatorGate
from omslam_operator_msgs.srv import SetSpeedLimit


PROFILE_VX = 2 * math.pi * .068 * 70. / 60.
PROFILE_WZ = .5
ROTATION_FLOOR = 17 * math.pi / 600


def axes(vx, wz):
    return (vx, 0., 0., 0., 0., wz)


def service_fixture(core):
    now = 1_000_000_000
    assert core.acquire('console', now, 10.)[0]
    published = []
    return SimpleNamespace(core=core, authorized=lambda token, epoch: core.authorized(token, epoch, now, 10.),
                           publish_speed=lambda: published.append(core.speed_percent)), published


def speed_request(core, rpm, *, legacy=False, linear=None, angular=None):
    return SetSpeedLimit.Request(token=core.token, epoch=core.epoch_ns,
                                 linear_limit=core.limits.max_vx if linear is None else linear,
                                 angular_limit=core.limits.max_wz if angular is None else angular,
                                 speed_percent=rpm / core.geometry.max_rpm * 100.,
                                 wheel_rpm_limit=0. if legacy else rpm)


@pytest.mark.parametrize('rpm', [3., 17.34567890123456, 35., 70.])
@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_service_caps_pure_axes_and_preserves_mixed_curvature(mode, rpm):
    core = OperatorGate(Limits(max_vx=PROFILE_VX, max_wz=PROFILE_WZ))
    service, published = service_fixture(core)
    request = speed_request(core, rpm)
    response = OperatorControl.speed(service, request, SetSpeedLimit.Response())
    assert response.success and response.wheel_rpm_limit == rpm
    assert published == [request.speed_percent]
    core.mode = mode
    linear = PROFILE_VX * (rpm / 70.)
    angular = max(PROFILE_WZ * (rpm / 70.), ROTATION_FLOOR)
    assert response.linear_limit == pytest.approx(linear)
    assert response.angular_limit == pytest.approx(angular)
    out = core.limit_values(axes(PROFILE_VX, 0.))
    assert core.geometry.rpm(out[0], out[5]) == pytest.approx((rpm, rpm))
    for sign in (-1., 1.):
        out = core.limit_values(axes(0., sign * PROFILE_WZ))
        assert out[5] == pytest.approx(sign * angular)
        assert all(core.geometry.rpm(out[0], out[5]))
        if rpm == 3.:
            assert core.geometry.rpm(out[0], out[5]) == (-sign * 3., sign * 3.)
    small = axes(linear * .001, angular * .001)
    assert core.limit_values(small) == small
    assert core.geometry.rpm(small[0], small[5]) == (0., 0.)
    out = core.limit_values(axes(PROFILE_VX, PROFILE_WZ))
    assert out[5] / out[0] == pytest.approx(angular / linear)
    assert max(map(abs, core.geometry.rpm(out[0], out[5]))) == pytest.approx(rpm)
    assert core.limit_values(axes(-PROFILE_VX, 0.))[0] == 0.
    core.reverse_allowed = True
    assert core.limit_values(axes(-PROFILE_VX, 0.))[0] == (-linear if mode == 'MANUAL' else 0.)


def test_manual_residual_translation_is_removed_before_rotation_cap():
    core = OperatorGate()
    service, _ = service_fixture(core)
    assert OperatorControl.speed(service, speed_request(core, 3.), SetSpeedLimit.Response()).success
    core.mode = 'MANUAL'
    core.sources[42] = SimpleNamespace(linear_requested=False)
    v, *_, w = core.limit_final(42, axes(.0002, core.rotation_floor))
    assert v == 0.
    assert core.geometry.rpm(v, w) == (-3., 3.)
    request = speed_request(core, 3., linear=.01, angular=.03)
    response = OperatorControl.speed(service, request, SetSpeedLimit.Response())
    assert response.success and (response.linear_limit, response.angular_limit) == (.01, .03)
    assert core.limit_values(axes(.01, 0.))[0] == .01
    assert core.limit_values(axes(0., core.rotation_floor))[5] == .03
    core.mode = 'AUTO'
    assert core.limit_values(axes(0., core.rotation_floor))[5] == core.rotation_floor


@pytest.mark.parametrize('maximum', [70., 95.])
def test_legacy_percentage_and_rejections_leave_selected_state_unchanged(maximum):
    core = OperatorGate(geometry=Geometry(max_rpm=maximum))
    service, published = service_fixture(core)
    for rpm in (3., 21.25, maximum):
        request = speed_request(core, rpm, legacy=True)
        response = OperatorControl.speed(service, request, SetSpeedLimit.Response())
        assert response.success and response.wheel_rpm_limit == pytest.approx(rpm)
    before = core.linear_limit, core.angular_limit, core.speed_percent, core.wheel_rpm_limit
    invalid = [speed_request(core, rpm) for rpm in (0., 2.999, 2.9999995, 2.9999999995, maximum + 1., -1., math.nan, math.inf)]
    invalid.append(speed_request(core, 2.9999995, legacy=True))
    for field, value in (('speed_percent', 100. - 1e-7), ('token', 'invalid'), ('epoch', 1),
                         ('linear_limit', math.nan), ('angular_limit', math.inf)):
        request = speed_request(core, maximum)
        setattr(request, field, value)
        invalid.append(request)
    for request in invalid:
        response = OperatorControl.speed(service, request, SetSpeedLimit.Response())
        assert not response.success
        assert before == (core.linear_limit, core.angular_limit, core.speed_percent, core.wheel_rpm_limit)
    assert len(published) == 3


def test_calibrated_asymmetric_radii_keep_floor_straight_speed():
    geometry = Geometry(left_radius=.069, right_radius=.070, max_rpm=70., min_rpm=3.)
    core = OperatorGate(Limits(max_vx=PROFILE_VX, max_wz=PROFILE_WZ), geometry=geometry)
    service, _ = service_fixture(core)
    assert OperatorControl.speed(service, speed_request(core, 3.), SetSpeedLimit.Response()).success
    for mode in ('MANUAL', 'AUTO'):
        core.mode = mode
        out = core.limit_values(axes(PROFILE_VX, 0.))
        assert max(map(abs, geometry.rpm(out[0], out[5]))) == pytest.approx(3.)
