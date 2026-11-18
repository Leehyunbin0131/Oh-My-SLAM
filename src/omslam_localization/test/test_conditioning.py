"""Safety-significant input handling without opening an IMU or starting ROS."""
from types import SimpleNamespace as NS
import json
import pytest
from omslam_localization.conditioning import Conditioner, load_bias


def sample(ns=1_000_000_000):
    return NS(header=NS(stamp=NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000), frame_id='imu_link'),
              angular_velocity=NS(x=.1, y=-.2, z=.5), angular_velocity_covariance=[0.]*9,
              orientation=NS(x=0., y=0., z=0., w=1.), orientation_covariance=[-1.]+[0.]*8,
              linear_acceleration=NS(x=0., y=0., z=0.), linear_acceleration_covariance=[-1.]+[0.]*8)


def test_bias_preserves_measurement_header_and_unavailable_fields():
    source = sample()
    output = Conditioner(bias=(.1, -.2, .05)).accept(source, 1_010_000_000)
    assert (output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z) == pytest.approx((0., 0., .45))
    assert output.header == source.header
    assert output.orientation_covariance[0] == -1.
    assert output.linear_acceleration_covariance[0] == -1.
    assert output.angular_velocity_covariance[8] == pytest.approx(1e-4)
    assert source.angular_velocity.z == .5
    assert source.angular_velocity_covariance == [0.]*9


def test_stale_future_duplicate_regression_and_clock_reset():
    c = Conditioner()
    c.accept(sample(), 1_010_000_000)
    for ns in (0, 700_000_000, 1_000_000_000, 999_999_999, 1_100_000_000):
        with pytest.raises(ValueError): c.accept(sample(ns), 1_010_000_000)
    # A new simulation epoch must not permanently suppress all following samples.
    c.accept(sample(500_000_000), 510_000_000)


def test_bad_frame_nan_and_missing_gyro_are_rejected():
    for mutate in (lambda s: setattr(s.header, 'frame_id', 'wrong_frame'),
                   lambda s: setattr(s.angular_velocity, 'z', float('nan')),
                   lambda s: s.angular_velocity_covariance.__setitem__(0, -1.)):
        msg = sample(); mutate(msg)
        with pytest.raises(ValueError): Conditioner().accept(msg, 1_010_000_000)


def test_known_covariance_is_retained():
    msg = sample(); msg.angular_velocity_covariance = [.01, .001, 0., .001, .02, 0., 0., 0., .03]
    output = Conditioner().accept(msg, 1_010_000_000)
    assert output.angular_velocity_covariance == msg.angular_velocity_covariance


def test_existing_bias_profile_is_read_only_and_identity_checked(tmp_path):
    path = tmp_path/'profile.json'
    profile = {'schema': 1, 'enabled_for_navigation': False, 'sensors': {'imu': {
        'model': 'HFI-A9', 'identity': 'device_a', 'coordinate_frame': 'hfi_a9_packet_xyz',
        'gyro_units': 'rad/s', 'gyro_bias_rad_s': [.01, -.02, .03], 'mounting_transform_calibrated': False, 'validation': {'stationary': True}}}}
    path.write_text(json.dumps(profile))
    before = path.read_bytes()
    assert load_bias(path, 'device_a') == (.01, -.02, .03)
    assert path.read_bytes() == before
    with pytest.raises(ValueError): load_bias(path, 'device_b')
    profile['sensors']['imu']['gyro_units'] = 'deg/s'; path.write_text(json.dumps(profile))
    with pytest.raises(ValueError): load_bias(path)

    profile['sensors']['imu']['gyro_units'] = 'rad/s'
    profile['sensors']['imu']['validation']['stationary'] = False
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError): load_bias(path)
