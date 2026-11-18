"""IMU acceptance and sensor-frame bias correction; no ROS or hardware access."""
from copy import deepcopy
import json
import math
from pathlib import Path


def vector3(values, name, positive=False):
    values = tuple(float(v) for v in values)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f'{name} requires three finite values')
    if positive and not all(v > 0 for v in values):
        raise ValueError(f'{name} requires positive variances')
    return values


def load_bias(path, identity=''):
    """Read an explicitly selected existing profile without modifying it.

    Bias calibration is distinct from navigation/mounting acceptance. Reading
    this profile does not certify its enabled_for_navigation flag or the mount.
    """
    profile = json.loads(Path(path).read_text())
    sensor = profile['sensors']['imu']
    if profile.get('schema') != 1 or sensor.get('model') != 'HFI-A9':
        raise ValueError('Expected schema 1 HFI-A9 calibration')
    if sensor.get('gyro_units') != 'rad/s' or sensor.get('coordinate_frame') != 'hfi_a9_packet_xyz':
        raise ValueError('Calibration gyro units or sensor axes do not match')
    if sensor.get('validation', {}).get('stationary') is not True:
        raise ValueError('Calibration lacks a confirmed stationary measurement')
    if identity and sensor.get('identity') != identity:
        raise ValueError('Calibration sensor identity does not match')
    return vector3(sensor['gyro_bias_rad_s'], 'gyro_bias_rad_s')


class Conditioner:
    def __init__(self, frame='imu_link', bias=(0., 0., 0.), variance=(1e-4, 1e-4, 1e-4),
                 max_age_s=.2, future_tolerance_s=.02):
        if not frame or frame.startswith('/'):
            raise ValueError('Expected nonempty relative sensor frame')
        self.frame = frame
        self.bias = vector3(bias, 'bias')
        self.variance = vector3(variance, 'variance', positive=True)
        if not math.isfinite(max_age_s) or max_age_s <= 0 or not math.isfinite(future_tolerance_s) or future_tolerance_s < 0:
            raise ValueError('Invalid IMU freshness limits')
        self.max_age_ns = int(max_age_s * 1e9)
        self.future_ns = int(future_tolerance_s * 1e9)
        self.last_stamp = None
        self.last_now = None

    def accept(self, msg, now_ns):
        if self.last_now is not None and now_ns < self.last_now:
            self.last_stamp = None
        self.last_now = now_ns
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg.header.frame_id != self.frame:
            raise ValueError('unexpected IMU frame')
        if stamp <= 0 or stamp > now_ns + self.future_ns or now_ns - stamp > self.max_age_ns:
            raise ValueError('invalid, future or stale IMU timestamp')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('duplicate or regressed IMU timestamp')
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        vector3(gyro, 'gyro')
        covariance = list(msg.angular_velocity_covariance)
        if covariance[0] == -1:
            raise ValueError('angular velocity unavailable')
        if not all(math.isfinite(v) for v in covariance):
            raise ValueError('invalid gyro covariance')
        if not any(covariance):
            covariance = [self.variance[0], 0., 0., 0., self.variance[1], 0., 0., 0., self.variance[2]]
        elif any(covariance[i] <= 0 for i in (0, 4, 8)):
            raise ValueError('gyro covariance must have positive diagonal')
        output = deepcopy(msg)
        output.angular_velocity.x = gyro[0] - self.bias[0]
        output.angular_velocity.y = gyro[1] - self.bias[1]
        output.angular_velocity.z = gyro[2] - self.bias[2]
        output.angular_velocity_covariance = covariance
        # Preserve orientation and acceleration including their -1 markers.
        # Mount correction is performed by robot_localization using TF.
        self.last_stamp = stamp
        return output
