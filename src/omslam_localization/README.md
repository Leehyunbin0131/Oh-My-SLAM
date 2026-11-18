# Wheel and IMU local odometry

`robot_localization` fuses `/wheel/odometry` forward velocity and yaw rate with
one IMU's three-axis gyro rotated into the body yaw rate. It publishes `/odom` and is the sole publisher of
`odom -> base_footprint`. It does not fuse wheel pose, magnetic yaw, acceleration,
LiDAR scans, SLAM pose, or control commands.

`imu_conditioner` subscribes to `/imu/raw` (`sensor_msgs/Imu`, sensor-data QoS),
subtracts a configured gyro bias in the **sensor axes**, and publishes `/imu/data`
with the original frame and timestamp. The mount rotation is applied once by the
EKF using TF. All three sensor-frame gyro inputs are selected because the body
yaw axis can project onto any sensor axis; `two_d_mode` constrains the transformed
body roll/pitch rates afterward. Missing gyro, wrong frames, nonfinite values, expired/future or
repeated/regressed stamps are rejected; no timer replays an old sample. Unknown
orientation/acceleration flags are preserved. All-zero gyro covariance uses the
explicit initial variance from `imu.yaml`; these example values are not a
physical noise calibration. A missing sensor produces diagnostics; the EKF can
continue predicting, so fresh `/odom` alone does not prove fresh sensors.

## Launch

```bash
ros2 launch omslam_localization base_ekf.launch.py
```

Default backend is mock, base remains unconfigured, and IMU source is external.
The combined mock launch supplies example `base_footprint -> base_link -> imu_link`
static TF. A verifier supplies independent encoder counts and IMU measurements;
no target velocity is passed off as measured motion.

`localization.launch.py` runs only conditioning and EKF. It defaults
`publish_static_tfs:=false` for use with a robot description. Use one TF source:
if robot_state_publisher already supplies these edges, do not enable duplicate
static transforms. Standalone/combined launch accepts `imu_x/y/z` in metres and
`imu_roll/pitch/yaw` in radians relative to base_link. The default `.101 m`
base_link height is the model value; the identity IMU example is not a measured
real mount.

An explicitly synthetic source is available with `imu_source:=mock`; its
`/mock_imu` parameter `yaw_rate` sets a constant sensor-Z rate independent of wheel
measurements. It is a software fixture, not a dynamic sensor simulation.

For hardware, set `backend:=real`, a real `base_config`, and verified mount/TF.
The combined launch maps `real` to the base bridge's `serial` backend.
`mounting_verified:=true` is an operator assertion after physical checks, not a
calibration algorithm. The built-in read-only source additionally requires
`imu_source:=hfi_a9 imu_port:=/dev/serial/by-id/…`; baud defaults to 921600.
It uses the pinned HFI decoder from `omslam_control_vendor` and opens the port
exclusively. No device configuration commands are sent. It publishes gyro only,
keeps the latest motion frame of a serial batch, drops duplicate ticks, and
latches regressions/serial failures until restart. Timestamps are **host receive
time**: device clock units and serial latency are not calibrated by this adapter.
No serial device is opened by default or by software validation.

Set `imu_config:=/absolute/path.yaml` to supply sensor-frame bias and variance.
An explicitly selected `calibration_file` can load `sensors.imu.gyro_bias_rad_s`
from the existing schema-1 HFI-A9 calibration. Units, axes and a confirmed stationary calibration must match;
`expected_identity` optionally requires the recorded identity. Reading bias
neither changes `enabled_for_navigation` nor certifies mounting or dynamics.
The original calibration file is never modified. When an upstream driver already
subtracts bias, leave this correction zero to avoid subtracting it twice.

LiDAR belongs to SLAM/AMCL and obstacle sensing in this architecture. A separate
LiDAR odometry estimator would need its own measurement and covariance contract;
raw LaserScan is not an EKF input here.
