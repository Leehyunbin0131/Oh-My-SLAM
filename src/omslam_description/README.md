# Robot model and simulated sensors

`omslam_description.model.simulation_urdf(settings=None)` resolves the installed
package and returns the pinned CAD base robot with Gazebo-only dynamics and
sensors. It does not require the upstream repository checkout, Fuel, or a USB
device. The source base URDF and referenced STL meshes are preserved verbatim.

The wrapper retains the original wheel radius 0.068 m, separation 0.480 m, +Y
wheel axes, base frame height and sensor geometry. Added masses/friction are
simulation assumptions. Optional settings cannot silently change the CAD wheel
geometry. The upstream source and mesh/license hashes are in `provenance.json`.

| Output | Gazebo topic | Frame / rate |
| --- | --- | --- |
| Wheel odometry | `/model/oh_my_mobilerobot/odometry` | odom → base_footprint, 50 Hz |
| Joint states | `/model/oh_my_mobilerobot/joint_state` | Original joint names, 50 Hz requested |
| LiDAR | `/scan` | laser_frame, 10 Hz, 720 rays, 0.15–12 m |
| External IMU | `/imu` | imu_link, 50 Hz |
| RGB / depth / intrinsics | `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | camera_optical_frame, 320×240, 10 Hz |
| Verification truth | `/model/oh_my_mobilerobot/ground_truth` | sim_world → base_footprint, 50 Hz |

DiffDrive commands use `/model/oh_my_mobilerobot/cmd_vel`. Wheel limits are 95 RPM;
the ROS adapter also enforces coupled wheel limits while preserving curvature.
DiffDrive's Gazebo TF topic is deliberately unbridged; EKF alone owns the ROS
`odom → base_footprint` transform. Ground truth is for verification only.

IMU mounting is an explicit identity transform from base_link to imu_link in
this simulation. It is not the measured physical HFI-A9 mounting. RGB-D uses
the original camera_link pose and optical frame convention: Z forward, X right,
Y down. The world must load the Gazebo Imu and Sensors systems.

The wrapper is consumed by `omslam_sim/sim.launch.py`; launch, topic remapping,
stamped command handling and runtime verification belong to that package.
