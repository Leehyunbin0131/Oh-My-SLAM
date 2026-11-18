# omslam_mapping

2D SLAM and saved-map AMCL localization above the existing simulation and wheel/IMU EKF.
Run the robot with `./sim.sh`, then source `install/setup.bash` in another terminal.

```bash
export ROS_DOMAIN_ID=86
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch omslam_mapping mapping.launch.py
ros2 run omslam_mapping save_map --output "$PWD/maps/my_map" \
  --backend simulation --workspace "$PWD"
```

The save command creates `.yaml`, `.pgm`, `.posegraph`, `.data`, and `.metadata.json`.
The metadata records the source commit and dirty flag, installed default configuration hashes,
effective SLAM parameters, geometry/calibration configuration hashes, pinned dependency commits,
the robot model provenance hash and commit, package versions, and
the saved file hashes. Defaults describe simulation assumptions, not physical calibration.
Specify `--geometry-config` and `--calibration-config` when using different configurations.
An existing or partial bundle is never overwritten; choose a new prefix after a failed save.

Stop mapping before starting localization:

```bash
ros2 launch omslam_mapping localization.launch.py map:="$PWD/maps/my_map.yaml"
```

Set the starting pose with `/initialpose` (for example RViz's **2D Pose Estimate**), or pass
`initial_pose:=true x:=0.0 y:=0.0 yaw:=0.0`. Coordinates are in the saved map frame.
`mode.launch.py mode:=mapping|localization` is the equivalent common entry point.
All launch files accept `use_sim_time:=true|false` and a backend `params_file:=…` override.

To continue a saved graph, start mapping and run:

```bash
ros2 run omslam_mapping load_graph --input "$PWD/maps/my_map" --match first
```

Use `--match pose --x … --y … --yaw …` to match at another map pose. The CLI checks the graph
hashes when bundle metadata exists. The installed deserialize service has an empty response;
service completion alone is not a successful-load assertion. Check the resulting map and graph.

The selected backend alone publishes `map -> odom`; the existing EKF keeps `odom -> base_footprint`.
A launch-held lock rejects another package mode on the same user, host, and ROS domain. It does
not coordinate unrelated or remote launches. Ground truth is never a SLAM or AMCL input.
The package starts no simulator, hardware driver, velocity publisher, or Nav2 controller.

See the workspace's `docs/sim_mapping.md` for the complete simulation workflow and validation.
