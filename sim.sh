#!/usr/bin/env bash
set -e
OMSLAM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$OMSLAM_WORKSPACE/install/setup.bash" ]]; then
  echo "Build this workspace first: source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to omslam_sim" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$OMSLAM_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${OMSLAM_SIM_DOMAIN_ID:-86}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${OMSLAM_GZ_PARTITION:-omslam_airport_${UID}_${ROS_DOMAIN_ID}}"
# Keep local Gazebo publishers reachable when an external network changes.
export GZ_IP="${GZ_IP:-127.0.0.1}"
export ROS_LOG_DIR="${OMSLAM_REPORT_DIR:-$OMSLAM_WORKSPACE/reports/airport/live}/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch omslam_sim sim.launch.py "$@"
