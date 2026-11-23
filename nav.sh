#!/usr/bin/env bash
# Start Gazebo, online SLAM, standard Nav2, and RViz2 for goal-based navigation.
set -eo pipefail
OMSLAM_NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$OMSLAM_NAV_ROOT/install/setup.bash" ]]; then
  echo "설치본이 없습니다. 먼저 omslam_navigation까지 빌드하세요." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$OMSLAM_NAV_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${OMSLAM_NAV_DOMAIN_ID:-86}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${OMSLAM_GZ_PARTITION:-omslam_mini_${UID}_${ROS_DOMAIN_ID}}"

WORLD="p4_course"
if [[ "${1:-}" == "--world" && -n "${2:-}" ]]; then
  WORLD="$2"
  shift 2
fi

RVIZ_CONFIG="$OMSLAM_NAV_ROOT/install/omslam_navigation/share/omslam_navigation/rviz/navigation.rviz"
ros2 launch omslam_navigation slam_navigation.launch.py world:="$WORLD" "$@" &
NAV_PID=$!
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

cleanup() {
  kill -INT "$RVIZ_PID" "$NAV_PID" 2>/dev/null || true
  wait "$RVIZ_PID" "$NAV_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait -n "$NAV_PID" "$RVIZ_PID"
