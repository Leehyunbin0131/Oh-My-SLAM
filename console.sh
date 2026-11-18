#!/usr/bin/env bash
set -eo pipefail
OMSLAM_CONSOLE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OMSLAM_UI_PYTHON="$OMSLAM_CONSOLE_ROOT/.venv-operator/bin/python"
if [[ ! -x "$OMSLAM_UI_PYTHON" ]]; then
  echo 'Qt 환경이 없습니다. python3 -m venv --system-site-packages .venv-operator 후 requirements-operator.txt를 설치하세요.' >&2
  exit 1
fi
if [[ ! -f "$OMSLAM_CONSOLE_ROOT/install/setup.bash" ]]; then
  echo '설치본이 없습니다. ROS Jazzy 환경에서 colcon build --packages-up-to omslam_operator_support를 실행하세요.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$OMSLAM_CONSOLE_ROOT/install/setup.bash"
export OMSLAM_CONSOLE_ROOT
export ROS_DOMAIN_ID="${OMSLAM_CONSOLE_DOMAIN_ID:-${ROS_DOMAIN_ID:-86}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export GZ_PARTITION="${OMSLAM_GZ_PARTITION:-${GZ_PARTITION:-omslam_console_${UID}_${ROS_DOMAIN_ID}}}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
exec "$OMSLAM_UI_PYTHON" -m omslam_operator_support.session --workspace "$OMSLAM_CONSOLE_ROOT" "$@"
