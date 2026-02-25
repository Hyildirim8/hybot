#!/usr/bin/env bash
# scripts/rviz.sh — Launch RViz2 on the host desktop to visualise the rover.
#
# Prerequisites:
#   - ROS 2 Humble installed on the host (or run inside a GUI container)
#   - ecza_description package built and sourced, OR use the --docker flag
#   - Docker stack running (robot_description service publishes /robot_description
#     and TF via network_mode: host)
#
# Usage:
#   ./scripts/rviz.sh                  # host-native ROS2
#   ./scripts/rviz.sh --sim            # use_sim_time:=true (for bag playback)
#
# The RViz config is at:
#   src/ecza_description/rviz/rover.rviz
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
RVIZ_CONFIG="${REPO_ROOT}/src/ecza_description/rviz/rover.rviz"
USE_SIM_TIME="false"

# Parse flags
for arg in "$@"; do
  case "$arg" in
    --sim) USE_SIM_TIME="true" ;;
    -h|--help)
      echo "Usage: $0 [--sim]"
      echo "  --sim   pass use_sim_time:=true (for bag playback)"
      exit 0
      ;;
  esac
done

# Source ROS2 if available
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# Source workspace install if it exists
WS_INSTALL="${REPO_ROOT}/install/setup.bash"
if [[ -f "${WS_INSTALL}" ]]; then
  # shellcheck disable=SC1090
  source "${WS_INSTALL}"
fi

echo "[rviz.sh] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"
echo "[rviz.sh] use_sim_time=${USE_SIM_TIME}"
echo "[rviz.sh] config=${RVIZ_CONFIG}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Launch display.launch.py — starts robot_state_publisher + joint_state_publisher_gui + RViz2
ros2 launch ecza_description display.launch.py \
  use_sim_time:="${USE_SIM_TIME}" \
  rvizconfig:="${RVIZ_CONFIG}"
