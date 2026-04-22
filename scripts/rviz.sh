#!/usr/bin/env bash
# scripts/rviz.sh — Launch RViz2 on the host desktop to visualise the rover.
#
# Prerequisites:
#   - ROS 2 Humble installed on the host (or run via --docker flag below)
#   - ecza_description package built and sourced, OR use the --docker flag
#   - Docker stack running (robot_description service publishes /robot_description
#     and TF via network_mode: host)
#
# Usage:
#   ./scripts/rviz.sh              # host-native ROS2
#   ./scripts/rviz.sh --docker     # run rviz inside the ecza-robotu container
#   ./scripts/rviz.sh --sim        # use_sim_time:=true (for bag playback)
#
# Display on Raspberry Pi OS (labwc + wayvnc):
#   Wayland-native rendering is used by default (QT_QPA_PLATFORM=wayland).
#   The Wayland socket is at /run/user/<UID>/wayland-0.
#   Fallback to X11/XWayland (:0) is available if Wayland fails.
#
# The RViz config is at:
#   src/ecza_description/rviz/rover.rviz
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
RVIZ_CONFIG="${REPO_ROOT}/src/ecza_description/rviz/rover.rviz"
USE_SIM_TIME="false"
USE_DOCKER="false"

# Parse flags
for arg in "$@"; do
  case "$arg" in
    --sim) USE_SIM_TIME="true" ;;
    --docker) USE_DOCKER="true" ;;
    -h|--help)
      echo "Usage: $0 [--sim] [--docker]"
      echo "  --sim     pass use_sim_time:=true (for bag playback)"
      echo "  --docker  launch rviz via docker compose (no host ROS2 needed)"
      exit 0
      ;;
  esac
done

if [[ "${USE_DOCKER}" == "true" ]]; then
  echo "[rviz.sh] Launching rviz via docker compose..."

  # Export UID/GID explicitly — bash does not export GID by default.
  export UID GID
  # GID may be unset in bash; fall back to the primary group of the current user.
  GID="${GID:-$(id -g)}"
  export GID

  # ── Display setup ────────────────────────────────────────────────────────
  # Raspberry Pi OS runs labwc (Wayland) + wayvnc. Try Wayland first; fall
  # back to X11/XWayland if the Wayland socket is absent.
  WAYLAND_SOCK="/run/user/${UID}/wayland-0"
  if [[ -S "${WAYLAND_SOCK}" ]]; then
    echo "[rviz.sh] Using Wayland display (${WAYLAND_SOCK})"
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${UID}}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
    # DISPLAY still needed as fallback for any X11-only Qt plugins
    export DISPLAY="${DISPLAY:-:0}"
  else
    echo "[rviz.sh] Wayland socket not found — falling back to X11/XWayland"
    export DISPLAY="${DISPLAY:-:0}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    # Grant Docker containers access to the X server (idempotent).
    if command -v xhost &>/dev/null; then
      DISPLAY="${DISPLAY}" xhost +local:docker 2>/dev/null \
        && echo "[rviz.sh] xhost +local:docker OK" \
        || echo "[rviz.sh] WARNING: xhost failed — RViz may be rejected by X server"
    else
      echo "[rviz.sh] WARNING: xhost not installed; X11 access control not updated"
    fi
  fi

  cd "${REPO_ROOT}"
  exec docker compose --profile rviz up rviz
fi

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
