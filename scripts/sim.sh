#!/bin/bash
# scripts/sim.sh — Launch entry point for the Isaac Sim simulation stack.
#
# This is the SIM counterpart to scripts/launch.sh. It runs the standalone
# docker-compose.sim.yaml stack, which:
#   • has NO micro-ros-agent  (no ESP32 — velocities come from Isaac Sim)
#   • has NO real RPLidar     (lidar service only restamps Isaac Sim's /scan_sim)
#   • robot_state_publisher from rover.urdf (no ros2_control hardware)
#
# Before starting the sim stack it tears DOWN the real-robot stack
# (docker-compose.yaml) so the two can never run at once — that double-publish
# is exactly what makes RViz show the real lidar while "in sim".
#
# Usage:
#   bash scripts/sim.sh                       # joy + teleop + description + lidar bridge
#   bash scripts/sim.sh --nav                 # + Nav2 + SLAM (build a map)
#   bash scripts/sim.sh --nav --rviz          # + RViz2 (recommended for mapping)
#   bash scripts/sim.sh --nav --rviz --map /maps/warehouse.yaml   # navigate saved map
#
# Prerequisites:
#   • Isaac Sim running on ISAACSIM_IP (default 10.42.101.217)
#   • Isaac Sim OmniGraph publishes /scan_sim and /odom, subscribes /cmd_vel
#   • Same ROS_DOMAIN_ID on both machines (default 0)
set -euo pipefail

cd "$(dirname "$0")/.."

REAL_COMPOSE="-f docker-compose.yaml"
SIM_COMPOSE="-f docker-compose.sim.yaml"

# ─────────────────────────────────────────────────────────────────────────────
# Parse flags: --nav, --rviz, --map <path>
# ─────────────────────────────────────────────────────────────────────────────
NAV=false
RVIZ=false
MAP_FILE=""
REMAINING_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --nav)  NAV=true;  shift ;;
        --rviz) RVIZ=true; shift ;;
        --map)  MAP_FILE="$2"; shift 2 ;;
        *)      REMAINING_ARGS+=("$1"); shift ;;
    esac
done

# Build COMPOSE_PROFILES (sim compose only defines: nav, rviz)
PROFILES=""
if [[ "$NAV" == "true" ]]; then
    PROFILES="nav"
fi
if [[ "$RVIZ" == "true" ]]; then
    PROFILES="${PROFILES:+$PROFILES,}rviz"
fi
export COMPOSE_PROFILES="$PROFILES"

# Nav mode: SLAM (build map) by default; saved map if --map given
if [[ -n "$MAP_FILE" ]]; then
    export MAP="$MAP_FILE"
    export NAV_MODE="nav"
    echo "Nav mode: localize on saved map ($MAP_FILE)"
else
    export NAV_MODE="slam"
    [[ "$NAV" == "true" ]] && echo "Nav mode: SLAM (building a new map)"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Stop the real-robot stack so it can't double-publish /scan, /tf, /odom.
# ─────────────────────────────────────────────────────────────────────────────
echo "Stopping real-robot stack (docker-compose.yaml) if running..."
docker compose $REAL_COMPOSE down --remove-orphans 2>/dev/null || true

# ─────────────────────────────────────────────────────────────────────────────
# Launch the sim stack. --force-recreate so bind-mounted config + env take effect.
# ─────────────────────────────────────────────────────────────────────────────
echo "Starting Isaac Sim stack (docker-compose.sim.yaml)..."
echo "  Profiles : ${COMPOSE_PROFILES:-<base: joy,teleop,description,lidar>}"
echo "  Isaac Sim: ${ISAACSIM_IP:-10.42.101.217}  (set ISAACSIM_IP to change)"
exec docker compose $SIM_COMPOSE up --force-recreate "${REMAINING_ARGS[@]+"${REMAINING_ARGS[@]}"}"
