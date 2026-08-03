#!/usr/bin/env bash
# scripts/rviz_viewer_pc.sh — Launch the RViz2 viewer container on YOUR PC
# (NOT the Pi). Renders with your own GPU over X11 — no Xvfb, no x11vnc,
# takes zero load off the robot's Raspberry Pi.
#
# Run this ON YOUR UBUNTU PC, from a checkout of this repo, on the same LAN
# as the robot. See docker-compose.viewer.yaml for what this wraps and why.
#
# Usage:
#   ./scripts/rviz_viewer_pc.sh              # start (builds image on first run)
#   ./scripts/rviz_viewer_pc.sh --build       # force image rebuild
#   ./scripts/rviz_viewer_pc.sh --down        # stop and remove the viewer container
#
# Requirements: Docker + docker compose, an X server (native Ubuntu desktop
# or WSLg), same ROS_DOMAIN_ID as the robot (default 0, matches .env).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${REPO_ROOT}/docker-compose.viewer.yaml"

cd "$REPO_ROOT"

for arg in "$@"; do
  case "$arg" in
    --down)
      echo "[rviz_viewer_pc] stopping viewer..."
      exec docker compose -f "$COMPOSE_FILE" down
      ;;
  esac
done

BUILD_FLAG=""
for arg in "$@"; do
  [[ "$arg" == "--build" ]] && BUILD_FLAG="--build"
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "[rviz_viewer_pc] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

if command -v xhost &>/dev/null; then
  xhost +local:docker >/dev/null 2>&1 \
    && echo "[rviz_viewer_pc] xhost +local:docker OK" \
    || echo "[rviz_viewer_pc] WARNING: xhost failed — rviz window may be rejected by the X server"
else
  echo "[rviz_viewer_pc] WARNING: xhost not installed — install x11-xserver-utils if the rviz window doesn't appear"
fi

echo "[rviz_viewer_pc] starting viewer (docker compose -f docker-compose.viewer.yaml up ${BUILD_FLAG})..."
# shellcheck disable=SC2086
exec docker compose -f "$COMPOSE_FILE" up ${BUILD_FLAG}
