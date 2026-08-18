#!/usr/bin/env bash
# scripts/rviz_viewer_pc.sh — Launch the RViz2 viewer container on YOUR PC
# (NOT the Pi). Renders with your own GPU over X11 — no Xvfb, no x11vnc,
# and it takes the whole RViz load off the robot's Raspberry Pi.
#
# Why bother: the Pi has no usable GPU for this, so the on-Pi RViz falls back
# to llvmpipe and renders every pixel on the CPU. Measured 2026-08-11 at
# 1080p it cost 120-142% CPU with a system load average of 10-15 on 4 cores,
# which starved the EKF ("Failed to meet update rate") and made RViz itself
# log TF extrapolation errors. Viewing from the PC costs the Pi nothing.
#
# ── IMPORTANT: start the robot in LAN mode ───────────────────────────────────
# By default the robot runs a Fast-DDS profile with
# ignoreParticipantFlags=FILTER_DIFFERENT_HOST, which ignores every DDS
# participant on another machine — including this viewer. Nothing will appear
# and no error will be printed. Launch the robot with --lan (or
# DDS_PROFILE=fastdds_lan.xml) so it accepts off-robot DDS:
#
#     on the Pi : bash scripts/launch.sh --nav --lan
#     on the PC : ./scripts/rviz_viewer_pc.sh
#
# The filter exists to keep a foreign robot on this LAN out of domain 0, so
# put the Pi back to the default profile when you are done. See
# config/fastdds_lan.xml for the trade-off.
#
# Usage (run this ON YOUR UBUNTU PC, from a checkout of this repo):
#   ./scripts/rviz_viewer_pc.sh                 # start (builds image on first run)
#   ./scripts/rviz_viewer_pc.sh --build         # force image rebuild
#   ./scripts/rviz_viewer_pc.sh --check         # only test if the robot is reachable
#   ./scripts/rviz_viewer_pc.sh --down          # stop and remove the viewer container
#
# Env:
#   ROBOT_IP        robot address used by --check (default 10.42.101.197)
#   ROS_DOMAIN_ID   must match the robot (default 0 — keep it 0, see .env)
#
# Requirements: Docker + docker compose, an X server (native Ubuntu desktop
# or WSLg), and a LAN that passes UDP multicast between the PC and the Pi.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${REPO_ROOT}/docker-compose.viewer.yaml"
ROBOT_IP="${ROBOT_IP:-10.42.101.197}"

cd "$REPO_ROOT"

BUILD_FLAG=""
CHECK_ONLY=false
for arg in "$@"; do
  case "$arg" in
    --down)
      echo "[rviz_viewer_pc] stopping viewer..."
      exec docker compose -f "$COMPOSE_FILE" down
      ;;
    --build) BUILD_FLAG="--build" ;;
    --check) CHECK_ONLY=true ;;
  esac
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "[rviz_viewer_pc] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  ROBOT_IP=${ROBOT_IP}"

# ── Reachability check ───────────────────────────────────────────────────────
# Only proves the PC can route to the Pi. It cannot prove DDS discovery works:
# ping succeeds even when the robot is running the isolated profile, and also
# when the LAN blocks multicast. Both of those show up as "RViz connects but
# stays empty", so the real verification is the topic list further down.
if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "[rviz_viewer_pc] robot reachable at ${ROBOT_IP}"
else
  echo "[rviz_viewer_pc] WARNING: no ping reply from ${ROBOT_IP} — wrong IP, or not on the same LAN"
fi

if [[ "$CHECK_ONLY" == "true" ]]; then
  echo "[rviz_viewer_pc] --check: verifying DDS discovery (this needs the robot in --lan mode)..."
  if docker compose -f "$COMPOSE_FILE" run --rm --entrypoint bash rviz_viewer -lc \
      'source /opt/ros/humble/setup.bash; timeout 12 ros2 topic list 2>/dev/null' \
      | grep -qx '/scan_slam'; then
    echo "[rviz_viewer_pc] OK — robot topics visible over DDS"
    exit 0
  fi
  echo "[rviz_viewer_pc] FAIL — robot topics not visible. Check, in this order:"
  echo "                 1. the Pi was launched with --lan (default profile ignores other hosts)"
  echo "                 2. ROS_DOMAIN_ID matches on both sides (should be 0)"
  echo "                 3. the Wi-Fi AP is not blocking UDP multicast (client isolation)"
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 \
    && echo "[rviz_viewer_pc] xhost +local:docker OK" \
    || echo "[rviz_viewer_pc] WARNING: xhost failed — the rviz window may be rejected by the X server"
else
  echo "[rviz_viewer_pc] WARNING: xhost not installed — install x11-xserver-utils if the rviz window doesn't appear"
fi

echo "[rviz_viewer_pc] starting viewer..."
echo "[rviz_viewer_pc] (empty RViz? run: $0 --check)"
# shellcheck disable=SC2086
exec docker compose -f "$COMPOSE_FILE" up ${BUILD_FLAG}
