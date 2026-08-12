#!/usr/bin/env bash
# scripts/remote_teleop_pc.sh — Drive the robot from YOUR PC's joystick.
#
# Run this ON YOUR PC (not the Pi). It wraps scripts/remote_teleop_client.py:
# reads the PC's gamepad with pygame and streams it to the robot's
# remote_teleop service, which republishes it on /joy alongside the Pi's own
# joy_node. The robot's local joystick keeps working — this is additive.
#
# This path is plain TCP (port 9092, newline-delimited JSON), NOT DDS, so it
# works regardless of which Fast-DDS profile the robot is running. You do NOT
# need to launch the Pi with --lan for this (that is only for the PC-side
# RViz viewer, scripts/rviz_viewer_pc.sh).
#
# Usage:
#   ./scripts/remote_teleop_pc.sh                       # connect to $ROBOT_IP
#   ./scripts/remote_teleop_pc.sh 10.42.101.197         # explicit robot IP
#   ./scripts/remote_teleop_pc.sh --cam                 # also show the camera stream
#   ./scripts/remote_teleop_pc.sh --calibrate           # map the pad's axes/buttons
#   ./scripts/remote_teleop_pc.sh --check               # test reachability, then exit
#   ./scripts/remote_teleop_pc.sh -- --debug --raw      # pass flags straight through
#
# Env:
#   ROBOT_IP    robot address (default 10.42.101.197)
#   JOY_PORT    remote_teleop TCP port (default 9092)
#
# Requirements on the PC: python3, pygame (pip install pygame), a joystick.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT="${SCRIPT_DIR}/remote_teleop_client.py"

ROBOT_IP="${ROBOT_IP:-10.42.101.197}"
JOY_PORT="${JOY_PORT:-9092}"
CHECK_ONLY=false
WANT_CAM=false
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) CHECK_ONLY=true; shift ;;
    --cam)   WANT_CAM=true; shift ;;
    --)      shift; PASSTHROUGH+=("$@"); break ;;
    -*)      PASSTHROUGH+=("$1"); shift ;;
    *)       ROBOT_IP="$1"; shift ;;   # first bare argument is the robot IP
  esac
done

if [[ ! -f "$CLIENT" ]]; then
  echo "[remote_teleop_pc] ERROR: $CLIENT not found — run this from a checkout of the repo" >&2
  exit 1
fi

echo "[remote_teleop_pc] robot=${ROBOT_IP} joy-port=${JOY_PORT}"

# Reachability: check the teleop port itself, not just ping — the robot can be
# up and pingable while the remote_teleop container is down, and the client
# would then just sit there retrying with no obvious reason.
port_open() {
  if command -v nc >/dev/null 2>&1; then
    nc -z -w 3 "$ROBOT_IP" "$JOY_PORT" >/dev/null 2>&1
  else
    timeout 3 bash -c "echo > /dev/tcp/${ROBOT_IP}/${JOY_PORT}" >/dev/null 2>&1
  fi
}

if port_open; then
  echo "[remote_teleop_pc] remote_teleop is listening on ${ROBOT_IP}:${JOY_PORT}"
else
  echo "[remote_teleop_pc] WARNING: nothing listening on ${ROBOT_IP}:${JOY_PORT}"
  echo "                   the remote_teleop container may not be running — on the Pi:"
  echo "                     docker compose ps remote_teleop"
fi

if [[ "$CHECK_ONLY" == "true" ]]; then
  port_open && exit 0 || exit 1
fi

if ! python3 -c 'import pygame' >/dev/null 2>&1; then
  echo "[remote_teleop_pc] ERROR: pygame is not installed — pip install pygame" >&2
  exit 1
fi

ARGS=("$ROBOT_IP" --joy-port "$JOY_PORT")
if [[ "$WANT_CAM" == "true" ]]; then
  # remote_teleop_client.py defaults --cam-host to the robot IP; naming it
  # explicitly keeps the intent obvious in `ps` output.
  ARGS+=(--cam-host "$ROBOT_IP")
fi
if [[ ${#PASSTHROUGH[@]} -gt 0 ]]; then
  ARGS+=("${PASSTHROUGH[@]}")
fi

echo "[remote_teleop_pc] starting: remote_teleop_client.py ${ARGS[*]}"
exec python3 "$CLIENT" "${ARGS[@]}"
