#!/usr/bin/env bash
# scripts/mac_start.sh — Drive the robot from a Mac. Remote control only.
#
# Clone this repo on the Mac, run this script, and you get:
#   1. the robot's Docker stack started on the Pi (over SSH)
#   2. the joystick + camera teleop client running on the Mac
#
# No visualiser. RViz stays on the Pi (scripts/launch.sh --rviz, viewed over
# VNC at <pi-ip>:5901) or on a Linux PC (scripts/rviz_viewer_pc.sh); neither is
# started or needed here.
#
# ── What runs where ──────────────────────────────────────────────────────────
# Nothing containerised runs on the Mac. Every service in this repo needs the
# Pi's hardware (RPLidar on USB, ESP32 over micro-ROS, CSI camera, I2C IMU), so
# the stack lives on the Pi and this script only starts it remotely.
#
# The Mac side is plain Python and needs no Docker and no ROS: the joystick
# link is newline-delimited JSON over TCP 9092, and the camera is chunked JPEG
# over UDP 8082. Neither touches DDS, so this works regardless of which
# Fast-DDS profile the robot is running — you do NOT need to launch the Pi
# with --lan (that flag only matters for the Linux PC-side RViz viewer).
#
# Usage:
#   ./scripts/mac_start.sh                    # start the stack, then drive
#   ./scripts/mac_start.sh --no-stack         # Pi already running: just drive
#   ./scripts/mac_start.sh --no-cam           # joystick only, no video window
#   ./scripts/mac_start.sh --calibrate        # map the gamepad's axes/buttons, then exit
#   ./scripts/mac_start.sh --check            # diagnose connectivity, then exit
#   ./scripts/mac_start.sh --down             # stop the stack on the Pi
#
# Env:
#   ROBOT_IP    robot address          (default 10.42.101.197)
#   ROBOT_USER  ssh user on the Pi     (default master)
#   ROBOT_REPO  repo path on the Pi    (default ~/Workspace/ecza-robotu)
#   LAUNCH_ARGS launch.sh flags        (default empty = base stack; use "--nav"
#                                       if you also want Nav2/SLAM running)
#   JOY_PORT    remote_teleop TCP port (default 9092)
#
# Requirements on the Mac: python3 (Xcode CLT or Homebrew) and a joystick.
# pygame/opencv/numpy are installed into a local virtualenv on first run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

ROBOT_IP="${ROBOT_IP:-10.42.101.197}"
ROBOT_USER="${ROBOT_USER:-master}"
ROBOT_REPO="${ROBOT_REPO:-~/Workspace/ecza-robotu}"
LAUNCH_ARGS="${LAUNCH_ARGS:-}"
JOY_PORT="${JOY_PORT:-9092}"

# Virtualenv rather than `pip install --user`: Homebrew and recent python.org
# builds mark the system interpreter as externally-managed (PEP 668), so a
# bare `pip install pygame` aborts with an error that reads like a pip bug.
VENV="${REPO_ROOT}/.venv-mac"

WANT_STACK=true
WANT_CAM=true
CHECK_ONLY=false
DO_DOWN=false
CALIBRATE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-stack)  WANT_STACK=false; shift ;;
    --no-cam)    WANT_CAM=false;   shift ;;
    --check)     CHECK_ONLY=true;  shift ;;
    --down)      DO_DOWN=true;     shift ;;
    --calibrate) CALIBRATE=true;   shift ;;
    -h|--help)   sed -n '2,40p' "$0" | sed 's/^#\{1,\} \{0,1\}//'; exit 0 ;;
    *) echo "[mac] unknown flag: $1 (try --help)" >&2; exit 2 ;;
  esac
done

log() { echo "[mac] $*"; }
err() { echo "[mac] ERROR: $*" >&2; }

if [[ "$(uname -s)" != "Darwin" ]]; then
  err "this script is for macOS. On Linux use scripts/remote_teleop_pc.sh"
  exit 1
fi

# ── SSH helpers ──────────────────────────────────────────────────────────────
# BatchMode so a missing key fails fast instead of blocking on a password
# prompt in the middle of an otherwise unattended script.
ssh_ok() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new \
      "${ROBOT_USER}@${ROBOT_IP}" true 2>/dev/null
}
ssh_run() {
  ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new \
      "${ROBOT_USER}@${ROBOT_IP}" "$@"
}

port_open() {
  # /dev/tcp is a bash builtin, so this needs no nc/netcat on the Mac.
  (exec 3<>"/dev/tcp/${ROBOT_IP}/$1") >/dev/null 2>&1
}

# ── --down ───────────────────────────────────────────────────────────────────
if [[ "$DO_DOWN" == "true" ]]; then
  log "stopping the stack on ${ROBOT_USER}@${ROBOT_IP}..."
  ssh_run "cd ${ROBOT_REPO} && docker compose --profile '*' down"
  log "stopped."
  exit 0
fi

# ── Reachability ─────────────────────────────────────────────────────────────
log "robot=${ROBOT_IP} user=${ROBOT_USER}"
# -W is milliseconds on macOS's ping (it is seconds on Linux's) — 2000 = 2s.
if ping -c 1 -W 2000 "$ROBOT_IP" >/dev/null 2>&1; then
  log "robot is reachable"
else
  err "no ping reply from ${ROBOT_IP} — wrong IP, or the Mac is on a different network"
  err "the robot's hotspot and your home Wi-Fi are different LANs; check which one the Mac joined"
  exit 1
fi

if [[ "$CHECK_ONLY" == "true" ]]; then
  ssh_ok && log "ssh: OK (key-based)" \
         || log "ssh: NOT passwordless — the script will prompt, or use --no-stack"
  port_open "$JOY_PORT" && log "remote_teleop :${JOY_PORT} is listening" \
                        || log "remote_teleop :${JOY_PORT} is NOT listening (stack not started?)"
  exit 0
fi

# ── 1. Start the stack on the Pi ─────────────────────────────────────────────
if [[ "$WANT_STACK" == "true" ]]; then
  if ! ssh_ok; then
    log "ssh is not passwordless — you will be prompted for ${ROBOT_USER}@${ROBOT_IP}'s password."
    log "to avoid this once and for all:  ssh-copy-id ${ROBOT_USER}@${ROBOT_IP}"
  fi
  log "starting stack on the Pi:  launch.sh ${LAUNCH_ARGS:-<base>} -d"
  # -d is forwarded by launch.sh straight to `docker compose up`, so the SSH
  # session returns instead of holding the stack in the foreground.
  ssh_run "cd ${ROBOT_REPO} && bash scripts/launch.sh ${LAUNCH_ARGS} -d"
  log "stack started"
else
  log "--no-stack: assuming the Pi is already running"
fi

# ── 2. Python environment on the Mac ─────────────────────────────────────────
# pygame/opencv/numpy all ship arm64 wheels, so this is a download, not a
# compile, on Apple Silicon.
if [[ ! -x "${VENV}/bin/python3" ]]; then
  log "creating virtualenv at ${VENV} (first run only)..."
  python3 -m venv "$VENV"
fi
if ! "${VENV}/bin/python3" -c 'import pygame, cv2, numpy' >/dev/null 2>&1; then
  log "installing pygame / opencv-python / numpy into the venv..."
  "${VENV}/bin/pip" install --quiet --upgrade pip
  "${VENV}/bin/pip" install --quiet pygame opencv-python numpy
fi

CLIENT="${SCRIPT_DIR}/remote_teleop_client.py"
[[ -f "$CLIENT" ]] || { err "$CLIENT not found — run this from a checkout of the repo"; exit 1; }

# ── 3. Calibration ───────────────────────────────────────────────────────────
if [[ "$CALIBRATE" == "true" ]]; then
  log "calibrating the gamepad — follow the prompts"
  # macOS SDL reports different axis signs and button numbers than joy_linux
  # does for the same pad, and remote_teleop_client.py's built-in correction
  # was verified against Linux SDL only. Calibrating once writes
  # scripts/remote_teleop_mapping.json, which later runs pick up automatically.
  exec "${VENV}/bin/python3" "$CLIENT" "$ROBOT_IP" --calibrate
fi

# ── 4. Drive ─────────────────────────────────────────────────────────────────
# Check the teleop port itself, not just ping: the Pi can be up and pingable
# while the remote_teleop container is down, and the client would then just sit
# there retrying with no obvious reason.
if ! port_open "$JOY_PORT"; then
  err "nothing listening on ${ROBOT_IP}:${JOY_PORT} — the remote_teleop container is not up"
  err "check on the Pi:  docker compose ps remote_teleop"
  exit 1
fi

if [[ ! -f "${SCRIPT_DIR}/remote_teleop_mapping.json" ]]; then
  log "NOTE: no gamepad calibration found. The built-in axis fix was verified on"
  log "      Linux SDL only; macOS may report different signs/button numbers."
  log "      If the rover moves the wrong way, stop and run: $0 --calibrate"
fi

ARGS=("$ROBOT_IP")
[[ "$WANT_CAM" == "true" ]] && ARGS+=(--cam)

log "starting teleop (ESC or close the window to quit)"
exec "${VENV}/bin/python3" "$CLIENT" "${ARGS[@]}"
