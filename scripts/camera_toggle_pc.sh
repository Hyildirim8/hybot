#!/usr/bin/env bash
# scripts/camera_toggle_pc.sh — Pause/resume the robot's CSI camera stream
# remotely, from YOUR PC, over SSH.
#
# The camera stream (csi_camera container) sustains ~3.5 MB/s over the
# robot's Wi-Fi link. That's fine when you just want to look around, but it
# competes for airtime with the remote-joystick TCP link and the SLAM/Nav2
# DDS traffic — on a busy shared Wi-Fi (office AP, other clients), that
# contention is what shows up as "ping" / lag while actively driving.
# Pausing the camera during an active drive session frees that bandwidth;
# resume it afterwards to look around again.
#
# Run this ON YOUR PC (not the Pi) — requires SSH access to the robot.
#
# Usage:
#   ./scripts/camera_toggle_pc.sh stop  [pi-user@pi-host] [repo-path-on-pi]
#   ./scripts/camera_toggle_pc.sh start [pi-user@pi-host] [repo-path-on-pi]
#
# Example:
#   ./scripts/camera_toggle_pc.sh stop  master@10.42.101.197
#   ./scripts/camera_toggle_pc.sh start master@10.42.101.197
set -euo pipefail

ACTION="${1:-}"
PI_HOST="${2:-master@10.42.101.197}"
REPO_PATH="${3:-~/Workspace/ecza-robotu}"

case "$ACTION" in
  stop)
    echo "[camera_toggle] ${PI_HOST}: kamera durduruluyor (Wi-Fi trafiği serbest bırakılıyor)..."
    ssh "${PI_HOST}" "cd ${REPO_PATH} && docker compose stop csi_camera"
    ;;
  start)
    echo "[camera_toggle] ${PI_HOST}: kamera yeniden başlatılıyor..."
    ssh "${PI_HOST}" "cd ${REPO_PATH} && docker compose start csi_camera"
    ;;
  *)
    echo "Usage: $0 {start|stop} [pi-user@pi-host] [repo-path-on-pi]" >&2
    exit 1
    ;;
esac
