#!/bin/bash
# csi_cam_stream.sh — host-side MJPEG source for the RPi Camera Module 2
# on the CAM0/CSI port (imx219 sensor).
#
# Runs OUTSIDE Docker on purpose: the imx219 needs the PiSP ISP stack tied
# to this exact host's libcamera build (Raspberry Pi OS), which has no
# matching package in the ROS container's Ubuntu 22.04 Jammy apt repos.
# rpicam-vid's --listen mode serves exactly one TCP client and then exits
# when that client disconnects, so this wraps it in a respawn loop.
# src/ecza_camera/ecza_camera/rpicam_node.py (running inside the ROS
# container, reachable via network_mode: host) is that client.
#
# Managed by scripts/ecza-robotu-csi-cam.service.

set -u

PORT="${CSI_CAM_PORT:-8090}"
WIDTH="${CSI_CAM_WIDTH:-1280}"
HEIGHT="${CSI_CAM_HEIGHT:-720}"
FRAMERATE="${CSI_CAM_FRAMERATE:-15}"
QUALITY="${CSI_CAM_QUALITY:-80}"

echo "[csi_cam_stream] serving camera 0 (imx219) MJPEG on tcp://0.0.0.0:${PORT}"

while true; do
    rpicam-vid \
        --camera 0 \
        --codec mjpeg \
        --width "$WIDTH" \
        --height "$HEIGHT" \
        --framerate "$FRAMERATE" \
        --quality "$QUALITY" \
        --nopreview \
        --listen \
        -o "tcp://0.0.0.0:${PORT}" \
        -t 0
    echo "[csi_cam_stream] rpicam-vid exited (client disconnected or camera error) — restarting in 1s"
    sleep 1
done
