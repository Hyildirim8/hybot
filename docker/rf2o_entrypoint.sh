#!/usr/bin/env bash
# RF2O and its covariance adapter share a lifetime; neither runs orphaned.
set -euo pipefail
rf2o_pid=""
adapter_pid=""
cleanup() {
    [ -z "$rf2o_pid" ] || kill "$rf2o_pid" 2>/dev/null || true
    [ -z "$adapter_pid" ] || kill "$adapter_pid" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
python3 /laser_odom_covariance.py &
adapter_pid=$!
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
    -p laser_scan_topic:=/scan -p odom_topic:=/odom_rf2o_raw \
    -p base_frame_id:=base_link -p odom_frame_id:=odom \
    -p publish_tf:=false -p freq:=10.0 \
    -p init_pose_from_topic:=/odometry/filtered --log-level error &
rf2o_pid=$!
# On a process-only restart, initialize at the current EKF pose rather than
# jumping back to the origin. On cold start EKF first runs from wheels + IMU.
wait -n "$rf2o_pid" "$adapter_pid"
# An unexpected child exit must restart the whole service.
exit 1
