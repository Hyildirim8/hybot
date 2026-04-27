#!/bin/bash
# nav2_entrypoint.sh
#
# Starts Nav2 with lifecycle autostart disabled, waits until the managed
# lifecycle services are visible on the ROS graph, then explicitly sends the
# STARTUP command. This avoids the startup race where lifecycle_manager tries
# to create service clients before DDS discovery has settled after a restart.

set -euo pipefail

MODE="${NAV_MODE:-slam}"
MAP_PATH="${MAP:-}"

launch_pid=""

cleanup() {
    if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -TERM "${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
}

trap cleanup INT TERM

wait_for_service() {
    local service_name="$1"
    local timeout_s="${2:-90}"
    local waited=0

    while [ "${waited}" -lt "${timeout_s}" ]; do
        if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
            wait "${launch_pid}"
        fi

        if ros2 service list 2>/dev/null | grep -Fxq "${service_name}"; then
            echo "[navigation] discovered ${service_name}"
            return 0
        fi

        sleep 1
        waited=$((waited + 1))
    done

    echo "[navigation] timeout waiting for ${service_name}" >&2
    return 1
}

startup_manager() {
    local manager_service="$1"
    local attempts=0

    wait_for_service "${manager_service}" 90

    while [ "${attempts}" -lt 10 ]; do
        if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
            wait "${launch_pid}"
        fi

        if ros2 service call "${manager_service}" nav2_msgs/srv/ManageLifecycleNodes \
            "{command: 0}" >/tmp/nav2_manage_nodes.out 2>/tmp/nav2_manage_nodes.err; then
            cat /tmp/nav2_manage_nodes.out
            return 0
        fi

        attempts=$((attempts + 1))
        sleep 1
    done

    cat /tmp/nav2_manage_nodes.err >&2 || true
    echo "[navigation] failed to startup lifecycle manager ${manager_service}" >&2
    return 1
}

wait_for_nav_services() {
    wait_for_service "/controller_server/get_state" 90
    wait_for_service "/smoother_server/get_state" 90
    wait_for_service "/planner_server/get_state" 90
    wait_for_service "/behavior_server/get_state" 90
    wait_for_service "/bt_navigator/get_state" 90
    wait_for_service "/waypoint_follower/get_state" 90
    wait_for_service "/velocity_smoother/get_state" 90
}

wait_for_localization_services() {
    wait_for_service "/map_server/get_state" 90
    wait_for_service "/amcl/get_state" 90
}

# Give the rest of the stack time to publish odom/scan and settle discovery.
sleep 8

launch_cmd=(
    ros2 launch ecza_navigation nav2_navigation.launch.py
    "mode:=${MODE}"
    "autostart:=False"
)

if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
    launch_cmd+=("map:=${MAP_PATH}")
fi

echo "[navigation] launching Nav2 in ${MODE} mode with manual lifecycle startup"
"${launch_cmd[@]}" &
launch_pid=$!

if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
    wait_for_localization_services
    startup_manager "/lifecycle_manager_localization/manage_nodes"
fi

wait_for_nav_services
startup_manager "/lifecycle_manager_navigation/manage_nodes"

wait "${launch_pid}"
