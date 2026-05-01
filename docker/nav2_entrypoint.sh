#!/bin/bash
# nav2_entrypoint.sh
#
# Starts Nav2 with lifecycle autostart disabled, then manually transitions
# each node through configure→activate using direct change_state service calls
# with generous timeouts. This bypasses the hardcoded 3-second timeout inside
# nav2_lifecycle_manager (which causes async_send_request failures on slow RPi
# hardware) and avoids the SIGSEGV that occurs when STARTUP is sent to
# lifecycle_manager after it has already run SHUTDOWN.
#
# On failure: kill the ENTIRE launch process group (setsid ensures all
# children — controller_server, planner_server, etc. — die together), wait
# for DDS participants to expire, then restart cleanly.

set -euo pipefail

MODE="${NAV_MODE:-slam}"
MAP_PATH="${MAP:-}"

launch_pid=""

cleanup() {
    kill_launch
}

trap cleanup INT TERM

# Verify a lifecycle get_state service is live by actually calling it.
# ros2 service list can return stale entries from a previous container.
wait_for_service() {
    local service_name="$1"
    local timeout_s="${2:-120}"
    local waited=0

    while [ "${waited}" -lt "${timeout_s}" ]; do
        if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
            wait "${launch_pid}" 2>/dev/null || true
            echo "[navigation] launch process exited unexpectedly" >&2
            return 1
        fi

        if timeout 5 ros2 service call "${service_name}" \
            lifecycle_msgs/srv/GetState '{}' >/dev/null 2>&1; then
            echo "[navigation] ${service_name} is live"
            return 0
        fi

        sleep 3
        waited=$((waited + 3))
    done

    echo "[navigation] timeout waiting for ${service_name}" >&2
    return 1
}

wait_for_nav_services() {
    wait_for_service "/controller_server/get_state" 120 &&
    wait_for_service "/smoother_server/get_state"   120 &&
    wait_for_service "/planner_server/get_state"    120 &&
    wait_for_service "/behavior_server/get_state"   120 &&
    wait_for_service "/bt_navigator/get_state"      120 &&
    wait_for_service "/waypoint_follower/get_state" 120 &&
    wait_for_service "/velocity_smoother/get_state" 120
}

wait_for_localization_services() {
    wait_for_service "/map_server/get_state" 120 &&
    wait_for_service "/amcl/get_state"       120
}

# Call change_state directly on a node — bypasses lifecycle_manager's 3 s timeout.
# transition IDs: configure=1, cleanup=2, activate=3, deactivate=4, shutdown=6
lifecycle_transition() {
    local node="$1"
    local transition_id="$2"
    local label="$3"
    local call_timeout="${4:-45}"

    echo "[navigation] ${label} /${node}..."
    local result
    result=$(timeout "${call_timeout}" ros2 service call \
        "/${node}/change_state" \
        lifecycle_msgs/srv/ChangeState \
        "{transition: {id: ${transition_id}}}" 2>&1) || true

    if echo "${result}" | grep -q "success=True"; then
        echo "[navigation] /${node} ${label} OK"
        return 0
    fi

    echo "[navigation] /${node} ${label} FAILED" >&2
    return 1
}

configure_nav_nodes() {
    lifecycle_transition "controller_server" 1 "configure" 60 &&
    lifecycle_transition "smoother_server"   1 "configure" 45 &&
    lifecycle_transition "planner_server"    1 "configure" 60 &&
    lifecycle_transition "behavior_server"   1 "configure" 45 &&
    lifecycle_transition "bt_navigator"      1 "configure" 45 &&
    lifecycle_transition "waypoint_follower" 1 "configure" 45 &&
    lifecycle_transition "velocity_smoother" 1 "configure" 30
}

activate_nav_nodes() {
    lifecycle_transition "controller_server" 3 "activate" 60 &&
    lifecycle_transition "smoother_server"   3 "activate" 45 &&
    lifecycle_transition "planner_server"    3 "activate" 60 &&
    lifecycle_transition "behavior_server"   3 "activate" 45 &&
    lifecycle_transition "bt_navigator"      3 "activate" 45 &&
    lifecycle_transition "waypoint_follower" 3 "activate" 60 &&
    lifecycle_transition "velocity_smoother" 3 "activate" 45
}

configure_localization_nodes() {
    lifecycle_transition "map_server" 1 "configure" 30 &&
    lifecycle_transition "amcl"       1 "configure" 30
}

activate_localization_nodes() {
    lifecycle_transition "map_server" 3 "activate" 30 &&
    lifecycle_transition "amcl"       3 "activate" 30
}

kill_launch() {
    if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
        echo "[navigation] stopping nav2 launch (killing process group ${launch_pid})..."
        # setsid makes launch the group leader — negative PID kills all children.
        kill -TERM -"${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
        sleep 4
        # Force-kill any survivors (processes that ignored SIGTERM)
        kill -KILL -"${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
    launch_pid=""
}

# Give the rest of the stack (hardware, lidar, robot_description) time to
# register their services before Nav2 launches.
sleep 8

launch_cmd=(
    ros2 launch ecza_navigation nav2_navigation.launch.py
    "mode:=${MODE}"
    "autostart:=False"
)

if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
    launch_cmd+=("map:=${MAP_PATH}")
fi

attempt=0
while [ "${attempt}" -lt 5 ]; do
    echo "[navigation] launching Nav2 in ${MODE} mode (attempt $((attempt+1))/5)..."
    # setsid: launch gets its own session → own process group → we can kill
    # the entire tree with kill -TERM -launch_pid.
    setsid "${launch_cmd[@]}" &
    launch_pid=$!

    if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
        if ! wait_for_localization_services; then
            echo "[navigation] localization services timed out" >&2
            kill_launch
            echo "[navigation] waiting 15s for DDS cleanup..."
            sleep 15
            attempt=$((attempt + 1))
            continue
        fi
    fi

    if ! wait_for_nav_services; then
        echo "[navigation] nav services timed out" >&2
        kill_launch
        echo "[navigation] waiting 15s for DDS cleanup..."
        sleep 15
        attempt=$((attempt + 1))
        continue
    fi

    # Give lifecycle_manager's DDS participant time to discover the same
    # endpoints the shell just confirmed before we call configure.
    echo "[navigation] DDS settle (10s)..."
    sleep 10

    if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
        wait "${launch_pid}" 2>/dev/null || true
        launch_pid=""
        attempt=$((attempt + 1))
        continue
    fi

    bringup_ok=false

    if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
        echo "[navigation] configuring localization nodes..."
        if configure_localization_nodes; then
            echo "[navigation] activating localization nodes..."
            activate_localization_nodes || true
        fi
    fi

    echo "[navigation] configuring nav2 nodes..."
    if configure_nav_nodes; then
        echo "[navigation] activating nav2 nodes..."
        if activate_nav_nodes; then
            bringup_ok=true
        fi
    fi

    if "${bringup_ok}"; then
        echo "[navigation] bringup complete — all nav2 nodes active"
        break
    fi

    echo "[navigation] bringup failed (attempt $((attempt+1))/5) — restarting launch..."
    kill_launch

    # Wait for process group to fully exit and DDS participants to expire.
    echo "[navigation] waiting 15s for DDS cleanup..."
    sleep 15

    attempt=$((attempt + 1))
done

wait "${launch_pid}"
