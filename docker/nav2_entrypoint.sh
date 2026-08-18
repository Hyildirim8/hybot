#!/bin/bash
# nav2_entrypoint.sh
#
# Starts Nav2 with lifecycle autostart disabled, then manually transitions
# each node through configure→activate using direct lifecycle CLI calls.
#
# Key improvement over lifecycle_manager's default approach:
#   • Waits for each node to reach the 'unconfigured' state (via ros2 lifecycle
#     get) before calling configure — this is more reliable than checking the
#     node list because the lifecycle service endpoint isn't registered until
#     the node has fully initialized and loaded its plugins.
#   • Kills the ENTIRE launch process group on failure (setsid ensures all
#     children die together), waits for DDS participants to expire, then
#     retries.

set -euo pipefail

MODE="${NAV_MODE:-slam}"
MAP_PATH="${MAP:-}"

launch_pid=""

cleanup() { kill_launch; }
trap cleanup INT TERM

kill_launch() {
    if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
        echo "[navigation] stopping nav2 launch (killing process group ${launch_pid})..."
        kill -TERM -"${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
        sleep 4
        kill -KILL -"${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
    launch_pid=""
}

# Wait for a lifecycle node to reach 'unconfigured' — the state it enters
# once internal init is complete and the lifecycle service is ready to accept
# a configure transition. More reliable than ros2 node list which only
# confirms DDS discovery, not that the plugin loading path is ready.
wait_for_lifecycle_ready() {
    local node="$1"
    local timeout_s="${2:-120}"
    local waited=0

    while [ "${waited}" -lt "${timeout_s}" ]; do
        if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
            echo "[navigation] launch process exited unexpectedly" >&2
            return 1
        fi

        state=$(timeout -s KILL 8 ros2 lifecycle get "/${node}" 2>/dev/null \
                | grep -o 'unconfigured\|inactive\|active\|finalized' | head -1 || echo "")

        if [ "${state}" = "unconfigured" ]; then
            echo "[navigation] /${node} ready (unconfigured)"
            return 0
        fi
        # Already transitioned (shouldn't happen on fresh launch, but handle it)
        if [ "${state}" = "inactive" ] || [ "${state}" = "active" ]; then
            echo "[navigation] /${node} already in ${state}"
            return 0
        fi

        sleep 3
        waited=$((waited + 3))
    done

    echo "[navigation] timeout: /${node} never reached unconfigured state" >&2
    return 1
}

wait_for_nav_services() {
    wait_for_service "/controller_server" 120 &&
    wait_for_service "/smoother_server"   120 &&
    wait_for_service "/planner_server"    120 &&
    wait_for_service "/behavior_server"   120 &&
    wait_for_service "/bt_navigator"      120 &&
    wait_for_service "/waypoint_follower" 120 &&
    wait_for_service "/velocity_smoother" 120
}

wait_for_localization_ready() {
    wait_for_lifecycle_ready "map_server" 60 &&
    wait_for_lifecycle_ready "amcl"       60
}

# Read a lifecycle node's current state label ("unconfigured", "inactive",
# "active", ...). Prints nothing and returns 1 if the query itself failed.
lifecycle_state() {
    local node="$1"
    local out
    out=$(timeout -s KILL 20 ros2 lifecycle get "/${node}" 2>/dev/null) || return 1
    # Output looks like "inactive [2]" — take the label only.
    out="${out%% *}"
    [ -n "${out}" ] || return 1
    echo "${out}"
}

# Transition a lifecycle node and VERIFY the result by reading its state back.
#
# Why not trust the CLI's stdout: `ros2 lifecycle set` first calls
# get_available_transitions, then change_state. On this robot both calls
# regularly lose their *response* under DDS load even though the server
# processed the request — observed on behavior_server during bringup:
#
#   [behavior_server.rclcpp]: failed to send response to
#   /behavior_server/get_available_transitions (timeout): client will not
#   receive response
#
# The old version treated that as a hard failure and tore the ENTIRE launch
# down (~2 min per retry), even though the node had usually transitioned
# fine. Reading the state back is the only trustworthy signal, and it also
# makes the call idempotent: a node already in the target state is a no-op,
# so re-running bringup is safe.
lifecycle_transition() {
    local node="$1"
    local transition_id="$2"
    local label="$3"
    local call_timeout="${4:-45}"

    local target
    case "${label}" in
        configure) target="inactive" ;;
        activate)  target="active"   ;;
        *)         target="" ;;
    esac

    local attempt
    for attempt in 1 2 3; do
        if [ -n "${target}" ] && [ "$(lifecycle_state "${node}" || true)" = "${target}" ]; then
            echo "[navigation] /${node} already ${target}"
            return 0
        fi

        echo "[navigation] ${label} /${node} (try ${attempt}/3)..."
        local result
        result=$(timeout -s KILL "${call_timeout}" ros2 lifecycle set "/${node}" "${label}" 2>&1) || true

        if echo "${result}" | grep -qi "Transitioning successful"; then
            echo "[navigation] /${node} ${label} OK"
            return 0
        fi

        # CLI reported nothing useful. The transition may still have landed —
        # give the node a moment to settle, then check the state itself.
        echo "${result}" >&2
        sleep 5
        if [ -n "${target}" ] && [ "$(lifecycle_state "${node}" || true)" = "${target}" ]; then
            echo "[navigation] /${node} ${label} OK (CLI response lost, state verified)"
            return 0
        fi

        echo "[navigation] /${node} ${label} did not take (try ${attempt}/3)" >&2
        sleep 5
    done

    echo "[navigation] /${node} ${label} FAILED after 3 tries" >&2
    return 1
}

# Order matters and must match Nav2's canonical bringup order: behavior_server
# has to be up before bt_navigator, because the behavior tree's recovery branch
# resolves the /backup, /spin and /wait action servers it owns at first tick.
#
# 2026-08-12: behavior_server and waypoint_follower were missing from both
# lists. autostart is False (see launch_cmd below), so nothing else ever sent
# them a transition — they sat in `unconfigured` indefinitely while every other
# server reported `active`. Nav2 therefore ran with NO recovery behaviors at
# all: any stuck robot failed its goal instantly instead of clearing costmaps
# and retrying. Verified live before the fix: `ros2 lifecycle get
# /behavior_server` -> unconfigured, /bond had 5 publishers instead of 7, and
# `ros2 action list` showed no /wait, /spin or /backup.
configure_nav_nodes() {
    lifecycle_transition "controller_server" 1 "configure" 120 &&
    lifecycle_transition "smoother_server"   1 "configure"  90 &&
    lifecycle_transition "planner_server"    1 "configure" 180 &&
    lifecycle_transition "behavior_server"   1 "configure" 90 &&
    lifecycle_transition "bt_navigator"      1 "configure" 90 &&
    lifecycle_transition "waypoint_follower" 1 "configure" 90 &&
    lifecycle_transition "velocity_smoother" 1 "configure" 90
}

activate_nav_nodes() {
    lifecycle_transition "controller_server" 3 "activate" 120 &&
    lifecycle_transition "smoother_server"   3 "activate"  90 &&
    lifecycle_transition "planner_server"    3 "activate" 180 &&
    lifecycle_transition "behavior_server"   3 "activate" 90 &&
    lifecycle_transition "bt_navigator"      3 "activate" 90 &&
    lifecycle_transition "waypoint_follower" 3 "activate" 90 &&
    lifecycle_transition "velocity_smoother" 3 "activate" 90
}

configure_localization_nodes() {
    lifecycle_transition "map_server" 1 "configure" 30 &&
    lifecycle_transition "amcl"       1 "configure" 30
}

activate_localization_nodes() {
    lifecycle_transition "map_server" 3 "activate" 30 &&
    lifecycle_transition "amcl"       3 "activate" 30
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

if [ "${MODE}" = "slam_only" ]; then
    echo "[navigation] launching SLAM-only mode — Nav2 lifecycle is skipped for clean mapping"
    setsid "${launch_cmd[@]}" &
    launch_pid=$!
    wait "${launch_pid}"
    exit $?
fi

attempt=0
while [ "${attempt}" -lt 5 ]; do
    echo "[navigation] launching Nav2 in ${MODE} mode (attempt $((attempt+1))/5)..."
    setsid "${launch_cmd[@]}" &
    launch_pid=$!

    if [ "${MODE}" = "nav" ] && [ -n "${MAP_PATH}" ]; then
        if ! wait_for_localization_ready; then
            echo "[navigation] localization nodes timed out" >&2
            kill_launch
            echo "[navigation] waiting 15s for DDS cleanup..."
            sleep 15
            attempt=$((attempt + 1))
            continue
        fi
        echo "[navigation] configuring localization nodes..."
        if configure_localization_nodes; then
            activate_localization_nodes || true
        fi
    fi

    if ! wait_for_nav_ready; then
        echo "[navigation] nav nodes timed out" >&2
        kill_launch
        echo "[navigation] waiting 15s for DDS cleanup..."
        sleep 15
        attempt=$((attempt + 1))
        continue
    fi

    if [ -n "${launch_pid}" ] && ! kill -0 "${launch_pid}" 2>/dev/null; then
        wait "${launch_pid}" 2>/dev/null || true
        launch_pid=""
        attempt=$((attempt + 1))
        continue
    fi

    bringup_ok=false

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
    echo "[navigation] waiting 15s for DDS cleanup..."
    sleep 15
    attempt=$((attempt + 1))
done

if [ -n "${launch_pid}" ]; then
    wait "${launch_pid}"
fi
