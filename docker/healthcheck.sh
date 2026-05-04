#!/bin/bash
# docker/healthcheck.sh — Rover stack health check
#
# Implements the health check contract from contracts/compose-schema.md §3.
#
# Usage:
#   docker/healthcheck.sh --tier1    # container state check (exit 0 = all running)
#   docker/healthcheck.sh --tier2    # ROS2 graph check (exit 0 = all topics active)
#
# Exit codes:
#   0 — all required services/topics are healthy
#   1 — one or more required services/topics are missing or stopped
#   2 — usage error (invalid arguments)
set -euo pipefail

REQUIRED_SERVICES=(joy teleop robot_description micro-ros-agent diagnostics)
REQUIRED_TOPICS=(/joy /cmd_vel /wheel_velocities /diagnostics)

# ─────────────────────────────────────────────────────────────────────────────
tier1() {
    # Calls `docker compose ps --format json` and checks that every required
    # service is in "running" state.
    # Output contract (stdout, one line per service):
    #   joy            running
    #   teleop         running
    #   kinematics     running
    #   micro-ros-agent running
    #   diagnostics    running
    #   recorder       running|stopped
    local all_ok=0

    local ps_json
    ps_json=$(docker compose ps --format json 2>/dev/null) || {
        echo "ERROR: docker compose ps failed — is the stack running?" >&2
        exit 1
    }

    declare -A svc_state
    while IFS= read -r line; do
        local name state
        name=$(echo "$line" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('Service',''))" 2>/dev/null)
        state=$(echo "$line" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('State',''))" 2>/dev/null)
        [[ -n "$name" ]] && svc_state["$name"]="${state:-unknown}"
    done < <(echo "$ps_json" | python3 -c "
import sys, json
data = sys.stdin.read().strip()
if data.startswith('['):
    items = json.loads(data)
else:
    items = [json.loads(l) for l in data.splitlines() if l.strip()]
for item in items:
    print(json.dumps(item))
" 2>/dev/null)

    for svc in "${REQUIRED_SERVICES[@]}"; do
        local state="${svc_state[$svc]:-stopped}"
        printf "%-20s %s\n" "$svc" "$state"
        if [[ "$state" != "running" ]]; then
            all_ok=1
        fi
    done

    # recorder is optional — show its state but don't fail on stopped
    local rec_state="${svc_state[recorder]:-stopped}"
    printf "%-20s %s\n" "recorder" "$rec_state"

    return $all_ok
}

# ─────────────────────────────────────────────────────────────────────────────
tier2() {
    # Executes `ros2 topic list` inside the teleop container to verify
    # that all required topics are active in the ROS2 graph.
    # Also checks for the ESP32 firmware node (informational only).
    #
    # Output contract (stdout):
    #   topics:
    #     /joy                  active
    #     /cmd_vel              active
    #     /wheel_velocities     active
    #     /diagnostics          active
    #   esp32_node:             connected|waiting
    local all_ok=0

    local topic_list
    topic_list=$(docker compose exec -T teleop \
        bash -c "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null || true && \
                 ros2 topic list 2>/dev/null") || {
        echo "ERROR: could not exec into teleop container" >&2
        exit 1
    }

    echo "topics:"
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if echo "$topic_list" | grep -qx "$topic"; then
            printf "  %-24s active\n" "$topic"
        else
            printf "  %-24s MISSING\n" "$topic"
            all_ok=1
        fi
    done

    # ESP32 firmware node check — informational only (does not affect exit code)
    local node_list
    node_list=$(docker compose exec -T teleop \
        bash -c "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null || true && \
                 ros2 node list 2>/dev/null") || node_list=""

    if echo "$node_list" | grep -q "esp32_firmware_node"; then
        echo "esp32_node:             connected"
    else
        echo "esp32_node:             waiting"
    fi

    return $all_ok
}

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
case "${1:-}" in
    --tier1)
        tier1
        ;;
    --tier2)
        tier2
        ;;
    "")
        echo "Usage: $0 --tier1 | --tier2" >&2
        exit 2
        ;;
    *)
        echo "Unknown option: $1" >&2
        echo "Usage: $0 --tier1 | --tier2" >&2
        exit 2
        ;;
esac
