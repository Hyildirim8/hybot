#!/bin/bash
# launch_isaac_sim.sh — Set ROS2 env vars at SHELL LEVEL and launch Isaac Sim.
#
# WHY THIS EXISTS:
#   Python's os.environ changes don't affect LD_LIBRARY_PATH for C++ dlopen()
#   calls that happen BEFORE Python sets them. The ROS2 bridge extension's C++
#   plugin is loaded by the Kit extension manager, which reads LD_LIBRARY_PATH
#   from the shell environment — so env vars MUST be set here, not in Python.
#
# USAGE (on mucahit@10.42.101.217):
#   bash ~/launch_isaac_sim.sh
#
# To make permanent (never need to run this manually again):
#   echo 'source ~/launch_isaac_sim.sh' is NOT the way — instead run:
#   bash ~/launch_isaac_sim.sh  (this script execs Isaac Sim directly)

set -euo pipefail

ISAACSIM_ROOT="${HOME}/isaacsim"
PYTHON_SH="${ISAACSIM_ROOT}/python.sh"

# Path to this script — used to find isaac_sim_standalone.py alongside it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STANDALONE_PY="${SCRIPT_DIR}/isaac_sim_standalone.py"

# If script is not found at expected place, try home dir (if user copied it there)
if [[ ! -f "$STANDALONE_PY" ]]; then
    STANDALONE_PY="${HOME}/isaac_sim_standalone.py"
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  ecza-robotu Isaac Sim 4.5 launcher"
echo "═══════════════════════════════════════════════════════════════"

# ── 1. Find ROS2 libraries ───────────────────────────────────────────────────
ROS2_LIBS_OK=false

# Option A: system ROS2 Humble (preferred — full library set)
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "  Sourcing system ROS2 Humble..."
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    ROS2_LIBS_OK=true
    echo "  ✓ /opt/ros/humble sourced"
fi

# Option B: locate librosidl_runtime_c.so anywhere in Isaac Sim extscache
if [[ "$ROS2_LIBS_OK" == "false" ]]; then
    echo "  System ROS2 not found — searching Isaac Sim extscache..."
    FOUND_LIB=""
    # Common locations in Isaac Sim 4.x
    SEARCH_BASES=(
        "${ISAACSIM_ROOT}/extscache"
        "${ISAACSIM_ROOT}/exts"
        "${HOME}/.local/share/ov/pkg"
    )
    for base in "${SEARCH_BASES[@]}"; do
        [[ -d "$base" ]] || continue
        while IFS= read -r lib; do
            dir="$(dirname "$lib")"
            echo "  Found: $lib"
            export LD_LIBRARY_PATH="${dir}:${LD_LIBRARY_PATH:-}"
            ROS2_LIBS_OK=true
        done < <(find "$base" -maxdepth 8 -name "librosidl_runtime_c.so" 2>/dev/null | head -5)
        [[ "$ROS2_LIBS_OK" == "true" ]] && break
    done
fi

if [[ "$ROS2_LIBS_OK" == "false" ]]; then
    echo ""
    echo "  ⚠  WARNING: librosidl_runtime_c.so not found anywhere."
    echo "     ROS2 Bridge will fail. Install ROS2 Humble:"
    echo "     sudo apt install ros-humble-desktop"
    echo ""
fi

# ── 2. Mandatory env vars ────────────────────────────────────────────────────
BRIDGE_HUMBLE="${ISAACSIM_ROOT}/exts/isaacsim.ros2.bridge/humble"

# AMENT_PREFIX_PATH: prefer system ROS2 workspace(s), fall back to bridge's own
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    if [[ -d "$BRIDGE_HUMBLE" ]]; then
        export AMENT_PREFIX_PATH="$BRIDGE_HUMBLE"
    fi
fi

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Always include bridge's own lib dir as a fallback
if [[ -d "${BRIDGE_HUMBLE}/lib" ]]; then
    export LD_LIBRARY_PATH="${BRIDGE_HUMBLE}/lib:${LD_LIBRARY_PATH:-}"
fi

echo ""
echo "  AMENT_PREFIX_PATH : ${AMENT_PREFIX_PATH:-<not set>}"
echo "  ROS_DISTRO        : ${ROS_DISTRO}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID}"
echo "  LD_LIBRARY_PATH   : ${LD_LIBRARY_PATH:0:80}..."
echo ""
echo "  Script: ${STANDALONE_PY}"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# ── 3. Run Isaac Sim (exec replaces this shell — env is inherited) ───────────
exec "$PYTHON_SH" "$STANDALONE_PY" "$@"
