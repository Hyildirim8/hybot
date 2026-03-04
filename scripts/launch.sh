#!/bin/bash
# scripts/launch.sh — Canonical launch entry point for ecza-robotu rover stack
#
# Usage:
#   bash scripts/launch.sh [docker compose up options]
#   bash scripts/launch.sh --dev [options]        # dev mode: source mount + agent debug
#   bash scripts/launch.sh --lidar                # base stack + RPLidar driver
#   bash scripts/launch.sh --nav                  # base stack + lidar + Nav2/SLAM
#   bash scripts/launch.sh --nav --map /maps/x.yaml  # nav mode with saved map
#   RECORD=true bash scripts/launch.sh            # also start rosbag2 recorder
#
# What this script does:
#   1. Validates Docker Engine >= 24.0 and Docker Compose plugin >= 2.20
#   2. Translates RECORD=true → COMPOSE_PROFILES includes 'record'
#   3. --lidar adds 'lidar' profile; --nav adds 'nav' profile (includes lidar)
#   4. --dev merges docker-compose.dev.yaml
#   5. By default uses ONLY docker-compose.yaml (restart:unless-stopped)
#
# See specs/006-docker-runtime/quickstart.md for the full operator guide.
set -euo pipefail

# ─────────────────────────────────────────────────────────────────────────────
# Version requirements
# ─────────────────────────────────────────────────────────────────────────────
REQUIRED_DOCKER_MAJOR=24
REQUIRED_COMPOSE_MAJOR=2
REQUIRED_COMPOSE_MINOR=20

# ─────────────────────────────────────────────────────────────────────────────
check_docker_version() {
    local raw_version
    raw_version=$(docker --version 2>/dev/null) || {
        echo "ERROR: 'docker' not found. Install Docker Engine >= ${REQUIRED_DOCKER_MAJOR}.0." >&2
        echo "       See https://docs.docker.com/engine/install/" >&2
        exit 1
    }

    # Extract major version: "Docker version 24.0.7, build afdd53b" → 24
    local major
    major=$(echo "$raw_version" | grep -oP '(?<=version )\d+')

    if [[ "$major" -lt "$REQUIRED_DOCKER_MAJOR" ]]; then
        echo "ERROR: Docker Engine ${REQUIRED_DOCKER_MAJOR}.0 or later is required." >&2
        echo "       Found: ${raw_version}" >&2
        echo "       See https://docs.docker.com/engine/install/" >&2
        exit 1
    fi
}

check_compose_version() {
    local raw_version
    raw_version=$(docker compose version 2>/dev/null) || {
        echo "ERROR: 'docker compose' plugin not found. Install Compose >= v${REQUIRED_COMPOSE_MAJOR}.${REQUIRED_COMPOSE_MINOR}." >&2
        echo "       See https://docs.docker.com/compose/install/" >&2
        exit 1
    }

    # Extract version: "Docker Compose version v2.20.3" → major=2 minor=20
    local major minor
    major=$(echo "$raw_version" | grep -oP '(?<=v)\d+(?=\.)')
    minor=$(echo "$raw_version" | grep -oP '(?<=v\d\.)\d+')

    if [[ "$major" -lt "$REQUIRED_COMPOSE_MAJOR" ]] || \
       [[ "$major" -eq "$REQUIRED_COMPOSE_MAJOR" && "$minor" -lt "$REQUIRED_COMPOSE_MINOR" ]]; then
        echo "ERROR: Docker Compose v${REQUIRED_COMPOSE_MAJOR}.${REQUIRED_COMPOSE_MINOR} or later is required." >&2
        echo "       Found: ${raw_version}" >&2
        echo "       See https://docs.docker.com/compose/install/" >&2
        exit 1
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Validate versions
# ─────────────────────────────────────────────────────────────────────────────
echo "Checking prerequisites..."
check_docker_version
check_compose_version
echo "Docker and Compose versions OK."

# ─────────────────────────────────────────────────────────────────────────────
# RECORD=true → COMPOSE_PROFILES=record translation
#
# Compose activates service profiles via the COMPOSE_PROFILES env var, not via
# an arbitrary RECORD env var. This translation lets operators use the more
# intuitive RECORD=true interface documented in the quickstart.
# ─────────────────────────────────────────────────────────────────────────────
RECORD="${RECORD:-false}"

# ─────────────────────────────────────────────────────────────────────────────
# Parse flags: --dev, --lidar, --nav, --map <path>
# ─────────────────────────────────────────────────────────────────────────────
DEV_MODE=false
LIDAR_MODE=false
NAV_MODE=false
MAP_FILE=""
REMAINING_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dev)   DEV_MODE=true;  shift ;;
        --lidar) LIDAR_MODE=true; shift ;;
        --nav)   NAV_MODE=true;  shift ;;
        --map)   MAP_FILE="$2";  shift 2 ;;
        *)       REMAINING_ARGS+=("$1"); shift ;;
    esac
done

# --nav implies --lidar (nav profile includes the lidar service)
if [[ "$NAV_MODE" == "true" ]]; then
    LIDAR_MODE=true
fi

# Build COMPOSE_PROFILES
PROFILES=""
if [[ "$RECORD" == "true" ]]; then
    PROFILES="record"
fi
if [[ "$NAV_MODE" == "true" ]]; then
    PROFILES="${PROFILES:+$PROFILES,}nav"
elif [[ "$LIDAR_MODE" == "true" ]]; then
    PROFILES="${PROFILES:+$PROFILES,}lidar"
fi

if [[ -n "$PROFILES" ]]; then
    export COMPOSE_PROFILES="$PROFILES"
    echo "Profiles: COMPOSE_PROFILES=$COMPOSE_PROFILES"
else
    unset COMPOSE_PROFILES
fi

# Export map path for navigation container
if [[ -n "$MAP_FILE" ]]; then
    export MAP="$MAP_FILE"
    export NAV_MODE_ENV="nav"
    echo "Nav mode: nav (map: $MAP_FILE)"
else
    export NAV_MODE_ENV="${NAV_MODE_ENV:-slam}"
fi

if [[ "$DEV_MODE" == "true" ]]; then
    COMPOSE_FILES="-f docker-compose.yaml -f docker-compose.dev.yaml"
    echo "DEV mode: merging docker-compose.dev.yaml (source mounts, debug agent, restart:no)"
else
    COMPOSE_FILES="-f docker-compose.yaml"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Launch the stack
# ─────────────────────────────────────────────────────────────────────────────
echo "Starting ecza-robotu rover stack..."
exec docker compose $COMPOSE_FILES up "${REMAINING_ARGS[@]+"${REMAINING_ARGS[@]}"}"
