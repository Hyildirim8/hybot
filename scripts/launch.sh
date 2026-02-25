#!/bin/bash
# scripts/launch.sh — Canonical launch entry point for ecza-robotu rover stack
#
# Usage:
#   bash scripts/launch.sh [docker compose up options]
#   RECORD=true bash scripts/launch.sh           # also start recorder service
#   RECORD=true bash scripts/launch.sh -d        # detached mode with recording
#
# What this script does:
#   1. Validates Docker Engine >= 24.0 and Docker Compose plugin >= 2.20
#   2. Translates RECORD=true into COMPOSE_PROFILES=record before invoking
#      docker compose, so operators can use the simpler RECORD=true syntax
#      documented in the quickstart rather than setting COMPOSE_PROFILES directly.
#   3. Passes all remaining arguments through to `docker compose up`.
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

if [[ "$RECORD" == "true" ]]; then
    export COMPOSE_PROFILES=record
    echo "Recording enabled: COMPOSE_PROFILES=record (recorder service will start)"
else
    # Ensure COMPOSE_PROFILES is not set to something unexpected from a prior
    # shell session that could accidentally activate the recorder.
    unset COMPOSE_PROFILES
fi

# ─────────────────────────────────────────────────────────────────────────────
# Launch the stack
# ─────────────────────────────────────────────────────────────────────────────
echo "Starting ecza-robotu rover stack..."
exec docker compose up "$@"
