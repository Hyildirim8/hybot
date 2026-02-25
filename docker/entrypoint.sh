#!/bin/bash
# entrypoint.sh — Sources ROS2 environment overlays, then exec's the command.
#
# Called as ENTRYPOINT in the runtime Docker image.
# Ensures every container process inherits the full ROS2 + workspace setup.
set -e

# Clean stale FastDDS shared-memory segments from previous runs.
# With ipc:host all containers share /dev/shm; orphaned fastrtps_* files from
# dead containers prevent SHM transport from initialising on restart (T-SHM-01).
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

# Source base ROS2 Humble installation
# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash

# Source workspace install overlay (created by colcon --merge-install).
# Guard against empty src/ scaffold where no packages were built.
if [ -f /ws/install/setup.bash ]; then
    # shellcheck source=/ws/install/setup.bash
    source /ws/install/setup.bash
fi

# Hand off to the command specified by `command:` in docker-compose.yaml
exec "$@"
