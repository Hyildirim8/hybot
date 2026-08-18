#!/usr/bin/env bash
# scripts/reset_map_pc.sh — Reset the robot's SLAM map remotely, from YOUR PC.
#
# slam_toolbox (mapping mode) has no "wipe map and start over" service —
# the reliable way to get a fresh, blank map is to restart the container
# that runs it. On this repo that's the `navigation` service: it embeds
# BOTH slam_toolbox and Nav2 together (see
# src/ecza_navigation/launch/nav2_navigation.launch.py), so restarting it
# also clears the Nav2 costmaps, which you want anyway right after a map
# reset (stale obstacles from the old map would otherwise linger).
#
# Run this ON YOUR PC (not the Pi) — requires SSH access to the robot.
#
# Usage:
#   ./scripts/reset_map_pc.sh [pi-user@pi-host] [repo-path-on-pi]
#
# Example:
#   ./scripts/reset_map_pc.sh master@10.42.101.197
#   ./scripts/reset_map_pc.sh master@10.42.101.197 ~/Workspace/ecza-robotu
set -euo pipefail

PI_HOST="${1:-master@10.42.101.197}"
REPO_PATH="${2:-~/Workspace/ecza-robotu}"

echo "[reset_map] ${PI_HOST} -> ${REPO_PATH}: slam_toolbox + Nav2 (navigation container) yeniden başlatılıyor..."
ssh "${PI_HOST}" "cd ${REPO_PATH} && docker compose restart navigation"

echo "[reset_map] tamam — RViz'de birkaç saniye içinde boş/yeni bir harita görmelisin."
echo "[reset_map] not: sadece Nav2 costmap'lerini temizlemek istiyorsan (SLAM haritasına dokunmadan),"
echo "             ssh ${PI_HOST} ile bağlanıp şu servisleri çağırabilirsin:"
echo "             ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}"
echo "             ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}"
