#!/bin/bash
# reset_map.sh — SLAM haritasını sıfırla.
#
# Kullanım (herhangi bir dizinden çalışır):
#   /home/master/Workspace/ecza-robotu/scripts/reset_map.sh
#   /home/master/Workspace/ecza-robotu/scripts/reset_map.sh --keep
#
# --keep: container'ı yeniden başlat ama mevcut haritayı silme

set -euo pipefail

REPO="/home/master/Workspace/ecza-robotu"

keep_map=false
[[ "${1:-}" == "--keep" ]] && keep_map=true

cd "$REPO"

echo "[reset_map] Harita sıfırlanıyor..."

docker compose stop navigation 2>&1 | grep -v "^$" || true

if ! "$keep_map"; then
    docker compose rm -f navigation 2>&1 | grep -v "^$" || true
    echo "[reset_map] Eski SLAM verisi temizlendi."
fi

COMPOSE_PROFILES=nav docker compose up -d navigation

echo ""
echo "[reset_map] Navigation yeniden başlatıldı."
echo "[reset_map] Nav2 bringup ~60 saniye sürer."
echo "[reset_map] Durum izlemek için:"
echo "            docker logs -f ecza-robotu-navigation-1"
