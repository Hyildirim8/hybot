#!/bin/bash
# reset_map.sh — SLAM haritasını sıfırla ve navigation container'ı yeniden başlat.
#
# Kullanım:
#   ./scripts/reset_map.sh          # Haritayı sil + navigation yeniden başlat
#   ./scripts/reset_map.sh --keep   # Mevcut haritayı koru, sadece yeniden başlat

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$(dirname "$SCRIPT_DIR")/docker-compose.yaml"

keep_map=false
if [[ "${1:-}" == "--keep" ]]; then
    keep_map=true
fi

echo "[reset_map] Navigation container durduruluyor..."
COMPOSE_PROFILES=nav docker compose -f "$COMPOSE_FILE" stop navigation

if ! "$keep_map"; then
    echo "[reset_map] Navigation container siliniyor (SLAM durumu temizlenir)..."
    COMPOSE_PROFILES=nav docker compose -f "$COMPOSE_FILE" rm -f navigation
fi

echo "[reset_map] Navigation container yeniden başlatılıyor..."
COMPOSE_PROFILES=nav docker compose -f "$COMPOSE_FILE" up -d navigation

echo "[reset_map] Tamamlandı. Nav2 bringup ~60s sürer."
echo "[reset_map] Durum: docker logs ecza-robotu-navigation-1 -f"
