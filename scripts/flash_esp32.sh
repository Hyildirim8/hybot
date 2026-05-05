#!/bin/bash
# flash_esp32.sh — ESP32-S3 firmware flash (manuel bootloader modu)
#
# Çalıştır: bash scripts/flash_esp32.sh
#
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$REPO_DIR/firmware/build"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║   ESP32-S3 Firmware Flash — ecza-robotu     ║"
echo "╚══════════════════════════════════════════════╝"
echo ""
echo "Gereksinimler kontrol ediliyor..."
if ! python3 -m esptool version &>/dev/null; then
    echo "HATA: esptool bulunamadı. Kurmak için:"
    echo "  pip3 install esptool"
    exit 1
fi

if [ ! -f "$BUILD_DIR/rover_firmware.bin" ]; then
    echo "HATA: Derlenmiş firmware bulunamadı: $BUILD_DIR/rover_firmware.bin"
    echo "Önce 'docker run --rm -v $REPO_DIR/firmware:/project -w /project espressif/idf:v5.3 idf.py build' çalıştır."
    exit 1
fi

# micro-ros-agent durdur (portu serbest bırak)
echo ""
echo "micro-ros-agent durduruluyor..."
cd "$REPO_DIR" && docker compose stop micro-ros-agent 2>/dev/null || true
sleep 1

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "ŞIMDI YAPMAN GEREKEN:"
echo ""
echo "  1. ESP32 kartındaki  [BOOT]  butonunu basılı tut"
echo "  2. [RESET] (veya EN) butonuna bir kez bas ve bırak"
echo "  3. [BOOT] butonunu bırak"
echo "  4. Aşağıda Enter'a bas"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
read -p "BOOT+RESET yaptın, hazırım → [Enter]" _
echo ""
echo "Flash başlıyor..."
echo ""

PORT="${1:-/dev/ttyACM0}"
echo "Port: $PORT"

python3 -m esptool --chip esp32s3 -p "$PORT" -b 460800 \
    --before no-reset --after hard-reset \
    write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB \
    0x0     "$BUILD_DIR/bootloader/bootloader.bin" \
    0x8000  "$BUILD_DIR/partition_table/partition-table.bin" \
    0x10000 "$BUILD_DIR/rover_firmware.bin"

echo ""
echo "✓ Flash tamamlandı. micro-ros-agent yeniden başlatılıyor..."
cd "$REPO_DIR" && docker compose start micro-ros-agent
echo "✓ Agent başlatıldı. Sistem hazır."
