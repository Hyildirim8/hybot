#!/usr/bin/env python3
"""
tools/provision_nvs.py — Write rover configuration to ESP32 NVS over USB.

Usage (run from firmware/ directory with ESP-IDF activated):
    python3 ../tools/provision_nvs.py \
        --port /dev/ttyUSB0 \
        --ssid  "MyNetwork" \
        --pass  "MyPassword" \
        --agent-ip   "192.168.1.10" \
        --agent-port 8888 \
        --max-speed  10.0 \
        --wdg-timeout 500

This script generates a temporary NVS CSV file, builds a binary partition
image using the ESP-IDF nvs_partition_gen.py utility, and flashes it to the
NVS partition of the connected ESP32.

Requirements:
    - ESP-IDF v5.x activated (IDF_PATH set)
    - esptool.py in PATH (bundled with ESP-IDF)
    - Device connected via USB

NVS namespace: rover_cfg  (matches firmware/main/nvs_config.h)
"""

import argparse
import csv
import os
import struct
import subprocess
import sys
import tempfile
import shutil

NVS_NAMESPACE = "rover_cfg"
NVS_PARTITION_SIZE = "0x6000"   # 24 kB — default NVS partition size
NVS_PARTITION_ADDR = "0x9000"   # default NVS partition address for ESP32-S3


def find_nvs_tool():
    idf_path = os.environ.get("IDF_PATH")
    if not idf_path:
        sys.exit("ERROR: IDF_PATH not set — activate ESP-IDF first.")
    tool = os.path.join(idf_path, "components", "nvs_flash", "nvs_partition_generator",
                        "nvs_partition_gen.py")
    if not os.path.isfile(tool):
        sys.exit(f"ERROR: nvs_partition_gen.py not found at {tool}")
    return tool


def build_csv(args, csv_path):
    rows = [
        # key, type, encoding, value
        ["key", "type", "encoding", "value"],
        [NVS_NAMESPACE, "namespace", "", ""],
        ["wifi_ssid",      "data", "string", args.ssid],
        ["wifi_pass",      "data", "string", args.pass_],
        ["agent_ip",       "data", "string", args.agent_ip],
        ["agent_port",     "data", "u16",    str(args.agent_port)],
        ["wdg_timeout_ms", "data", "u32",    str(args.wdg_timeout)],
    ]
    # max_speed: store as 4-byte IEEE-754 blob using hex2bin encoding
    bits = struct.pack("<f", args.max_speed)
    hex_str = "".join(f"{b:02x}" for b in bits)
    rows.append(["max_speed_rads", "data", "hex2bin", hex_str])

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    print(f"[provision] CSV written to {csv_path}")


def main():
    parser = argparse.ArgumentParser(description="Provision ESP32 NVS configuration")
    parser.add_argument("--port",        default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--ssid",        required=True,          help="WiFi SSID")
    parser.add_argument("--pass",        required=True,          dest="pass_",    help="WiFi password")
    parser.add_argument("--agent-ip",    required=True,          help="micro-ROS agent IPv4")
    parser.add_argument("--agent-port",  type=int, default=8888, help="micro-ROS agent UDP port")
    parser.add_argument("--max-speed",   type=float, default=10.0, help="Max wheel speed (rad/s)")
    parser.add_argument("--wdg-timeout", type=int, default=500,  help="Watchdog timeout (ms)")
    parser.add_argument("--dry-run",     action="store_true",    help="Build CSV/binary only, do not flash")
    args = parser.parse_args()
    # make --pass accessible as args.pass
    setattr(args, "pass", args.pass_)

    nvs_tool = find_nvs_tool()
    tmp_dir = tempfile.mkdtemp(prefix="rover_nvs_")
    csv_path = os.path.join(tmp_dir, "rover_nvs.csv")
    bin_path = os.path.join(tmp_dir, "rover_nvs.bin")

    try:
        build_csv(args, csv_path)

        cmd = [
            sys.executable, nvs_tool, "generate",
            csv_path, bin_path,
            NVS_PARTITION_SIZE,
        ]
        print(f"[provision] Running: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        print(f"[provision] NVS binary: {bin_path}")

        if args.dry_run:
            dest = "rover_nvs.bin"
            shutil.copy(bin_path, dest)
            print(f"[provision] Dry run — binary saved to {dest}")
            return

        flash_cmd = [
            "esptool.py",
            "--port", args.port,
            "--baud", "460800",
            "write_flash",
            NVS_PARTITION_ADDR, bin_path,
        ]
        print(f"[provision] Flashing: {' '.join(flash_cmd)}")
        subprocess.run(flash_cmd, check=True)
        print("[provision] Done — NVS provisioned successfully.")

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
