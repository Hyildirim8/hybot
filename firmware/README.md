# Firmware — ESP32-S3 Motor Controller

**Feature**: `001-esp32-firmware`
**Target**: ESP32-S3-WROOM-1 via ESP-IDF v5.2+
**Transport**: micro-ROS over WiFi UDP (micro_ros_espidf_component)

---

## Quick start

```bash
# 1. Activate ESP-IDF
source ~/esp/esp-idf/export.sh

# 2. Provision WiFi and agent credentials (once per device)
cd firmware
python3 ../tools/provision_nvs.py \
    --port /dev/ttyUSB0 \
    --ssid  "YourSSID" \
    --pass  "YourPassword" \
    --agent-ip   "192.168.x.x" \
    --agent-port 8888

# 3. Build
idf.py build

# 4. Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Topic interface

| Topic | Direction | Message type | QoS | Description |
|-------|-----------|-------------|-----|-------------|
| `/wheel_velocities` | Subscribe | `std_msgs/msg/Float32MultiArray` | RELIABLE / VOLATILE / KEEP_LAST(1) | Per-wheel speed commands in rad/s; array length must be 4; order: FL[0] FR[1] RL[2] RR[3] |
| `/firmware_status`  | Publish   | `std_msgs/msg/String` (JSON)    | BEST_EFFORT / VOLATILE / KEEP_LAST(1) | Status at ≥1 Hz; see JSON schema below |

### ROS2 node identity

| Attribute | Value |
|-----------|-------|
| Node name | `esp32_firmware_node` |
| Namespace | `/rover` |
| Full name | `/rover/esp32_firmware_node` |

Verify after launch:
```bash
ros2 node list          # shows /rover/esp32_firmware_node
ros2 topic list         # shows /wheel_velocities and /firmware_status
ros2 topic hz /firmware_status   # should be ≥1 Hz
```

---

## /firmware_status JSON schema

```json
{
  "commanded_speeds":    [0.0, 0.0, 0.0, 0.0],
  "watchdog_state":      "active" | "timed_out",
  "motor_faults":        [false, false, false, false],
  "uptime_ms":           12345,
  "malformed_msg_count": 0
}
```

Fields:

| Field | Type | Description |
|-------|------|-------------|
| `commanded_speeds` | float[4] | Last valid commanded speed per wheel (FL FR RL RR), rad/s |
| `watchdog_state` | string | `"active"` = commands arriving; `"timed_out"` = watchdog stopped motors |
| `motor_faults` | bool[4] | Per-motor fault flags (FL FR RL RR); currently always false until fault-pin GPIO is wired |
| `uptime_ms` | uint | Firmware uptime in milliseconds since boot |
| `malformed_msg_count` | uint | Count of discarded malformed `/wheel_velocities` messages |

---

## GPIO pinout

Default pin assignments (configurable via `idf.py menuconfig` → *Rover Motor GPIO Configuration*):

| Signal | GPIO | Motor | Direction |
|--------|------|-------|-----------|
| FL_RPWM | 4  | Front-Left  | Reverse (negative speed) |
| FL_LPWM | 5  | Front-Left  | Forward (positive speed) |
| FR_RPWM | 6  | Front-Right | Reverse |
| FR_LPWM | 7  | Front-Right | Forward |
| RL_RPWM | 15 | Rear-Left   | Reverse |
| RL_LPWM | 16 | Rear-Left   | Forward |
| RR_RPWM | 17 | Rear-Right  | Reverse |
| RR_LPWM | 18 | Rear-Right  | Forward |

**BTS7960B wiring**: Connect RPWM/LPWM to the corresponding driver pins.
Hold EN HIGH (tie to 3.3 V or drive from a GPIO). The firmware does not
currently drive EN pins programmatically; wire them permanently HIGH.

---

## NVS key reference

All values stored in NVS namespace `rover_cfg`:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `wifi_ssid` | string | *(empty)* | WiFi SSID |
| `wifi_pass` | string | *(empty)* | WiFi password |
| `agent_ip`  | string | `192.168.1.1` | micro-ROS agent IPv4 |
| `agent_port` | uint16 | `8888` | micro-ROS agent UDP port |
| `max_speed_rads` | float blob | `10.0` | Max wheel speed (rad/s) — update after calibration |
| `wdg_timeout_ms` | uint32 | `500` | Software watchdog period (ms) |

Write values using `tools/provision_nvs.py`. The firmware falls back to
compile-time defaults for any missing key, so the device is functional for
bench testing without provisioning (no WiFi credentials → WiFi connection
will fail, but the motor logic is still exercisable via a direct micro-ROS
connection if credentials are hardcoded in NVS).

---

## Watchdog behaviour

**Default timeout**: 500 ms (configurable via NVS key `wdg_timeout_ms`).

- Starts in `TIMED_OUT` state on every boot — no motion before first command.
- Resets on every valid `/wheel_velocities` message.
- On expiry: all motors stop immediately; `watchdog_state` → `"timed_out"`.
- On next valid command: normal operation resumes.
- On micro-ROS session loss: watchdog fires immediately (does not wait for timer expiry).
- TWDT (hardware): 2000 ms (4× software timeout); triggers a full system reset.

---

## Build system

| Command | Purpose |
|---------|---------|
| `idf.py build` | Compile firmware |
| `idf.py set-target esp32s3` | Set target (already set via `sdkconfig`) |
| `idf.py menuconfig` | Configure GPIO pins, WiFi, etc. |
| `idf.py fullclean` | Remove all build artefacts |
| `idf.py -p PORT flash` | Flash to device |
| `idf.py -p PORT monitor` | Serial monitor (Ctrl+] to exit) |
| `idf.py -p PORT flash monitor` | Flash and immediately monitor |

**micro-ROS component**: `components/micro_ros_espidf_component/`
Cloned from https://github.com/micro-ROS/micro_ros_espidf_component (humble branch).
Do not modify files inside this directory; they are managed separately.
