# Quickstart: 009-encoder-auto-cal-brat-on

**Phase**: 1 — Design
**Feature**: Encoder Auto-Calibration (Boot/Runtime)
**Priority**: US1 (P1/MVP), US2 (P2), US3 (P3)

---

## Prerequisites

- Feature 008 (`encoder-feedback`) fully implemented and tested
- Rover on flat surface, all 4 wheels unobstructed with ~20 cm clearance
- micro-ROS agent running on host: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
- Motors wired; `firmware_status` topic visible

---

## US1 — Direction Calibration (P1/MVP)

**Goal**: All wheels report positive `encoder_velocities` when commanded forward.

### Step 1 — First boot (no NVS keys)

Flash firmware. On first power-on with no calibration keys in NVS:

- If `CONFIG_CALIBRATE_ON_BOOT=y`: calibration auto-runs during boot (see step 2).
- If `CONFIG_CALIBRATE_ON_BOOT=n`: calibration uses defaults (+1 for all). Skip to
  verification; if any wheel shows negative velocity, trigger calibration manually.

### Step 2 — Trigger calibration (GPIO 22 or Kconfig)

**Method A — Button press**: Hold GPIO 22 low (active-low, pull-up) before power-on.
Release after serial port appears. Calibration begins automatically.

**Method B — Kconfig**: Set `CONFIG_CALIBRATE_ON_BOOT=y` in `menuconfig`. Flash.
Calibration runs on every boot when NVS keys are absent.

**During calibration** (total ~6 seconds, rover stationary):

```
BOOT: starting direction calibration for FL...
BOOT: FL direction detected: +1
BOOT: starting direction calibration for FR...
BOOT: FR direction detected: -1
...
BOOT: direction calibration complete. NVS written.
```

> ⚠ Keep the rover on the floor during calibration — wheels will spin briefly.

### Step 3 — Verify direction correction

```bash
ros2 topic echo /wheel_velocities --once
```

Command forward motion:

```bash
ros2 topic pub --once /wheel_velocities_cmd_f32 \
  std_msgs/msg/Float32MultiArray \
  "data: [2.0, 2.0, 2.0, 2.0]"
```

**Expected**: All four values in `/wheel_velocities` are **positive** (within ±0.5 rad/s tolerance at 2.0 rad/s command).

**Failure indication**: One or more values negative → that wheel's direction sign was
not detected or NVS write failed. Check serial log for `ERROR` tag.

---

## US2 — Speed Calibration (P2)

**Goal**: All wheels reach commanded speed within ±5% of each other.

### Trigger

Speed calibration runs in the same `calibration_run()` call as direction calibration,
after the direction pass completes. No separate trigger needed.

During calibration, serial output shows:

```
BOOT: starting speed calibration for FL (30% duty)...
BOOT: FL measured speed at 30%: 3.41 rad/s
BOOT: starting speed calibration for FL (70% duty)...
BOOT: FL measured speed at 70%: 8.02 rad/s
BOOT: FL computed scale: 1.02  (clamped from 1.02)
...
BOOT: speed calibration complete. NVS written.
```

### Verify speed uniformity

Command a constant speed across all wheels:

```bash
ros2 topic pub --rate 10 /wheel_velocities_cmd_f32 \
  std_msgs/msg/Float32MultiArray \
  "data: [5.0, 5.0, 5.0, 5.0]"
```

Read velocities:

```bash
ros2 topic echo /wheel_velocities --once
```

**Expected**: All four `encoder_velocities` within ±5% of each other (e.g., 4.75–5.25 rad/s range all acceptable).

**Failure indication**: One wheel significantly slower/faster → scale factor out of [0.5, 2.0] clamp (possible mechanical fault). Check serial for `WARN` tag.

---

## US3 — Status Visibility & Calibration Reset (P3)

**Goal**: Verify calibration state visible in `/firmware_status`; confirm `/calibration_reset` service restores factory defaults.

### Step 1 — Inspect calibration state

```bash
ros2 topic echo /firmware_status --once | python3 -c \
  "import sys,json; d=json.loads(input()); print('cal_direction:', d['cal_direction']); print('cal_speed_scale:', d['cal_speed_scale'])"
```

**Expected after calibration**:
```
cal_direction:   [1, -1, 1, 1]    # example: FR inverted
cal_speed_scale: [1.02, 0.98, 1.05, 0.97]
```

**Expected on fresh firmware (no calibration)**:
```
cal_direction:   [1, 1, 1, 1]
cal_speed_scale: [1.0, 1.0, 1.0, 1.0]
```

### Step 2 — Call calibration reset service

```bash
ros2 service call /calibration_reset std_srvs/srv/Empty
```

**Expected response**: Service returns immediately (empty response).

### Step 3 — Verify defaults restored

```bash
ros2 topic echo /firmware_status --once | python3 -c \
  "import sys,json; d=json.loads(input()); print('cal_direction:', d['cal_direction']); print('cal_speed_scale:', d['cal_speed_scale'])"
```

**Expected**:
```
cal_direction:   [1, 1, 1, 1]
cal_speed_scale: [1.0, 1.0, 1.0, 1.0]
```

NVS keys are now erased. Next reboot with `CONFIG_CALIBRATE_ON_BOOT=y` will trigger calibration again.

---

## Calibration Kconfig Options

Access via `idf.py menuconfig` → **Rover Calibration**:

| Symbol | Default | Description |
|--------|---------|-------------|
| `CONFIG_CALIBRATE_ON_BOOT` | `n` | Auto-run calibration when NVS keys absent |
| `CONFIG_CAL_TRIGGER_GPIO` | `22` | GPIO pin for manual calibration trigger (active-low) |
| `CONFIG_CAL_BURST_MS` | `500` | Motor burst duration per measurement (ms) |
| `CONFIG_CAL_DUTY_LOW_PCT` | `30` | Low-end duty level for speed profile (%) |
| `CONFIG_CAL_DUTY_HIGH_PCT` | `70` | High-end duty level for speed profile (%) |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Wheel shows 0 velocity after calibration | Encoder wiring fault or motor stall | Check encoder wiring; see feature 008 quickstart |
| Scale factor WARN in serial log | Measured scale outside [0.5, 2.0] | Mechanical fault — check wheel gears, re-run calibration |
| `/calibration_reset` service not found | US3 not implemented (P3) | Implement service or update `CONFIG_CALIBRATE_ON_BOOT=n` and calibrate manually |
| Calibration auto-runs every boot | NVS keys not persisting | Check NVS partition; run `idf.py erase-flash` and re-flash |
| Negative velocities after calibration | GPIO 22 triggered during normal boot | Check button/wiring for false trigger; or disable `CONFIG_CAL_TRIGGER_GPIO` |
