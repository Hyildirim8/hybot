# Interface Contracts: 009-encoder-auto-cal-brat-on

**Phase**: 1 — Design
**Date**: 2026-03-06
**Depends on**: data-model.md, research.md (R-005, R-007)

---

## Overview

This feature introduces one **new** ROS 2 interface and extends one **existing** interface.
The two velocity topics (`/wheel_velocities`, `/wheel_velocities_cmd_f32`) are transparent
to calibration — direction sign and speed scale corrections are applied inside the firmware
pipeline and are not visible at the ROS 2 layer.

---

## Topic 1: `/firmware_status` (EXTENDED)

**Direction**: Firmware → ROS 2
**Type**: `std_msgs/msg/String` (JSON payload)
**QoS**: Reliable, Transient-Local, depth 1

### Existing fields (unchanged)

```json
{
  "commanded_speeds":    [0.0, 0.0, 0.0, 0.0],
  "watchdog_timed_out":  false,
  "motor_faults":        [false, false, false, false],
  "uptime_ms":           12345,
  "malformed_msg_count": 0,
  "encoder_counts":      [0, 0, 0, 0],
  "encoder_velocities":  [0.0, 0.0, 0.0, 0.0],
  "encoder_faults":      [false, false, false, false]
}
```

### New fields (added by this feature, US3)

```json
{
  "cal_direction":   [1, -1, 1, 1],
  "cal_speed_scale": [1.02, 0.98, 1.05, 0.97]
}
```

### Full schema after this feature

```json
{
  "commanded_speeds":    [number, number, number, number],
  "watchdog_timed_out":  boolean,
  "motor_faults":        [boolean, boolean, boolean, boolean],
  "uptime_ms":           integer,
  "malformed_msg_count": integer,
  "encoder_counts":      [integer, integer, integer, integer],
  "encoder_velocities":  [number, number, number, number],
  "encoder_faults":      [boolean, boolean, boolean, boolean],
  "cal_direction":       [integer, integer, integer, integer],
  "cal_speed_scale":     [number, number, number, number]
}
```

### Field semantics

| Field | Index | Units | Range | Semantics |
|-------|-------|-------|-------|-----------|
| `cal_direction` | FL=0,FR=1,RL=2,RR=3 | — | `[-1, +1]` | Active direction correction factor. +1 = no inversion; -1 = encoder output inverted. Reflects current `g_cal_params.dir_sign`. |
| `cal_speed_scale` | FL=0,FR=1,RL=2,RR=3 | — | [0.5, 2.0] | Active speed scaling factor per wheel. 1.0 = unity gain. Reflects current `g_cal_params.speed_scale`. |

### Change compatibility

This is an **additive** change to the JSON payload. Existing consumers that do not
read `cal_direction` or `cal_speed_scale` are unaffected.

---

## Service 1: `/calibration_reset` (NEW — US3, P3)

**Direction**: ROS 2 → Firmware
**Type**: `std_srvs/srv/Empty`
**QoS**: Reliable (default service QoS)

### Purpose

Erase all calibration keys from NVS and restore in-memory `g_cal_params` to factory
defaults (+1 direction, 1.0 scale). The rover continues operating immediately using
defaults. Primarily used for testing, re-calibration workflows, or recovery from
a bad calibration run.

### Request

```
--- (empty)
```

### Response

```
--- (empty)
```

### Effect

1. In `calibration_reset_callback()` (executor task context):
   - Erase NVS keys `cal_dir_0..3` and `cal_spd_0..3` via `nvs_erase_key`
   - Single `nvs_commit()` after all erases
   - Reset `g_cal_params.dir_sign[]` to +1
   - Reset `g_cal_params.speed_scale[]` to 1.0
   - Set `g_cal_params.calibrated = false`
2. Respond with empty response (no error reporting — success assumed)
3. Next `/firmware_status` publish reflects factory defaults

### Registration

```c
// uros_transport.c — executor capacity must be 5 (was 4 post-008)
rclc_service_init_default(&reset_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
    "/calibration_reset");

rclc_executor_add_service(&executor,
    &reset_service,
    &reset_request, &reset_response,
    calibration_reset_callback);
```

### Constraints

| Constraint | Value |
|------------|-------|
| `RMW_UXRCE_MAX_SERVICES` | 1 (already set in `colcon.meta` — no rebuild needed) |
| Executor capacity | 5 (incremented from 4 by this feature) |
| Blocking NVS operations | In executor task only — no ISR or timer context |

---

## Unchanged Topics (reference)

The following topics from prior features are **not modified** by this feature.
Direction sign and speed scale corrections are applied transparently inside firmware.

| Topic | Type | Notes |
|-------|------|-------|
| `/wheel_velocities` | `std_msgs/msg/Float32MultiArray` | direction-corrected velocities already reflected |
| `/wheel_velocities_cmd_f32` | `std_msgs/msg/Float32MultiArray` | speed scale applied before PWM, transparent to sender |

---

## RMW Resource Summary (cumulative)

| Resource | `colcon.meta` key | Value post-009 |
|----------|-------------------|----------------|
| Publishers | `RMW_UXRCE_MAX_PUBLISHERS` | 3 (set by feature 008) |
| Subscribers | `RMW_UXRCE_MAX_SUBSCRIPTIONS` | 1 |
| Services | `RMW_UXRCE_MAX_SERVICES` | **1** (no change — pre-existing) |
| Executor handles | capacity arg | **5** (4 from 008 + 1 service) |
