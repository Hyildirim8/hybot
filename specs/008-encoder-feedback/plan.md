# Implementation Plan: Wheel Encoder Feedback

**Branch**: `008-encoder-feedback` | **Date**: 2026-03-06 | **Spec**: [spec.md](spec.md)

## Summary

Add quadrature encoder reading to the ESP32-S3 firmware using hardware PCNT peripherals,
publish measured wheel velocities at 50 Hz over micro-ROS on `/wheel_velocities`, and
enable closed-loop odometry in `mecanum_drive_controller` once feedback is verified.
No new ROS 2 packages or nodes are required — only firmware additions and one config
parameter change.

## Technical Context

**Language/Version**: C17 (ESP-IDF component), Python 3.10 (ROS 2 config only)
**Primary Dependencies**:
- `driver/pulse_cnt.h` (`esp_driver_pcnt` component) — hardware PCNT quadrature decoding
- `driver/gptimer.h` (`esp_driver_gptimer` component) — periodic velocity sampling timer
- `micro_ros_espidf_component` — existing micro-ROS transport (requires `colcon.meta` rebuild)
- `mecanum_drive_controller` (ROS 2 Humble) — existing; `open_loop: false` activation
**Storage**: N/A (Kconfig compile-time parameters; no NVS additions required)
**Testing**: Manual hardware bring-up (spin-by-hand, drive 1m, 10-revolution count verification)
**Target Platform**: ESP32-S3-WROOM-1 (Xtensa LX7 dual-core, 512 KB SRAM, no PSRAM)
**Project Type**: Embedded firmware + ROS 2 controller config
**Performance Goals**: Velocity publish latency ≤ 25 ms; 50 Hz sustained; PCNT handles 5907 ticks/sec/wheel (well within hardware capability)
**Constraints**: 4 PCNT units available = 4 encoders maximum (zero margin). `RMW_UXRCE_MAX_PUBLISHERS` must be raised to 3 (was 2) and `libmicroros.a` rebuilt. SRAM budget: ~80–100 KB free heap; adding encoder subsystem costs < 2 KB.
**Scale/Scope**: 4 new firmware source files (`encoder.h`, `encoder.c`, `wheel_publisher.h`, `wheel_publisher.c`); 5 modified existing files; 1 config parameter change

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-checked after Phase 1 design.*

| Principle | Check | Status |
|-----------|-------|--------|
| **I. ROS2-First Architecture** | Encoder data exposed as a ROS 2 topic (`/wheel_velocities`). No direct inter-process calls. All data flows through standard ROS 2 middleware. | ✅ PASS |
| **II. Mecanum Kinematics Correctness** | This feature does not change kinematics. It provides state feedback to `mecanum_drive_controller`, which already has correct geometry parameters. `open_loop: false` enables the controller to use measured velocities — the kinematic model is unchanged. | ✅ PASS |
| **III. Joystick Input via joy** | No changes to joystick or teleop subsystems. | ✅ PASS |
| **IV. Hardware Abstraction Layer** | All encoder hardware code (PCNT, GPTimer, GPIO) is in the ESP32 firmware (`encoder.c`). No ROS 2 node knows about GPIO pins or PCNT registers. ROS 2 side sees only a Float32MultiArray topic. ⚠️ **Pre-existing violation noted**: The active firmware uses USB CDC-ACM transport (`esp32s2_usbcdc_transport`); the constitution §IV mandates WiFi micro-ROS UDP. This encoder feature does not introduce or worsen the transport choice — it is a pre-existing issue requiring a separate constitution amendment or transport migration task outside this feature's scope. | ⚠️ PRE-EXISTING |
| **V. Observability & Diagnostics** | `/firmware_status` JSON extended with `encoder_counts`, `encoder_velocities`, `encoder_faults`. `quickstart.md` documents `rosbag2` recording command. | ✅ PASS |
| **VI. Simplicity & Incremental Delivery** | 3 independently testable user stories (US1 → US2 → US3). US1 is complete and independently valuable. `open_loop: false` is gated behind US1 verification — not prematurely activated. | ✅ PASS |

**Post-design re-check**: Constitution §IV has a pre-existing transport violation (USB CDC-ACM vs. mandated WiFi UDP) — not introduced by this feature. All other principles pass. No Complexity Tracking section required for this feature.

## Project Structure

### Documentation (this feature)

```text
specs/008-encoder-feedback/
├── spec.md              ✅ complete
├── plan.md              ✅ this file
├── research.md          ✅ R-001 through R-007 resolved
├── data-model.md        ✅ entities, state transitions, relationships
├── quickstart.md        ✅ US1/US2 verification steps, troubleshooting
├── contracts/
│   └── topics.md        ✅ /wheel_velocities, /firmware_status, /odom
└── tasks.md             ⏳ Phase 2 output (/speckit.tasks — NOT yet created)
```

### Source Code Changes

```text
firmware/
├── main/
│   ├── encoder.h             NEW — EncoderChannel config, GPIO defaults,
│   │                               shared globals declarations
│   ├── encoder.c             NEW — PCNT init/start, GPTimer alarm callback,
│   │                               velocity computation, noise suppression
│   ├── wheel_publisher.h     NEW — wheel_publisher_init/fini declarations
│   ├── wheel_publisher.c     NEW — rclc_timer at 50 Hz, publishes /wheel_velocities
│   ├── app_main.c            MOD — add encoder_init(), encoder_start() before
│   │                               retry loop; add wheel_publisher_init/fini;
│   │                               SPIN_TIMEOUT_NS 50ms → 15ms
│   ├── status_reporter.h     MOD — extend FirmwareStatus struct (3 new fields)
│   ├── status_reporter.c     MOD — extend status_serialize() with encoder JSON
│   ├── uros_transport.c      MOD — executor capacity 3 → 4
│   └── CMakeLists.txt        MOD — add encoder.c, wheel_publisher.c to SRCS;
│                                   add esp_driver_pcnt, esp_driver_gptimer to REQUIRES
└── components/
    └── micro_ros_espidf_component/
        └── colcon.meta       MOD — RMW_UXRCE_MAX_PUBLISHERS 2 → 3; rebuild required

config/
└── controllers.yaml          MOD — open_loop: true → false (GATED: only after US1 verified)
```

## Complexity Tracking

No Constitution violations. No complexity justification required.
