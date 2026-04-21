# ecza-robotu Development Guidelines

Auto-generated from all feature plans. Last updated: 2026-02-24

## Active Technologies
- Python 3.10 (ROS2 launch files, scripts), C++17 (007-fix-nav2-slam)
- `/maps` bind-mount volume (host `./maps/`) for saved map files; no database (007-fix-nav2-slam)
- C17 (ESP-IDF component), Python 3.10 (ROS 2 config only) (008-encoder-feedback)
- N/A (Kconfig compile-time parameters; no NVS additions required) (008-encoder-feedback)

- Dockerfile (multi-stage), YAML (Compose v3.8+), Bash (helper scripts) (006-docker-runtime)

## Project Structure

```text
src/
tests/
```

## Commands

# Add commands for Dockerfile (multi-stage), YAML (Compose v3.8+), Bash (helper scripts)

## Code Style

Dockerfile (multi-stage), YAML (Compose v3.8+), Bash (helper scripts): Follow standard conventions

## Recent Changes
- 009-encoder-auto-cal-brat-on: Added [if applicable, e.g., PostgreSQL, CoreData, files or N/A]
- 008-encoder-feedback: Added C17 (ESP-IDF component), Python 3.10 (ROS 2 config only)
- 008-encoder-feedback: Added [if applicable, e.g., PostgreSQL, CoreData, files or N/A]


<!-- MANUAL ADDITIONS START -->
## 009-encoder-auto-cal-brat-on — Encoder Auto-Calibration

**Technology**: C17 (ESP-IDF), NVS flash storage, ESP-IDF GPTimer ISR, micro-ROS `std_srvs/srv/Empty` service

**New files**:
- `firmware/main/calibration.h` — `CalibrationParams` struct, `g_cal_params` extern declaration, API
- `firmware/main/calibration.c` — `calibration_params_load()`, `calibration_run()`, `calibration_reset_callback()`, `cal_gpio_trigger_active()`

**Modified files**:
- `firmware/main/encoder.c` — apply `g_cal_params.dir_sign[i]` in `velocity_sample_cb()` before `g_encoder_velocities[]`
- `firmware/main/velocity_subscriber.c` — apply `g_cal_params.speed_scale[i]` after `clamp_speed()`, before `speed_to_duty()`
- `firmware/main/app_main.c` — add `calibration_params_load()`, `encoder_init/start`, calibration trigger check before retry loop
- `firmware/main/status_reporter.h/.c` — add `cal_direction[4]` (int8_t) and `cal_speed_scale[4]` (float) fields to `FirmwareStatus`
- `firmware/main/nvs_config.h` — add `NVS_KEY_CAL_DIR_0..3` and `NVS_KEY_CAL_SPD_0..3` constants
- `firmware/main/Kconfig.projbuild` — add "Rover Calibration" menu (CONFIG_CALIBRATE_ON_BOOT, CONFIG_CAL_TRIGGER_GPIO, etc.)
- `firmware/main/uros_transport.c` — executor capacity 4→5 (US3 only)
- `firmware/main/CMakeLists.txt` — add `calibration.c` to SRCS

**Key design invariants**:
- `g_cal_params` is `volatile CalibrationParams`; `dir_sign` reads are ISR-safe (atomic int8_t on LX7)
- `speed_scale` reads only in executor task context — no mutex needed
- NVS keys: `cal_dir_0..3` (nvs_set_i8), `cal_spd_0..3` (nvs_set_u32 + memcpy float) under `rover_cfg` namespace
- Scale factor clamped to [0.5, 2.0] before NVS write; out-of-range → WARN log
- Calibration runs in TWDT-safe window (before `esp_task_wdt_add(NULL)`) — no watchdog changes
- GPIO 22 is the only safe free GPIO (all others allocated to motors, encoders, USB, strapping pins)
<!-- MANUAL ADDITIONS END -->
