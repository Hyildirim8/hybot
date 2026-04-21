#pragma once
/**
 * calibration.h — Encoder auto-calibration (009-encoder-auto-cal-brat-on)
 *
 * Provides per-wheel direction sign and speed scale correction factors.
 * Parameters are persisted to NVS and applied automatically on every boot.
 *
 * Direction calibration (US1):
 *   Drives each wheel forward briefly and reads the encoder sign.
 *   Stores int8_t ±1 per wheel to NVS key cal_dir_0..3.
 *   Applied in encoder.c velocity_sample_cb() ISR (before g_encoder_velocities write).
 *
 * Speed calibration (US2):
 *   Runs two motor bursts per wheel (LOW and HIGH duty), measures velocity,
 *   computes a scale factor, stores as float (u32+memcpy) to NVS key cal_spd_0..3.
 *   Applied in velocity_subscriber.c velocity_callback() after clamp_speed().
 *
 * Reset service (US3):
 *   /calibration_reset std_srvs/srv/Empty erases all 8 NVS keys and resets
 *   g_cal_params to safe defaults (dir=+1, scale=1.0, calibrated=false).
 *
 * ISR safety:
 *   g_cal_params.dir_sign[i] is int8_t read by GPTimer ISR (single byte read
 *   is always atomic on all architectures). No mutex required.
 *   g_cal_params.speed_scale[i] is float read by FreeRTOS task only (executor).
 *
 * NVS keys (under "rover_cfg" namespace, all ≤15 chars):
 *   cal_dir_0..3  — int8_t per-wheel direction sign (±1)
 *   cal_spd_0..3  — float per-wheel speed scale [0.5, 2.0] (stored as u32)
 */

#include <stdbool.h>
#include <stdint.h>

/* ─── CalibrationParams ─────────────────────────────────────────────────── */

/**
 * CalibrationParams — per-wheel correction factors loaded from NVS.
 *
 * Declared volatile because dir_sign[] is read from the GPTimer ISR context.
 * The struct is not mutex-protected; individual element reads are ISR-safe
 * (single-byte int8_t, 4-byte aligned float — both atomic on LX7).
 */
typedef struct {
    int8_t  dir_sign[4];    /**< ±1: +1 = encoder wired in forward direction */
    float   speed_scale[4]; /**< [0.5, 2.0]: multiply command speed to equalise */
    bool    calibrated;     /**< true only when ALL 4 direction keys were present in NVS */
} CalibrationParams;

/** Global calibration state. Defined in calibration.c.
 *  Read from ISR (dir_sign) and FreeRTOS tasks (all fields).              */
extern volatile CalibrationParams g_cal_params;

/* ─── Function prototypes ───────────────────────────────────────────────── */

/**
 * calibration_params_load() — Load calibration from NVS into g_cal_params.
 *
 * Opens NVS "rover_cfg" handle and reads cal_dir_0..3 (int8) and
 * cal_spd_0..3 (u32+memcpy float). Missing keys default to dir=+1, scale=1.0.
 * Sets g_cal_params.calibrated = true only if ALL 4 direction keys are present
 * (partial NVS corruption is treated as uncalibrated).
 *
 * Must be called AFTER nvs_config_load() and BEFORE encoder_start().
 */
void calibration_params_load(void);

/**
 * calibration_run() — Blocking calibration sequence (direction + speed passes).
 *
 * Drives each wheel forward at low and high duty, samples encoder velocity,
 * computes direction signs and speed scales, and writes all 8 keys to NVS.
 *
 * TWDT safety: must be called BEFORE esp_task_wdt_add(NULL) in app_main.
 * Execution time: ~16 motor bursts × (350ms settle + 150ms coast) = ~8 s.
 *
 * Motion guard: aborts and returns immediately if any wheel is moving
 * (|g_encoder_velocities[i]| > 0.5 rad/s). Call only when rover is stationary.
 */
void calibration_run(void);

/**
 * calibration_reset_callback() — micro-ROS service callback for /calibration_reset.
 *
 * Signature matches rclc_executor_add_service() requirement.
 * Erases all 8 cal_dir/cal_spd NVS keys and resets g_cal_params to defaults.
 *
 * @param req   Ignored (std_srvs/srv/Empty_Request — no fields)
 * @param res   Ignored (std_srvs/srv/Empty_Response — no fields)
 */
void calibration_reset_callback(const void *req, void *res);

/**
 * cal_gpio_trigger_active() — Check if calibration trigger GPIO is held LOW.
 *
 * Configures CONFIG_CAL_TRIGGER_GPIO as input with internal pull-up and
 * reads the level. Returns true if LOW (active-low convention).
 *
 * @return true if trigger GPIO is asserted (LOW)
 */
bool cal_gpio_trigger_active(void);

/**
 * cal_nvs_keys_absent() — Fast check: has calibration ever been run?
 *
 * Probes cal_dir_0 key only. Returns true if not found.
 * Used as a "never calibrated" gate for CONFIG_CALIBRATE_ON_BOOT logic.
 * Does NOT determine fully calibrated state — use g_cal_params.calibrated for that.
 *
 * @return true if cal_dir_0 is absent from NVS
 */
bool cal_nvs_keys_absent(void);
