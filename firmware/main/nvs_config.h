#pragma once
/**
 * nvs_config.h — Non-volatile storage configuration loader
 *
 * Stores rover tuning parameters in ESP32 NVS flash.
 * The ESP32 communicates with the ROS 2 graph via WiFi micro-ROS (UDP).
 *
 * NVS namespace: "rover_cfg"
 *
 * Key names (max 15 chars, NVS constraint):
 *   "max_speed_rads" — maximum wheel speed in rad/s (float, default 10.0)
 *   "wdg_timeout_ms" — watchdog period in ms (uint32, default 500)
 *   "cal_dir_0..3"   — per-wheel encoder direction signs (int8, ±1)
 *   "cal_spd_0..3"   — per-wheel speed scale factors (float via u32, [0.5,2.0])
 */

#include <stdint.h>

#define NVS_NAMESPACE       "rover_cfg"
#define NVS_KEY_MAX_SPEED   "max_speed_rads"
#define NVS_KEY_WDG_TIMEOUT "wdg_timeout_ms"

/* 009: Per-wheel encoder calibration keys (all ≤15 chars — NVS constraint) */
#define NVS_KEY_CAL_DIR_0   "cal_dir_0"     /**< int8: direction sign wheel FL (±1) */
#define NVS_KEY_CAL_DIR_1   "cal_dir_1"     /**< int8: direction sign wheel FR (±1) */
#define NVS_KEY_CAL_DIR_2   "cal_dir_2"     /**< int8: direction sign wheel RL (±1) */
#define NVS_KEY_CAL_DIR_3   "cal_dir_3"     /**< int8: direction sign wheel RR (±1) */
#define NVS_KEY_CAL_SPD_0   "cal_spd_0"     /**< u32/float: speed scale wheel FL [0.5,2.0] */
#define NVS_KEY_CAL_SPD_1   "cal_spd_1"     /**< u32/float: speed scale wheel FR */
#define NVS_KEY_CAL_SPD_2   "cal_spd_2"     /**< u32/float: speed scale wheel RL */
#define NVS_KEY_CAL_SPD_3   "cal_spd_3"     /**< u32/float: speed scale wheel RR */

/** Rover runtime configuration populated from NVS (or compile-time defaults). */
typedef struct {
    float    max_speed_rad_s;     /**< Max wheel speed rad/s (default 10.0)  */
    uint32_t watchdog_timeout_ms; /**< Software watchdog period (default 500)*/
} RoverConfig;

/**
 * nvs_config_load() — Initialise NVS and populate a RoverConfig struct.
 *
 * Falls back to compile-time defaults for any missing NVS key so the
 * firmware is functional without provisioning for bench testing.
 *
 * @param[out] cfg  Pointer to a RoverConfig to be populated.
 */
void nvs_config_load(RoverConfig *cfg);

/**
 * nvs_config_write_str() — Write a string value to NVS (provisioning helper).
 *
 * Used by the provisioning flow; not called at runtime.
 */
void nvs_config_write_str(const char *key, const char *value);

/**
 * nvs_config_write_u16() — Write a uint16 value to NVS.
 */
void nvs_config_write_u16(const char *key, uint16_t value);

/**
 * nvs_config_write_float() — Write a float value to NVS (stored as uint32 blob).
 */
void nvs_config_write_float(const char *key, float value);

/**
 * nvs_config_write_u32() — Write a uint32 value to NVS.
 */
void nvs_config_write_u32(const char *key, uint32_t value);
