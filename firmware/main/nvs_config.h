#pragma once
/**
 * nvs_config.h — Non-volatile storage configuration loader (FR-012)
 *
 * Stores rover tuning parameters in ESP32 NVS flash.
 * WiFi credentials and agent IP/port are no longer used — micro-ROS
 * communicates over USB serial (CDC-ACM) instead.
 *
 * NVS namespace: "rover_cfg"
 *
 * Key names (max 15 chars, NVS constraint):
 *   "max_speed_rads" — maximum wheel speed in rad/s (float, default 10.0)
 *   "wdg_timeout_ms" — watchdog period in ms (uint32, default 500)
 */

#include <stdint.h>

#define NVS_NAMESPACE       "rover_cfg"
#define NVS_KEY_MAX_SPEED   "max_speed_rads"
#define NVS_KEY_WDG_TIMEOUT "wdg_timeout_ms"

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
