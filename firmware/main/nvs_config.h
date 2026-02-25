#pragma once
/**
 * nvs_config.h — Non-volatile storage configuration loader (FR-012)
 *
 * Stores WiFi credentials, micro-ROS agent address, and rover tuning
 * parameters in ESP32 NVS flash. Values are written once at provisioning
 * time via tools/provision_nvs.py and read at every boot.
 *
 * NVS namespace: "rover_cfg"
 *
 * Key names (max 15 chars, NVS constraint):
 *   "wifi_ssid"      — WiFi SSID (string, max 32 chars)
 *   "wifi_pass"      — WiFi password (string, max 64 chars)
 *   "agent_ip"       — micro-ROS agent IPv4 as string (e.g. "192.168.1.10")
 *   "agent_port"     — micro-ROS agent UDP port (uint16)
 *   "max_speed_rads" — maximum wheel speed in rad/s (float, default 10.0)
 *   "wdg_timeout_ms" — watchdog period in ms (uint32, default 500)
 */

#include <stdint.h>

#define NVS_NAMESPACE       "rover_cfg"
#define NVS_KEY_SSID        "wifi_ssid"
#define NVS_KEY_PASS        "wifi_pass"
#define NVS_KEY_AGENT_IP    "agent_ip"
#define NVS_KEY_AGENT_PORT  "agent_port"
#define NVS_KEY_MAX_SPEED   "max_speed_rads"
#define NVS_KEY_WDG_TIMEOUT "wdg_timeout_ms"

/** Rover runtime configuration populated from NVS (or compile-time defaults). */
typedef struct {
    char     wifi_ssid[33];       /**< WiFi SSID, NUL-terminated             */
    char     wifi_pass[65];       /**< WiFi password, NUL-terminated         */
    char     agent_ip[16];        /**< Agent IPv4 string, NUL-terminated     */
    uint16_t agent_port;          /**< Agent UDP port (default 8888)         */
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
