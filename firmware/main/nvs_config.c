/**
 * nvs_config.c — NVS configuration loader implementation
 */

#include "nvs_config.h"

#include <string.h>

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_config";

/* Compile-time defaults (used when NVS key is absent) */
#define DEFAULT_AGENT_PORT      8888u
#define DEFAULT_MAX_SPEED       10.0f
#define DEFAULT_WDG_TIMEOUT_MS  500u
#define DEFAULT_AGENT_IP        "192.168.1.1"
#define DEFAULT_SSID            ""
#define DEFAULT_PASS            ""

/* ─── Internal helpers ──────────────────────────────────────────────────── */

static nvs_handle_t open_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return 0;
    }
    return handle;
}

/* ─── Public API ────────────────────────────────────────────────────────── */

void nvs_config_load(RoverConfig *cfg)
{
    /* Initialise NVS flash partition; erase if version mismatch */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, erasing…");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Populate defaults first, then override from NVS */
    strncpy(cfg->wifi_ssid,  DEFAULT_SSID,      sizeof(cfg->wifi_ssid)  - 1);
    strncpy(cfg->wifi_pass,  DEFAULT_PASS,      sizeof(cfg->wifi_pass)  - 1);
    strncpy(cfg->agent_ip,   DEFAULT_AGENT_IP,  sizeof(cfg->agent_ip)   - 1);
    cfg->agent_port          = DEFAULT_AGENT_PORT;
    cfg->max_speed_rad_s     = DEFAULT_MAX_SPEED;
    cfg->watchdog_timeout_ms = DEFAULT_WDG_TIMEOUT_MS;

    nvs_handle_t h = open_nvs();
    if (!h) {
        ESP_LOGW(TAG, "NVS unavailable — using compile-time defaults");
        return;
    }

    size_t len;

    len = sizeof(cfg->wifi_ssid);
    if (nvs_get_str(h, NVS_KEY_SSID, cfg->wifi_ssid, &len) != ESP_OK)
        ESP_LOGW(TAG, "%s not set in NVS", NVS_KEY_SSID);

    len = sizeof(cfg->wifi_pass);
    if (nvs_get_str(h, NVS_KEY_PASS, cfg->wifi_pass, &len) != ESP_OK)
        ESP_LOGW(TAG, "%s not set in NVS", NVS_KEY_PASS);

    len = sizeof(cfg->agent_ip);
    if (nvs_get_str(h, NVS_KEY_AGENT_IP, cfg->agent_ip, &len) != ESP_OK)
        ESP_LOGW(TAG, "%s not set in NVS — using default %s",
                 NVS_KEY_AGENT_IP, DEFAULT_AGENT_IP);

    uint16_t port;
    if (nvs_get_u16(h, NVS_KEY_AGENT_PORT, &port) == ESP_OK)
        cfg->agent_port = port;

    /* max_speed stored as IEEE-754 little-endian uint32 blob */
    uint32_t speed_bits;
    size_t blob_len = sizeof(speed_bits);
    if (nvs_get_blob(h, NVS_KEY_MAX_SPEED, &speed_bits, &blob_len) == ESP_OK)
        memcpy(&cfg->max_speed_rad_s, &speed_bits, sizeof(float));

    uint32_t wdg;
    if (nvs_get_u32(h, NVS_KEY_WDG_TIMEOUT, &wdg) == ESP_OK)
        cfg->watchdog_timeout_ms = wdg;

    nvs_close(h);
    ESP_LOGI(TAG, "config: ssid=%s agent=%s:%u maxspd=%.1f wdg=%lums",
             cfg->wifi_ssid, cfg->agent_ip, cfg->agent_port,
             cfg->max_speed_rad_s, (unsigned long)cfg->watchdog_timeout_ms);
}

void nvs_config_write_str(const char *key, const char *value)
{
    nvs_handle_t h = open_nvs();
    if (!h) return;
    ESP_ERROR_CHECK(nvs_set_str(h, key, value));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}

void nvs_config_write_u16(const char *key, uint16_t value)
{
    nvs_handle_t h = open_nvs();
    if (!h) return;
    ESP_ERROR_CHECK(nvs_set_u16(h, key, value));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}

void nvs_config_write_float(const char *key, float value)
{
    nvs_handle_t h = open_nvs();
    if (!h) return;
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    ESP_ERROR_CHECK(nvs_set_blob(h, key, &bits, sizeof(bits)));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}

void nvs_config_write_u32(const char *key, uint32_t value)
{
    nvs_handle_t h = open_nvs();
    if (!h) return;
    ESP_ERROR_CHECK(nvs_set_u32(h, key, value));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}
