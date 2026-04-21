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
#define DEFAULT_MAX_SPEED       10.0f
#define DEFAULT_WDG_TIMEOUT_MS  1000u
#define MIN_WDG_TIMEOUT_MS      200u
#define MAX_WDG_TIMEOUT_MS      10000u

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
    cfg->max_speed_rad_s     = DEFAULT_MAX_SPEED;
    cfg->watchdog_timeout_ms = DEFAULT_WDG_TIMEOUT_MS;

    nvs_handle_t h = open_nvs();
    if (!h) {
        ESP_LOGW(TAG, "NVS unavailable — using compile-time defaults");
        return;
    }

    /* max_speed stored as IEEE-754 little-endian uint32 blob */
    uint32_t speed_bits;
    size_t blob_len = sizeof(speed_bits);
    if (nvs_get_blob(h, NVS_KEY_MAX_SPEED, &speed_bits, &blob_len) == ESP_OK)
        memcpy(&cfg->max_speed_rad_s, &speed_bits, sizeof(float));

    uint32_t wdg;
    if (nvs_get_u32(h, NVS_KEY_WDG_TIMEOUT, &wdg) == ESP_OK)
        cfg->watchdog_timeout_ms = wdg;

    /* Guard against stale/corrupt NVS values that can make the watchdog
     * expire between valid command frames and keep motors permanently stopped. */
    if (cfg->watchdog_timeout_ms < MIN_WDG_TIMEOUT_MS) {
        ESP_LOGW(TAG, "wdg_timeout_ms too low (%lu) -> clamped to %lu",
                 (unsigned long)cfg->watchdog_timeout_ms,
                 (unsigned long)MIN_WDG_TIMEOUT_MS);
        cfg->watchdog_timeout_ms = MIN_WDG_TIMEOUT_MS;
    } else if (cfg->watchdog_timeout_ms > MAX_WDG_TIMEOUT_MS) {
        ESP_LOGW(TAG, "wdg_timeout_ms too high (%lu) -> clamped to %lu",
                 (unsigned long)cfg->watchdog_timeout_ms,
                 (unsigned long)MAX_WDG_TIMEOUT_MS);
        cfg->watchdog_timeout_ms = MAX_WDG_TIMEOUT_MS;
    }

    nvs_close(h);
    ESP_LOGI(TAG, "config: maxspd=%.1f wdg=%lums (USB serial transport)",
             cfg->max_speed_rad_s, (unsigned long)cfg->watchdog_timeout_ms);
}
