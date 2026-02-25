/**
 * app_main.c — Rover firmware entry point
 *
 * Boot sequence (FR-010, US2-AC3):
 *   1. Drive all motor GPIO LOW immediately (safe default before LEDC init)
 *   2. Enable TWDT at 2000 ms (hardware safety net)
 *   3. Load configuration from NVS
 *   4. Initialise motor LEDC channels
 *   5. Initialise software watchdog (starts in TIMED_OUT state)
 *   6. Connect to WiFi
 *   7. Init micro-ROS node, subscriber, status reporter
 *   8. Spin executor; on session loss → stop motors → retry
 */

#include "nvs_config.h"
#include "motor.h"
#include "watchdog.h"
#include "wifi.h"
#include "uros_transport.h"
#include "velocity_subscriber.h"
#include "status_reporter.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

/* Spin timeout: 100 ms — gives executor a chance to process incoming messages
 * while keeping the TWDT fed via esp_task_wdt_reset() in the loop.          */
#define SPIN_TIMEOUT_NS    (100ULL * 1000000ULL)   /* 100 ms in nanoseconds */
#define RECONNECT_DELAY_MS 2000

void app_main(void)
{
    /* ── 1. SAFE STATE: stop all motors immediately on every reset ─────── */
    /* motor_init() drives all LEDC channels to 0; GPIO starts low on reset  */
    motor_init();
    motor_stop_all();

    /* ── 2. TWDT: hardware-level watchdog (FR-010) ─────────────────────── */
    /* TWDT may already be initialized by IDF if CONFIG_ESP_TASK_WDT_INIT=y;
     * reconfigure() works regardless of prior state.                         */
    esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 2000,
        .idle_core_mask = 0,   /* don't watch IDLE — main blocks during WiFi */
        .trigger_panic  = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));

    /* IDF auto-subscribes main task when CONFIG_ESP_TASK_WDT_INIT=y.
     * Unsubscribe now so WiFi init (can block >2 s) doesn't trigger TWDT.  */
    esp_task_wdt_delete(NULL);   /* ignore error — may not be subscribed yet */

    /* ── 3. Load NVS configuration ──────────────────────────────────────── */
    RoverConfig cfg;
    nvs_config_load(&cfg);

    /* ── 4. Software watchdog (FR-005) — starts in TIMED_OUT state ─────── */
    watchdog_init(cfg.watchdog_timeout_ms);
    watchdog_expire_cb();   /* ensure safe default before first command (T020) */

    /* ── 5. WiFi ────────────────────────────────────────────────────────── */
    wifi_init_sta(&cfg);

    /* ── 6. micro-ROS + executor spin loop with reconnect ──────────────── */
    while (true) {
        rcl_node_t      node;
        rclc_support_t  support;
        rclc_executor_t executor;

        if (!uros_init(&cfg, &node, &support, &executor)) {
            ESP_LOGE(TAG, "micro-ROS init failed — retry in %d ms",
                     RECONNECT_DELAY_MS);
            watchdog_expire_cb();
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            continue;
        }

        /* Register subscriber and status publisher with the executor */
        velocity_subscriber_init(&node, &executor);
        status_reporter_init(&node, &executor);

        /* Subscribe main task to TWDT only now — blocking init phase is done.
         * Delete first in case we are re-entering after a reconnect.        */
        esp_task_wdt_delete(NULL);          /* no-op if not subscribed */
        ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
        esp_task_wdt_reset();

        ESP_LOGI(TAG, "micro-ROS running — entering spin loop");

        /* Spin until session loss */
        while (true) {
            bool ok = uros_spin_once(&executor, SPIN_TIMEOUT_NS);
            esp_task_wdt_reset();   /* feed TWDT each cycle */

            if (!ok) {
                ESP_LOGW(TAG, "session loss — tearing down and reconnecting");
                break;
            }
        }

        /* Tear down and retry */
        watchdog_expire_cb();
        esp_task_wdt_delete(NULL);          /* unsubscribe before blocking reconnect */
        status_reporter_fini(&node);
        velocity_subscriber_fini(&node);
        uros_fini(&node, &support);

        for (int i = 0; i < RECONNECT_DELAY_MS / 100; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }
    }
}

