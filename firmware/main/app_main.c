/**
 * app_main.c — Rover firmware entry point
 *
 * Boot sequence (FR-010, US2-AC3):
 *   1. Drive all motor GPIO LOW immediately (safe default before LEDC init)
 *   2. Enable TWDT at 2000 ms (hardware safety net)
 *   3. Load configuration from NVS
 *   4. Initialise motor LEDC channels
 *   5. Initialise software watchdog (starts in TIMED_OUT state)
 *   6. Init micro-ROS node over USB serial, subscriber, status reporter
 *   7. Spin executor; on session loss → stop motors → retry
 *
 * WiFi is no longer used. The ESP32-S3 native USB port (CDC-ACM) carries
 * the micro-ROS XRCE session directly to the host via /dev/ttyACM*.
 */

#include "nvs_config.h"
#include "motor.h"
#include "watchdog.h"
#include "uros_transport.h"
#include "velocity_subscriber.h"
#include "status_reporter.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

/* Spin timeout: 50 ms — drain the USB CDC RX buffer frequently so XRCE frames
 * are not delayed behind a 100 ms poll boundary.  TWDT is fed each iteration
 * (2000 ms limit) so the tighter loop is safe.                              */
#define SPIN_TIMEOUT_NS    (50ULL * 1000000ULL)    /* 50 ms in nanoseconds */
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

    /* ── 3. Load NVS configuration ──────────────────────────────────────── */
    RoverConfig cfg;
    nvs_config_load(&cfg);

    /* ── 4. Software watchdog (FR-005) — starts in TIMED_OUT state ─────── */
    watchdog_init(cfg.watchdog_timeout_ms);
    watchdog_expire_cb();   /* ensure safe default before first command (T020) */

    /* ── 5. One-time TinyUSB driver install (must happen before retry loop) */
    /* Installing inside the retry loop causes ESP_ERR_INVALID_STATE on the   */
    /* second attempt, leaving CDC-ACM uninitialized and silently broken.      */
    uros_transport_hw_init();

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
        status_reporter_init(&node, &support, &executor);

        /* Subscribe main task to TWDT only now — blocking init phase is done.
         * Only delete first if already subscribed (avoids 'task not found').*/
        if (esp_task_wdt_status(NULL) == ESP_OK) {
            esp_task_wdt_delete(NULL);
        }
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

            /* Software watchdog is reset only in velocity_callback() when a
             * command actually arrives — not here.  This means motors stop
             * if commands cease even while the XRCE session stays up.      */
        }

        /* Tear down and retry */
        watchdog_expire_cb();
        esp_task_wdt_delete(NULL);          /* unsubscribe before blocking reconnect */
        status_reporter_fini(&node);
        velocity_subscriber_fini(&node);
        uros_fini(&node, &support);

        /* Simple delay — no TWDT reset here, task is unsubscribed above. */
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
}

