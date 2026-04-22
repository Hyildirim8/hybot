/**
 * watchdog.c — Software watchdog implementation
 */

#include "watchdog.h"
#include "motor.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "watchdog";

#define MIN_WDG_TIMEOUT_MS 200u
#define MAX_WDG_TIMEOUT_MS 10000u

volatile WatchdogState g_watchdog_state = WDG_STATE_TIMED_OUT;

static TimerHandle_t s_timer     = NULL;
static TaskHandle_t  s_wdg_task  = NULL;
static StaticTimer_t s_timer_buf;

/* Dedicated watchdog expiry task — has its own stack so that motor_stop_all()
 * and ESP_LOGW do not execute inside the tiny Tmr Svc stack (≈2 KiB).      */
static void wdg_expire_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        watchdog_expire_cb();
    }
}

/* FreeRTOS timer callback — runs in Tmr Svc; do NO heavy work here.
 * Just unblock the dedicated expiry task via a direct notification.         */
static void timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (s_wdg_task) {
        xTaskNotifyGive(s_wdg_task);
    }
}

void watchdog_init(uint32_t timeout_ms)
{
    if (timeout_ms < MIN_WDG_TIMEOUT_MS) {
        ESP_LOGW(TAG, "watchdog timeout too low (%lu) -> %lu ms",
                 (unsigned long)timeout_ms, (unsigned long)MIN_WDG_TIMEOUT_MS);
        timeout_ms = MIN_WDG_TIMEOUT_MS;
    } else if (timeout_ms > MAX_WDG_TIMEOUT_MS) {
        ESP_LOGW(TAG, "watchdog timeout too high (%lu) -> %lu ms",
                 (unsigned long)timeout_ms, (unsigned long)MAX_WDG_TIMEOUT_MS);
        timeout_ms = MAX_WDG_TIMEOUT_MS;
    }

    /* Spawn the expiry task first so timer_cb can reference s_wdg_task safely */
    xTaskCreate(wdg_expire_task, "wdg_expire", 4096, NULL,
                configMAX_PRIORITIES - 2, &s_wdg_task);

    s_timer = xTimerCreateStatic(
        "wdg_timer",
        pdMS_TO_TICKS(timeout_ms),
        pdFALSE,           /* auto-reload = false; reset manually on each cmd */
        NULL,
        timer_cb,
        &s_timer_buf);

    if (!s_timer) {
        ESP_LOGE(TAG, "xTimerCreateStatic failed — fallback to ACTIVE state");
        g_watchdog_state = WDG_STATE_ACTIVE;
        return;
    }

    /* Start in expired state so no motion occurs before first command (US2-AC3) */
    g_watchdog_state = WDG_STATE_TIMED_OUT;
    ESP_LOGI(TAG, "watchdog_init: %lu ms period, starting TIMED_OUT",
             (unsigned long)timeout_ms);
}

void watchdog_reset(void)
{
    g_watchdog_state = WDG_STATE_ACTIVE;

    if (!s_timer) return;

    /* Reset the timer from any task context (xTimerResetFromISR if needed) */
    if (xPortInIsrContext()) {
        BaseType_t woken = pdFALSE;
        xTimerResetFromISR(s_timer, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xTimerReset(s_timer, pdMS_TO_TICKS(10));
    }
}

void watchdog_expire_cb(void)
{
    g_watchdog_state = WDG_STATE_TIMED_OUT;
    motor_stop_all();
    ESP_LOGW(TAG, "watchdog expired — all motors stopped");
}
