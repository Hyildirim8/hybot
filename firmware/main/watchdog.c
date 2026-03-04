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

volatile WatchdogState g_watchdog_state = WDG_STATE_TIMED_OUT;

static TimerHandle_t s_timer     = NULL;
static TaskHandle_t  s_wdg_task  = NULL;

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
    /* Spawn the expiry task first so timer_cb can reference s_wdg_task safely */
    xTaskCreate(wdg_expire_task, "wdg_expire", 4096, NULL,
                configMAX_PRIORITIES - 2, &s_wdg_task);

    s_timer = xTimerCreate(
        "wdg_timer",
        pdMS_TO_TICKS(timeout_ms),
        pdFALSE,           /* auto-reload = false; reset manually on each cmd */
        NULL,
        timer_cb);

    if (!s_timer) {
        ESP_LOGE(TAG, "xTimerCreate failed — watchdog not active");
        return;
    }

    /* Start in expired state so no motion occurs before first command (US2-AC3) */
    g_watchdog_state = WDG_STATE_TIMED_OUT;
    ESP_LOGI(TAG, "watchdog_init: %lu ms period, starting TIMED_OUT",
             (unsigned long)timeout_ms);
}

void watchdog_reset(void)
{
    if (!s_timer) return;
    g_watchdog_state = WDG_STATE_ACTIVE;
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
