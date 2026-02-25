/**
 * watchdog.c — Software watchdog implementation
 */

#include "watchdog.h"
#include "motor.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char *TAG = "watchdog";

volatile WatchdogState g_watchdog_state = WDG_STATE_TIMED_OUT;

static TimerHandle_t s_timer = NULL;

/* FreeRTOS timer callback — runs in timer daemon task context */
static void timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    watchdog_expire_cb();
}

void watchdog_init(uint32_t timeout_ms)
{
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
