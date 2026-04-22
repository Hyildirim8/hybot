#pragma once
/**
 * watchdog.h — Software watchdog and TWDT management (FR-005, FR-006, FR-010)
 *
 * Software watchdog: FreeRTOS xTimerCreate with configurable period.
 * Expiry callback stops all motors and sets g_watchdog_state = TIMED_OUT.
 * Calling watchdog_reset() from the subscriber callback restarts the timer.
 *
 * TWDT (hardware): enabled in app_main via esp_task_wdt_init() at 2000 ms.
 * This file only manages the software watchdog; TWDT management lives in app_main.c.
 */

#include <stdbool.h>
#include <stdint.h>

/** Watchdog state visible to status_reporter */
typedef enum {
    WDG_STATE_ACTIVE    = 0,   /**< Commands arriving within timeout */
    WDG_STATE_TIMED_OUT = 1,   /**< No command received for > timeout_ms */
} WatchdogState;

/** Global watchdog state — read by status_reporter.c */
extern volatile WatchdogState g_watchdog_state;

/**
 * watchdog_init() — Create and start the FreeRTOS software watchdog timer.
 *
 * @param timeout_ms  Period in milliseconds (from NVS or default 500 ms).
 *
 * MUST be called after motor_init() (expire callback calls motor_stop_all()).
 */
void watchdog_init(uint32_t timeout_ms);

/**
 * watchdog_reset() — Restart the timer; call on every valid command receipt.
 * Sets g_watchdog_state = WDG_STATE_ACTIVE.
 */
void watchdog_reset(void);

/**
 * watchdog_expire_cb() — Fire the safe-stop immediately.
 *
 * Also called directly by uros_transport on session loss (FR-013) and by
 * the FreeRTOS timer callback on expiry. Safe to call from any task context.
 */
void watchdog_expire_cb(void);
