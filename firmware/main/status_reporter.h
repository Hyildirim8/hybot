#pragma once
/**
 * status_reporter.h — /firmware_status publisher and JSON serialiser (FR-007)
 *
 * Publishes std_msgs/msg/String (JSON) on /firmware_status at 1 Hz.
 * QoS: BEST_EFFORT / VOLATILE / KEEP_LAST(1) per FR-011.
 *
 * JSON schema (per Key Entities / FirmwareStatus):
 * {
 *   "commanded_speeds": [fl, fr, rl, rr],   // rad/s, 4 floats
 *   "watchdog_state":   "active"|"timed_out",
 *   "motor_faults":     [false, false, false, false],  // 4 booleans
 *   "uptime_ms":        12345,               // integer
 *   "malformed_msg_count": 0                 // integer
 * }
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/** Per-motor fault flags — set by fault detection (currently always false;
 *  extend when BTS7960B IS/EN fault-pin GPIO is wired)                      */
extern volatile bool g_motor_faults[4];

/**
 * FirmwareStatus snapshot — populated just before serialisation.
 */
typedef struct {
    float    commanded_speeds[4];
    bool     watchdog_timed_out;
    bool     motor_faults[4];
    uint64_t uptime_ms;
    uint32_t malformed_msg_count;
} FirmwareStatus;

/**
 * status_serialize() — Render a FirmwareStatus into a JSON string.
 *
 * @param s    Source status snapshot.
 * @param buf  Output buffer.
 * @param len  Buffer size in bytes.
 * @return     Number of characters written (not counting NUL), or -1 on error.
 */
int status_serialize(const FirmwareStatus *s, char *buf, size_t len);

/**
 * status_reporter_init() — Create the /firmware_status publisher and register
 * a 1 Hz rclc_timer (added to the executor) that publishes the current status
 * snapshot.  Publishing runs in the micro-ROS executor task, never in Tmr Svc.
 *
 * @param node      Initialised rcl_node_t.
 * @param executor  Executor (rcl_timer handle added here; capacity must be ≥3).
 */
void status_reporter_init(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *executor);

/**
 * status_reporter_fini() — Destroy publisher resources.
 */
void status_reporter_fini(rcl_node_t *node);
