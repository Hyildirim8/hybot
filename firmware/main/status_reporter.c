/**
 * status_reporter.c — /firmware_status publisher at 1 Hz
 *
 * Uses an rclc_timer (added to the micro-ROS executor) so that rcl_publish
 * is always called from the micro-ROS task context — never from the FreeRTOS
 * Tmr Svc task, which has a small stack (2 KiB) that cannot handle the full
 * micro-ROS publish call chain.
 */

#include "status_reporter.h"
#include "watchdog.h"
#include "velocity_subscriber.h"   /* g_commanded_speeds, g_malformed_msg_count */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

static const char *TAG = "status";

#define STATUS_BUF_LEN  256
#define STATUS_HZ_NS    (1000000000ULL)   /* 1 Hz in nanoseconds */

volatile bool g_motor_faults[4] = {false, false, false, false};

static rcl_publisher_t        s_publisher;
static std_msgs__msg__String  s_msg;
static char                   s_json_buf[STATUS_BUF_LEN];
static rcl_timer_t            s_timer;

/* ─── JSON serialiser ───────────────────────────────────────────────────── */

int status_serialize(const FirmwareStatus *s, char *buf, size_t len)
{
    return snprintf(buf, len,
        "{"
        "\"commanded_speeds\":[%.3f,%.3f,%.3f,%.3f],"
        "\"watchdog_state\":\"%s\","
        "\"motor_faults\":[%s,%s,%s,%s],"
        "\"uptime_ms\":%llu,"
        "\"malformed_msg_count\":%lu"
        "}",
        (double)s->commanded_speeds[0],
        (double)s->commanded_speeds[1],
        (double)s->commanded_speeds[2],
        (double)s->commanded_speeds[3],
        s->watchdog_timed_out ? "timed_out" : "active",
        s->motor_faults[0] ? "true" : "false",
        s->motor_faults[1] ? "true" : "false",
        s->motor_faults[2] ? "true" : "false",
        s->motor_faults[3] ? "true" : "false",
        (unsigned long long)s->uptime_ms,
        (unsigned long)s->malformed_msg_count);
}

/* ─── rclc timer callback (runs in micro-ROS executor task) ─────────────── */

static void status_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (!timer) return;

    FirmwareStatus snap;

    for (int i = 0; i < 4; i++) {
        snap.commanded_speeds[i] = g_commanded_speeds[i];
        snap.motor_faults[i]     = g_motor_faults[i];
    }
    snap.watchdog_timed_out  = (g_watchdog_state == WDG_STATE_TIMED_OUT);
    snap.uptime_ms           = (uint64_t)(esp_timer_get_time() / 1000ULL);
    snap.malformed_msg_count = g_malformed_msg_count;

    int written = status_serialize(&snap, s_json_buf, sizeof(s_json_buf));
    if (written < 0 || (size_t)written >= sizeof(s_json_buf)) {
        ESP_LOGE(TAG, "status_serialize overflow");
        return;
    }

    s_msg.data.data = s_json_buf;
    s_msg.data.size = (size_t)written;

    rcl_ret_t rc = rcl_publish(&s_publisher, &s_msg, NULL);
    if (rc != RCL_RET_OK) {
        ESP_LOGD(TAG, "rcl_publish failed: %ld (agent may be disconnected)", (long)rc);
    }
}

/* ─── Public API ────────────────────────────────────────────────────────── */

void status_reporter_init(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *executor)
{
    /* Create BEST_EFFORT publisher for /firmware_status (FR-011) */
    rcl_ret_t rc = rclc_publisher_init_best_effort(
        &s_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/firmware_status");

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create /firmware_status publisher: %ld", (long)rc);
        return;
    }

    /* std_msgs/String — data points into s_json_buf, capacity fixed */
    memset(&s_msg, 0, sizeof(s_msg));

    /* rclc 1 Hz timer — fires in executor task, not Tmr Svc */
    s_timer = rcl_get_zero_initialized_timer();
    rc = rclc_timer_init_default2(&s_timer, support,
                                  RCL_MS_TO_NS(1000), status_timer_cb, true);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_timer_init_default failed: %ld", (long)rc);
        return;
    }

    rc = rclc_executor_add_timer(executor, &s_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_add_timer failed: %ld", (long)rc);
        return;
    }

    ESP_LOGI(TAG, "/firmware_status publisher running at 1 Hz (executor timer)");
}

void status_reporter_fini(rcl_node_t *node)
{
    rcl_timer_fini(&s_timer);
    rcl_publisher_fini(&s_publisher, node);
}


