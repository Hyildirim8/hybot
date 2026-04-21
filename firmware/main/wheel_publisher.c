/**
 * wheel_publisher.c — /wheel_velocities micro-ROS publisher (008-encoder-feedback)
 *
 * Publishes g_encoder_velocities[] as Float32MultiArray on /wheel_velocities
 * at 50 Hz using an rclc_timer (runs in the micro-ROS executor task).
 *
 * The Float32MultiArray data buffer is a static array — no malloc required.
 * This avoids heap fragmentation on restarts and is safe to re-publish from
 * the same buffer every cycle.
 *
 * Lifecycle:
 *   wheel_publisher_init() — called inside micro-ROS retry loop after uros_init
 *   wheel_publisher_fini() — called on session loss before uros_fini
 *   (encoder hardware always running — only publishing stops on disconnect)
 */

#include "wheel_publisher.h"
#include "encoder.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

static const char *TAG = "wheel_pub";

#define WHEEL_COUNT  4

/* Static data buffer — avoids malloc on reconnect */
static float s_data[WHEEL_COUNT];

static rcl_publisher_t             s_publisher;
static std_msgs__msg__Float32MultiArray s_msg;
static rcl_timer_t                 s_timer;

/* ─── rclc timer callback (runs in micro-ROS executor task) ─────────────── */

static void wheel_publish_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (!timer) return;

    /* Sample encoder counters in task context (avoids ISR-time driver calls). */
    encoder_sample_now();

    /* Copy velocities snapshot — avoids ISR tearing on multi-word read */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        s_data[i] = g_encoder_velocities[i];
    }

    s_msg.data.data = s_data;
    s_msg.data.size = WHEEL_COUNT;

    rcl_ret_t rc = rcl_publish(&s_publisher, &s_msg, NULL);
    if (rc != RCL_RET_OK) {
        /* Transient during session teardown — not a fatal error */
        ESP_LOGD(TAG, "rcl_publish failed: %ld", (long)rc);
    }
}

/* ─── Public API ────────────────────────────────────────────────────────── */

bool wheel_publisher_init(rcl_node_t *node, rclc_support_t *support,
                          rclc_executor_t *executor)
{
    /* Initialise message with static buffer — no malloc */
    memset(&s_msg, 0, sizeof(s_msg));
    s_msg.data.data     = s_data;
    s_msg.data.size     = WHEEL_COUNT;
    s_msg.data.capacity = WHEEL_COUNT;

    /* Create RELIABLE publisher on /wheel_velocities */
    rcl_ret_t rc = rclc_publisher_init_default(
        &s_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_velocities");

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create /wheel_velocities publisher: %ld", (long)rc);
        return false;
    }

    /* rclc timer at CONFIG_ENCODER_SAMPLE_PERIOD_MS (default 20 ms = 50 Hz) */
    s_timer = rcl_get_zero_initialized_timer();
    rc = rclc_timer_init_default2(&s_timer, support,
        RCL_MS_TO_NS(CONFIG_ENCODER_SAMPLE_PERIOD_MS),
        wheel_publish_cb, true);

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_timer_init_default2 failed: %ld", (long)rc);
        return false;
    }

    rc = rclc_executor_add_timer(executor, &s_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_add_timer failed: %ld", (long)rc);
        return false;
    }

    ESP_LOGI(TAG, "/wheel_velocities publisher running at %d Hz (executor timer)",
             1000 / CONFIG_ENCODER_SAMPLE_PERIOD_MS);
    return true;
}

void wheel_publisher_fini(rcl_node_t *node)
{
    /* Must fini timer before publisher (rcl ownership order) */
    rcl_timer_fini(&s_timer);
    rcl_publisher_fini(&s_publisher, node);
    /* Note: s_data is static — no free needed */
}
