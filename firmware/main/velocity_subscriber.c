/**
 * velocity_subscriber.c — /wheel_velocities subscriber implementation
 */

#include "velocity_subscriber.h"
#include "motor.h"
#include "watchdog.h"

#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

static const char *TAG = "vel_sub";

#define WHEEL_COUNT 4

volatile uint32_t g_malformed_msg_count = 0;

/** Current commanded speeds written by callback, read by status_reporter */
volatile float g_commanded_speeds[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

static rcl_subscription_t s_subscriber;
static std_msgs__msg__Float32MultiArray s_msg;

/* ─── Subscription callback ─────────────────────────────────────────────── */

static void velocity_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    /* Validate array length — must be exactly 4 (FL FR RL RR) */
    if (!msg || msg->data.size != WHEEL_COUNT) {
        g_malformed_msg_count++;
        ESP_LOGW(TAG, "malformed msg: expected 4 floats, got %zu (total: %lu)",
                 msg ? msg->data.size : 0,
                 (unsigned long)g_malformed_msg_count);
        return;
    }

    static const char *WHEEL_NAMES[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};

    /* Process each wheel: clamp → duty → motor_set_pwm */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        float speed = clamp_speed(msg->data.data[i]);   /* FR-004 */
        g_commanded_speeds[i] = speed;

        uint32_t duty = speed_to_duty(speed);           /* FR-002 */

        /* Direction: positive speed → LPWM (forward), negative → RPWM (reverse) */
        uint32_t rpwm = (speed < 0.0f) ? duty : 0u;
        uint32_t lpwm = (speed > 0.0f) ? duty : 0u;

        motor_set_pwm((MotorChannel)i, rpwm, lpwm);     /* FR-003 */

        ESP_LOGI(TAG, "%s: speed=%.3f rad/s  duty=%lu  dir=%s",
                 WHEEL_NAMES[i], (double)speed, (unsigned long)duty,
                 (speed > 0.001f) ? "FWD" : (speed < -0.001f) ? "REV" : "STOP");
    }

    /* Reset watchdog on every valid command (FR-006, T018) */
    watchdog_reset();

    ESP_LOGI(TAG, "cmd [FL FR RL RR]: %.3f  %.3f  %.3f  %.3f  (rad/s)",
             (double)g_commanded_speeds[0], (double)g_commanded_speeds[1],
             (double)g_commanded_speeds[2], (double)g_commanded_speeds[3]);
}

/* ─── Public API ────────────────────────────────────────────────────────── */

void velocity_subscriber_init(rcl_node_t *node, rclc_executor_t *executor)
{
    /* Initialise message memory — micro-ROS requires pre-allocated buffers */
    std_msgs__msg__Float32MultiArray__init(&s_msg);

    /* Allocate data buffer for max 4 floats */
    s_msg.data.capacity = WHEEL_COUNT;
    s_msg.data.data     = (float *)malloc(sizeof(float) * WHEEL_COUNT);
    s_msg.data.size     = 0;

    if (!s_msg.data.data) {
        ESP_LOGE(TAG, "failed to allocate Float32MultiArray buffer");
        return;
    }

    /* Create subscriber with RELIABLE / VOLATILE / KEEP_LAST(1) QoS (FR-011) */
    rcl_ret_t rc = rclc_subscription_init_best_effort(
        &s_subscriber, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_velocities");

    /* rclc_subscription_init_best_effort is BEST_EFFORT; use init_default for RELIABLE */
    if (rc != RCL_RET_OK) {
        /* Fall back to default (RELIABLE) */
        rc = rclc_subscription_init_default(
            &s_subscriber, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/wheel_velocities");
    }

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create /wheel_velocities subscriber: %ld", (long)rc);
        return;
    }

    /* Register callback with executor */
    rc = rclc_executor_add_subscription(
        executor, &s_subscriber, &s_msg,
        &velocity_callback, ON_NEW_DATA);

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to add subscriber to executor: %ld", (long)rc);
        return;
    }

    ESP_LOGI(TAG, "/wheel_velocities subscriber registered (RELIABLE/VOLATILE/KEEP_LAST(1))");
}

void velocity_subscriber_fini(rcl_node_t *node)
{
    rcl_subscription_fini(&s_subscriber, node);
    std_msgs__msg__Float32MultiArray__fini(&s_msg);
}
