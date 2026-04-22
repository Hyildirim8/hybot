/**
 * velocity_subscriber.c — /wheel_velocities subscriber implementation
 *
 * 010: PID closed-loop control replaces the previous open-loop speed_to_duty()
 * mapping. Each wheel now has a PID controller that compares the commanded
 * velocity (from ROS2) against the encoder-measured velocity and outputs
 * a PWM duty correction, ensuring actual wheel speed matches commanded speed.
 */

#include "velocity_subscriber.h"
#include "motor.h"
#include "watchdog.h"
#include "calibration.h"       /* g_cal_params.speed_scale[] (009) */
#include "pid_controller.h"    /* PidState, pid_update, g_pid_errors/outputs (010) */
#include "encoder.h"           /* g_encoder_velocities[] (008) */

#include "esp_log.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>

static const char *TAG = "vel_sub";

#define WHEEL_COUNT 4

volatile uint32_t g_malformed_msg_count = 0;

/** Current commanded speeds written by callback, read by status_reporter */
volatile float g_commanded_speeds[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

static rcl_subscription_t s_subscriber;
static std_msgs__msg__Float32MultiArray s_msg;

/* ─── PID state (010) ──────────────────────────────────────────────────── */

static PidState s_pid[WHEEL_COUNT];
static int64_t  s_last_cmd_time_us = 0;   /* microseconds from esp_timer */

/* Feedforward scale: PWM_MAX_TICKS / MAX_SPEED_RAD_S ≈ 54.56 ticks per rad/s.
 * This gives the PID a baseline output matching the old speed_to_duty() mapping
 * so the PID only has to correct for the error, not generate the full duty.  */
static float s_ff_scale = 0.0f;

/* ─── PID initialisation (called from app_main before spin loop) ──────── */

void velocity_subscriber_pid_init(void)
{
    float kp   = (float)CONFIG_PID_KP / 1000.0f;
    float ki   = (float)CONFIG_PID_KI / 1000.0f;
    float kd   = (float)CONFIG_PID_KD / 1000.0f;
    float imax = (float)CONFIG_PID_INTEGRAL_MAX / 1000.0f;

    /* Output range: PID correction in duty ticks (added to feedforward).
     * Allow full range so PID can fully override feedforward if needed.    */
    float out_min = -(float)PWM_MAX_TICKS;
    float out_max =  (float)PWM_MAX_TICKS;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        pid_init(&s_pid[i], kp, ki, kd, out_min, out_max, imax);
    }

    /* Compute feedforward scale: ticks per rad/s. This replicates the
     * old speed_to_duty() linear mapping as a baseline.                    */
    s_ff_scale = (float)PWM_MAX_TICKS / MAX_SPEED_RAD_S;

    s_last_cmd_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "PID init: Kp=%.4f Ki=%.4f Kd=%.4f Imax=%.4f ff_scale=%.2f",
             (double)kp, (double)ki, (double)kd, (double)imax, (double)s_ff_scale);
}

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

    /* Compute dt from last callback */
    int64_t now_us = esp_timer_get_time();
    float dt_s = (float)(now_us - s_last_cmd_time_us) / 1000000.0f;
    s_last_cmd_time_us = now_us;

    /* Clamp dt to sane range: [1ms, 200ms] */
    if (dt_s < 0.001f) dt_s = 0.001f;
    if (dt_s > 0.200f) dt_s = 0.200f;

    static const char *WHEEL_NAMES[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};

    for (int i = 0; i < WHEEL_COUNT; i++) {
        float speed = clamp_speed(msg->data.data[i]);   /* FR-004 */

        /* 009: Apply per-wheel speed scale after clamp */
        speed *= g_cal_params.speed_scale[i];

        g_commanded_speeds[i] = speed;

        /* Read encoder-measured velocity (written by encoder_sample_now()) */
        float measured = g_encoder_velocities[i];

        /* 010: Feedforward + PID closed-loop control.
         *
         * Feedforward: ff = speed * s_ff_scale  (baseline duty from linear model)
         * PID:         correction = PID(speed, measured, dt)  (error correction)
         * Output:      duty = ff + correction  (combined)
         *
         * When commanded speed is ~0, reset PID and output zero. */
        float total_output;
        if (fabsf(speed) < 0.01f) {
            pid_reset(&s_pid[i]);
            total_output = 0.0f;
        } else {
            /* Feedforward: linear speed-to-duty mapping (like old speed_to_duty) */
            float ff = speed * s_ff_scale;

            /* PID correction: closes the loop using encoder feedback.
             * PID operates in duty-ticks domain by scaling error by ff_scale. */
            float error = speed - measured;
            float pid_correction = pid_update(&s_pid[i], speed, measured, dt_s);
            /* Scale PID output from rad/s error domain to duty ticks domain */
            pid_correction *= s_ff_scale;

            total_output = ff + pid_correction;
        }

        /* Store PID diagnostics for status_reporter */
        g_pid_errors[i]  = speed - measured;
        g_pid_outputs[i] = total_output;

        /* Clamp total output to valid duty range */
        if (total_output > (float)PWM_MAX_TICKS)  total_output = (float)PWM_MAX_TICKS;
        if (total_output < -(float)PWM_MAX_TICKS) total_output = -(float)PWM_MAX_TICKS;

        /* Convert signed output to RPWM/LPWM duty.
         * output > 0 → forward (LPWM), output < 0 → reverse (RPWM). */
        uint32_t duty = (uint32_t)(fabsf(total_output) + 0.5f);
        if (duty > PWM_MAX_TICKS) duty = PWM_MAX_TICKS;

        uint32_t rpwm = (total_output < 0.0f) ? duty : 0u;
        uint32_t lpwm = (total_output > 0.0f) ? duty : 0u;

        motor_set_pwm((MotorChannel)i, rpwm, lpwm);     /* FR-003 */

        ESP_LOGD(TAG, "%s: cmd=%.3f meas=%.3f err=%.3f out=%.1f duty=%lu dir=%s",
                 WHEEL_NAMES[i], (double)speed, (double)measured,
                 (double)g_pid_errors[i], (double)total_output,
                 (unsigned long)duty,
                 (total_output > 0.001f) ? "FWD" : (total_output < -0.001f) ? "REV" : "STOP");
    }

    /* Reset watchdog on every valid command (FR-006, T018) */
    watchdog_reset();

    ESP_LOGD(TAG, "cmd [FL FR RL RR]: %.3f  %.3f  %.3f  %.3f  (rad/s)",
             (double)g_commanded_speeds[0], (double)g_commanded_speeds[1],
             (double)g_commanded_speeds[2], (double)g_commanded_speeds[3]);
}

/* ─── Public API ────────────────────────────────────────────────────────── */

bool velocity_subscriber_init(rcl_node_t *node, rclc_executor_t *executor)
{
    /* Initialise message memory — micro-ROS requires pre-allocated buffers */
    std_msgs__msg__Float32MultiArray__init(&s_msg);

    /* Allocate data buffer for max 4 floats */
    s_msg.data.capacity = WHEEL_COUNT;
    s_msg.data.data     = (float *)malloc(sizeof(float) * WHEEL_COUNT);
    s_msg.data.size     = 0;

    if (!s_msg.data.data) {
        ESP_LOGE(TAG, "failed to allocate Float32MultiArray buffer");
        return false;
    }

    /* Keep command channel RELIABLE so host control messages are not dropped
     * by transport-specific QoS compatibility edge cases. */
    rcl_ret_t rc = rclc_subscription_init_default(
        &s_subscriber, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_velocities_cmd_f32");

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create /wheel_velocities subscriber: %ld", (long)rc);
        return false;
    }

    /* Register callback with executor */
    rc = rclc_executor_add_subscription(
        executor, &s_subscriber, &s_msg,
        &velocity_callback, ON_NEW_DATA);

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to add subscriber to executor: %ld", (long)rc);
        return false;
    }

    ESP_LOGI(TAG, "/wheel_velocities_cmd_f32 subscriber registered (RELIABLE, PID closed-loop)");
    return true;
}

void velocity_subscriber_fini(rcl_node_t *node)
{
    rcl_subscription_fini(&s_subscriber, node);
    std_msgs__msg__Float32MultiArray__fini(&s_msg);
}
