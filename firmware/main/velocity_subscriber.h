#pragma once
/**
 * velocity_subscriber.h — micro-ROS subscriber for /wheel_velocities (FR-001)
 *
 * Subscribes to std_msgs/msg/Float32MultiArray on /wheel_velocities.
 * QoS: RELIABLE / VOLATILE / KEEP_LAST(1) per FR-011.
 * Array must have exactly 4 elements; FL[0] FR[1] RL[2] RR[3].
 *
 * On each valid message:
 *   1. Clamp each speed with clamp_speed()          (FR-004)
 *   2. Convert to duty cycle with speed_to_duty()   (FR-002)
 *   3. Call motor_set_pwm() for all 4 channels      (FR-003)
 *   4. Call watchdog_reset()                        (FR-006)
 *
 * On malformed message (length ≠ 4 or deserialisation error):
 *   Discard and increment g_malformed_msg_count.    (FR-008)
 */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/** Malformed message counter — read by status_reporter.c */
extern volatile uint32_t g_malformed_msg_count;

/** Last commanded speeds (rad/s) — read by status_reporter.c */
extern volatile float g_commanded_speeds[4];

/**
 * velocity_subscriber_pid_init() — Initialise PID controllers for all 4 wheels.
 *
 * Reads gains from Kconfig (CONFIG_PID_KP/KI/KD/INTEGRAL_MAX, ÷1000 scaled).
 * Must be called once at boot, before the micro-ROS spin loop.
 */
void velocity_subscriber_pid_init(void);

/**
 * velocity_subscriber_init() — Create the /wheel_velocities subscriber and
 * register its callback with the executor.
 *
 * @param node      Initialised rcl_node_t.
 * @param executor  Executor to register the subscription callback with.
 */
bool velocity_subscriber_init(rcl_node_t *node, rclc_executor_t *executor);

/**
 * velocity_subscriber_fini() — Destroy subscriber resources.
 */
void velocity_subscriber_fini(rcl_node_t *node);
