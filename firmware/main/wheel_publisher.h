#pragma once
/**
 * wheel_publisher.h — /wheel_velocities micro-ROS publisher (008-encoder-feedback)
 *
 * Publishes measured wheel velocities from g_encoder_velocities[] as a
 * std_msgs/Float32MultiArray on /wheel_velocities at 50 Hz (20 ms period).
 *
 * The publisher uses an rclc_timer (added to the micro-ROS executor) so that
 * rcl_publish is always called from the micro-ROS task context.
 *
 * QoS: RELIABLE, depth=1 (matches ros2_control bridge subscriber).
 *
 * Wheel order in array: [FL, FR, RL, RR] — index = MotorChannel.
 * Values are in rad/s, positive = forward, negative = reverse.
 *
 * Lifecycle:
 *   - wheel_publisher_init() must be called INSIDE the micro-ROS retry loop
 *     (after uros_init returns success) — not before.
 *   - wheel_publisher_fini() is called on session loss before uros_fini().
 *   - encoder hardware (encoder.c) is always running regardless of publish state.
 */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * wheel_publisher_init() — Create the /wheel_velocities publisher and register
 * a 50 Hz rclc_timer (added to the executor) that publishes g_encoder_velocities[].
 *
 * @param node      Initialised rcl_node_t.
 * @param support   rclc_support_t (needed for rclc_timer_init_default2).
 * @param executor  Executor — timer handle added here; capacity must be ≥4 (008).
 */
bool wheel_publisher_init(rcl_node_t *node, rclc_support_t *support,
                          rclc_executor_t *executor);

/**
 * wheel_publisher_fini() — Destroy publisher and timer resources.
 *
 * Call before uros_fini() on session loss or shutdown.
 * Does NOT free data buffer (it is a static array, not malloc'd).
 */
void wheel_publisher_fini(rcl_node_t *node);
