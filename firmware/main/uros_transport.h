#pragma once
/**
 * uros_transport.h — micro-ROS Serial (USB CDC-ACM) transport, node, and executor (FR-011)
 *
 * Creates:
 *   Node:      esp32_firmware_node  (namespace /rover)
 *   Transport: USB Serial (CDC-ACM), no WiFi or IP networking required.
 *
 * The ESP32-S3 native USB port appears as /dev/ttyACM* on the host.
 * Start the agent on the host with:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
 *
 * Subscribers and publishers are registered here before executor spin starts.
 * On session loss (RMW_RET_ERROR), motors are stopped immediately and
 * transport re-initialisation is attempted (FR-013).
 */

#include "nvs_config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/** Node name and namespace — must match FR-011 */
#define UROS_NODE_NAME      "esp32_firmware_node"
#define UROS_NAMESPACE      "rover"

/**
 * uros_init() — Set up USB serial transport, allocator, support, node, and executor.
 *
 * @param cfg           RoverConfig (agent_ip / agent_port fields are unused for serial).
 * @param[out] node     Populated rcl_node_t.
 * @param[out] support  Populated rclc_support_t.
 * @param[out] executor Populated rclc_executor_t (capacity for 2 handles).
 *
 * @return true on success, false if transport or node init fails.
 */
/**
 * uros_transport_hw_init — install TinyUSB driver once at boot.
 * Must be called ONCE before the retry loop; calling it again is a no-op.
 */
void uros_transport_hw_init(void);

bool uros_init(const RoverConfig *cfg,
               rcl_node_t        *node,
               rclc_support_t    *support,
               rclc_executor_t   *executor);

/**
 * uros_spin_once() — Run one executor cycle; detect session loss.
 *
 * @param executor  Executor to spin.
 * @param timeout_ns  Timeout for waiting on DDS (nanoseconds).
 *
 * @return true  — cycle completed normally.
 * @return false — session loss detected (motors already stopped by caller context).
 */
bool uros_spin_once(rclc_executor_t *executor, uint64_t timeout_ns);

/**
 * uros_fini() — Destroy executor, node, and transport.
 * Safe to call even if uros_init() failed partway through.
 */
void uros_fini(rcl_node_t *node, rclc_support_t *support);
