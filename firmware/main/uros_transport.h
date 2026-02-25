#pragma once
/**
 * uros_transport.h — micro-ROS UDP transport, node, and executor (FR-011)
 *
 * Creates:
 *   Node:      esp32_firmware_node  (namespace /rover)
 *   Transport: UDP, connects to agent_ip:agent_port from RoverConfig
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
 * uros_init() — Set up UDP transport, allocator, support, node, and executor.
 *
 * @param cfg           RoverConfig with agent_ip and agent_port.
 * @param[out] node     Populated rcl_node_t.
 * @param[out] support  Populated rclc_support_t.
 * @param[out] executor Populated rclc_executor_t (capacity for 2 handles).
 *
 * @return true on success, false if transport or node init fails.
 */
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
