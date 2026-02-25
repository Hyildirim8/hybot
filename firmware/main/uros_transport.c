/**
 * uros_transport.c — micro-ROS UDP transport and node lifecycle
 *
 * Uses the built-in UDP/WiFi transport provided by micro_ros_espidf_component:
 *   - WiFi is already connected by wifi.c before uros_init() is called
 *   - Agent IP/port are passed via rmw_uros_options_set_udp_address
 *   - rclc_support_init_with_options wires up the DDS participant to the agent
 */

#include "uros_transport.h"
#include "motor.h"
#include "watchdog.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "uros_transport";

/* Allocator must outlive the executor — keep it at file scope.
 * rcl_get_default_allocator() always returns the same function pointers
 * so a single static instance is safe across reconnect cycles.          */
static rcl_allocator_t s_allocator;

/* Convenience macro: log and return false on RCL error */
#define UROS_CHECK(fn, label)                                       \
    do {                                                            \
        rcl_ret_t _rc = (fn);                                       \
        if (_rc != RCL_RET_OK) {                                    \
            ESP_LOGE(TAG, label " failed: %ld", (long)_rc);        \
            return false;                                           \
        }                                                           \
    } while (0)

bool uros_init(const RoverConfig *cfg,
               rcl_node_t        *node,
               rclc_support_t    *support,
               rclc_executor_t   *executor)
{
    s_allocator = rcl_get_default_allocator();

    /* Build init options and set agent UDP address */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    UROS_CHECK(rcl_init_options_init(&init_options, s_allocator),
               "rcl_init_options_init");

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);

    /* Convert agent_port (uint16) to string for the API */
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", cfg->agent_port);

    UROS_CHECK(rmw_uros_options_set_udp_address(cfg->agent_ip, port_str,
                                                rmw_options),
               "rmw_uros_options_set_udp_address");

    /* NOTE: Do NOT call rcl_init_options_set_domain_id() for micro-ROS.
     * micro-XRCE-DDS transports all topics inside a single XRCE session;
     * the "domain" is an application-level concept managed by the agent's
     * ROS_DOMAIN_ID env var, not by the XRCE client.  Setting it here
     * causes rmw_microxrcedds to fail immediately on the first spin. */

    /* Initialise rclc support — this performs the XRCE session handshake.
     * Blocks until agent responds or times out (typically 1-3 s).
     * TWDT is not watching main during this call (unsubscribed in app_main). */
    UROS_CHECK(rclc_support_init_with_options(support, 0, NULL,
                                              &init_options, &s_allocator),
               "rclc_support_init_with_options");

    esp_task_wdt_reset();   /* session handshake done; feed TWDT if subscribed */

    /* Free init_options after support is initialised */
    rcl_init_options_fini(&init_options);

    /* Create node — name and namespace per FR-011 */
    UROS_CHECK(rclc_node_init_default(node, UROS_NODE_NAME, UROS_NAMESPACE,
                                      support),
               "rclc_node_init_default");

    /* Executor with capacity for velocity subscriber + status timer (2 handles) */
    *executor = rclc_executor_get_zero_initialized_executor();
    UROS_CHECK(rclc_executor_init(executor, &support->context, 3, &s_allocator),
               "rclc_executor_init");

    ESP_LOGI(TAG, "micro-ROS node /%s/%s initialised, agent %s:%s",
             UROS_NAMESPACE, UROS_NODE_NAME,
             cfg->agent_ip, port_str);
    return true;
}

bool uros_spin_once(rclc_executor_t *executor, uint64_t timeout_ns)
{
    static int s_error_count = 0;
    rcl_ret_t rc = rclc_executor_spin_some(executor, timeout_ns);

    if (rc == RCL_RET_OK || rc == RCL_RET_TIMEOUT) {
        s_error_count = 0;
        return true;
    }

    /* RCL_RET_ERROR (1) can fire transiently on the first few spins while
     * the XRCE session is still being confirmed by the agent.
     * Only treat it as a session loss after several consecutive failures.  */
    s_error_count++;
    ESP_LOGW(TAG, "executor spin error %ld (count=%d)", (long)rc, s_error_count);

    if (s_error_count < 5) {
        return true;   /* tolerate up to 4 transient errors */
    }

    s_error_count = 0;
    ESP_LOGE(TAG, "persistent spin error — session loss");
    watchdog_expire_cb();
    return false;
}

void uros_fini(rcl_node_t *node, rclc_support_t *support)
{
    /* Best-effort cleanup; errors here are non-fatal */
    (void)rcl_node_fini(node);
    rclc_support_fini(support);
}
