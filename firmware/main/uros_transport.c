/**
 * uros_transport.c — micro-ROS USB CDC-ACM transport and node lifecycle
 *
 * Uses TinyUSB CDC-ACM custom transport (esp32s2_usbcdc_transport component):
 *   - Connect the ESP32-S3 USB-C port directly to the host PC / RPi
 *   - Device appears as /dev/ttyACM0 (or ttyACM1) on the host
 *   - No WiFi, GPIO wiring, or IP networking required
 *   - Start the micro-ROS agent on the host with:
 *       ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
 *     OR via docker compose (SERIAL_DEV=/dev/ttyACM0 docker compose up)
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
#include <rmw_microros/custom_transport.h>

#include "esp32s2_usbcdc_transport.h"   /* TinyUSB CDC-ACM custom transport */

static const char *TAG = "uros_transport";

/* Use the first CDC-ACM interface (the only one we register) */
static tinyusb_cdcacm_itf_t s_cdc_port = TINYUSB_CDC_ACM_0;

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

    /* Install the USB CDC-ACM custom transport.
     * framing=true enables XRCE serial framing (required for CDC/UART).
     * args=&s_cdc_port passes the CDC interface number to open/close/read/write. */
    rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
        true,
        (void *)&s_cdc_port,
        esp32s2_usbcdc_open,
        esp32s2_usbcdc_close,
        esp32s2_usbcdc_write,
        esp32s2_usbcdc_read
    );

    if (rmw_ret != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_set_custom_transport failed: %ld", (long)rmw_ret);
        return false;
    }

    /* Build init options (no UDP address needed for serial transport) */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    UROS_CHECK(rcl_init_options_init(&init_options, s_allocator),
               "rcl_init_options_init");

    /* NOTE: Do NOT call rcl_init_options_set_domain_id() for micro-ROS.
     * micro-XRCE-DDS transports all topics inside a single XRCE session;
     * the "domain" is an application-level concept managed by the agent's
     * ROS_DOMAIN_ID env var, not by the XRCE client.  Setting it here
     * causes rmw_microxrcedds to fail immediately on the first spin. */

    /* Initialise rclc support — this performs the XRCE session handshake
     * over the USB serial link.  Blocks until agent responds or times out.
     * TWDT is not watching main during this call (unsubscribed in app_main). */
    UROS_CHECK(rclc_support_init_with_options(support, 0, NULL,
                                              &init_options, &s_allocator),
               "rclc_support_init_with_options");

    /* Feed TWDT only if this task is already subscribed (it may not be on the
     * first call — app_main subscribes after uros_init returns).           */
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        esp_task_wdt_reset();
    }

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

    ESP_LOGI(TAG, "micro-ROS node /%s/%s initialised over USB CDC-ACM",
             UROS_NAMESPACE, UROS_NODE_NAME);
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
