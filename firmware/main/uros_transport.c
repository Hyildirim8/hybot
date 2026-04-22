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
#include "calibration.h"    /* calibration_reset_callback (009) */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uxr/client/transport.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>

#include <std_srvs/srv/empty.h>         /* /calibration_reset service type (009) */

static const char *TAG = "uros_transport";

static bool s_usj_installed = false;

/* Allocator must outlive the executor — keep it at file scope.
 * rcl_get_default_allocator() always returns the same function pointers
 * so a single static instance is safe across reconnect cycles.          */
static rcl_allocator_t s_allocator;

/* 009: /calibration_reset service — declared at file scope so it persists
 * across the uros_init call and is valid when the executor spins.       */
static rcl_service_t                    s_reset_service;
static std_srvs__srv__Empty_Request     s_reset_request;
static std_srvs__srv__Empty_Response    s_reset_response;

static bool usj_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    if (!s_usj_installed) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_usj_installed = true;
    }
    return true;
}

static bool usj_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

static size_t usj_write(struct uxrCustomTransport *transport, const uint8_t *buf,
                        size_t len, uint8_t *err)
{
    (void)transport;
    int written = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(20));
    if (written < 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)written;
}

static size_t usj_read(struct uxrCustomTransport *transport, uint8_t *buf,
                       size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    TickType_t to_ticks = (timeout > 0) ? pdMS_TO_TICKS(timeout) : 0;
    int read = usb_serial_jtag_read_bytes(buf, (uint32_t)len, to_ticks);
    if (read < 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)read;
}

void uros_transport_hw_init(void)
{
    if (!s_usj_installed) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
        if (ret == ESP_OK) {
            s_usj_installed = true;
        } else {
            ESP_LOGW(TAG, "boot usj install failed: %s", esp_err_to_name(ret));
        }
    }
}

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
    (void)cfg;
    s_allocator = rcl_get_default_allocator();

    /* Install the USB Serial/JTAG custom transport.
     * framing=true enables XRCE serial framing (required for serial links). */
    rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        usj_open,
        usj_close,
        usj_write,
        usj_read
    );

    if (rmw_ret != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_set_custom_transport failed: %ld", (long)rmw_ret);
        return false;
    }

    /* Build init options (no UDP address needed for serial transport) */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t rc = rcl_init_options_init(&init_options, s_allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rcl_init_options_init failed: %ld", (long)rc);
        return false;
    }

    /* NOTE: Do NOT call rcl_init_options_set_domain_id() for micro-ROS.
     * micro-XRCE-DDS transports all topics inside a single XRCE session;
     * the "domain" is an application-level concept managed by the agent's
     * ROS_DOMAIN_ID env var, not by the XRCE client.  Setting it here
     * causes rmw_microxrcedds to fail immediately on the first spin. */

    /* Initialise rclc support — this performs the XRCE session handshake
     * over the USB serial link.  Blocks until agent responds or times out.
     * TWDT is not watching main during this call (unsubscribed in app_main). */
    rc = rclc_support_init_with_options(support, 0, NULL, &init_options, &s_allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init_with_options failed: %ld", (long)rc);
        (void)rcl_init_options_fini(&init_options);
        return false;
    }

    /* Feed TWDT only if this task is already subscribed (it may not be on the
     * first call — app_main subscribes after uros_init returns).           */
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        esp_task_wdt_reset();
    }

    /* Free init_options after support is initialised */
    (void)rcl_init_options_fini(&init_options);

    /* Create node — name and namespace per FR-011 */
    rc = rclc_node_init_default(node, UROS_NODE_NAME, UROS_NAMESPACE, support);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_node_init_default failed: %ld", (long)rc);
        rclc_support_fini(support);
        return false;
    }

    /* Executor with capacity for: velocity subscriber, status timer, wheel_publisher timer
     * (008: raised 3→4), and calibration_reset service (009: raised 4→5) */
    *executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(executor, &support->context, 5, &s_allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_init failed: %ld", (long)rc);
        (void)rcl_node_fini(node);
        rclc_support_fini(support);
        return false;
    }

    /* 009: Initialise /calibration_reset service (std_srvs/srv/Empty) */
    rc = rclc_service_init_default(
        &s_reset_service, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
        "/calibration_reset");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_service_init_default /calibration_reset failed: %ld", (long)rc);
        (void)rclc_executor_fini(executor);
        (void)rcl_node_fini(node);
        rclc_support_fini(support);
        return false;
    }

    /* Register service callback with executor */
    rc = rclc_executor_add_service(
        executor, &s_reset_service,
        &s_reset_request, &s_reset_response,
        calibration_reset_callback);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_add_service /calibration_reset failed: %ld", (long)rc);
        (void)rcl_service_fini(&s_reset_service, node);
        (void)rclc_executor_fini(executor);
        (void)rcl_node_fini(node);
        rclc_support_fini(support);
        return false;
    }

    ESP_LOGI(TAG, "micro-ROS node %s/%s initialised over USB Serial/JTAG",
             UROS_NAMESPACE, UROS_NODE_NAME);
    return true;
}

bool uros_spin_once(rclc_executor_t *executor, uint64_t timeout_ns)
{
    static int     s_error_count  = 0;
    static int     s_ping_counter = 0;

    /* Ping the agent every ~200 spins (~3 s at 15 ms/spin) to detect loss
     * when spin_some keeps returning OK while the agent is gone (e.g. after
     * agent restart: ESP32 is in HEARTBEAT mode, spin returns TIMEOUT/OK
     * indefinitely without ever detecting the disconnected agent).          */
    s_ping_counter++;
    if (s_ping_counter >= 200) {
        s_ping_counter = 0;
        /* 1 attempt, 200 ms timeout — non-blocking enough at 15 ms spin */
        if (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
            ESP_LOGW(TAG, "ping_agent failed — session loss");
            s_error_count = 0;
            watchdog_expire_cb();
            return false;
        }
    }

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
    (void)rcl_service_fini(&s_reset_service, node);  /* 009 */
    (void)rcl_node_fini(node);
    rclc_support_fini(support);
}
