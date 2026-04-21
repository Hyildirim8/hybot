#include "esp32s2_usbcdc_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *CDC_TAG = "cdc_transport";

/* Set to true after tinyusb_driver_install() succeeds once. */
static bool s_tusb_installed = false;

void uros_transport_hw_init(void)
{
    if (s_tusb_installed) return;

    const tinyusb_config_t tinyusb_config = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    esp_err_t ret = tinyusb_driver_install(&tinyusb_config);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE /* already installed */) {
        s_tusb_installed = true;
    }
    /* Give the host 200 ms to enumerate the USB device before we proceed */
    vTaskDelay(pdMS_TO_TICKS(200));
}

// Open USB-CDC
bool esp32s2_usbcdc_open(struct uxrCustomTransport* transport) {
    /* TinyUSB driver must have been installed by uros_transport_hw_init().
     * If somehow not installed yet, try once here as a fallback.          */
    if (!s_tusb_installed) {
        uros_transport_hw_init();
        if (!s_tusb_installed) return false;
    }

    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = *cdc_port,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    esp_err_t init_ret = tusb_cdc_acm_init(&acm_cfg);
    if (init_ret != ESP_OK) {
        if (init_ret == ESP_ERR_INVALID_STATE || tusb_cdc_acm_initialized(*cdc_port)) {
            /* Already initialized — this can happen when rmw_uros_ping_agent
             * opened and then re-closed the transport, or on reconnect when
             * close was not called before open. Treat as success.          */
            ESP_LOGD(CDC_TAG, "CDC already initialized — skipping re-init");
        } else {
            ESP_LOGE(CDC_TAG, "tusb_cdc_acm_init failed: %s", esp_err_to_name(init_ret));
            return false;
        }
    }

    /* Wait for the host to open the CDC port (DTR asserted) before returning.
     * Some host drivers (e.g. micro-ros-agent using O_NONBLOCK) never assert
     * DTR, so we cap the wait at 3 s and proceed anyway — the XRCE session
     * handshake will simply retry on the next uros_init() cycle if the agent
     * isn't ready yet.
     * Poll at 50 ms intervals; the TWDT is not subscribed during init.       */
    for (int i = 0; i < 60; i++) {   /* 60 × 50 ms = 3 s max */
        if (tud_cdc_n_connected((uint8_t)*cdc_port)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return true;
}

// Close USB-CDC — flush pending TX only; keep CDC-ACM initialized so the
// next open() can proceed without re-init. Deinit/reinit of CDC-ACM is
// fragile because rmw_init.c calls CLOSE_TRANSPORT on session-creation
// failure, which races against the next open() call.
bool esp32s2_usbcdc_close(struct uxrCustomTransport* transport) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    /* Flush any pending output — ignore errors since we're closing anyway */
    tinyusb_cdcacm_write_flush(*cdc_port, 0);
    /* Do NOT call tusb_cdc_acm_deinit — CDC stays alive for the next open() */
    return true;
}

// Write to USB-CDC
size_t esp32s2_usbcdc_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    size_t tx_size = tinyusb_cdcacm_write_queue(*cdc_port, buf, len);
    tinyusb_cdcacm_write_flush(*cdc_port, 0);
    return tx_size;
}

// Read from USB-CDC
size_t esp32s2_usbcdc_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(*cdc_port, buf, len, &rx_size);
    return (ret == ESP_OK) ? rx_size : 0;
}