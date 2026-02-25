#pragma once
/**
 * wifi.h — WiFi station initialisation
 *
 * Connects to the WiFi network using credentials from RoverConfig.
 * Blocks until an IP address is obtained or until a timeout, then returns.
 * On failure, the firmware logs an error and the micro-ROS init will also
 * fail, which triggers the watchdog safe-stop.
 */

#include "nvs_config.h"

/**
 * wifi_init_sta() — Initialise WiFi in station mode and connect.
 *
 * @param cfg   Pointer to RoverConfig populated by nvs_config_load().
 *              Uses cfg->wifi_ssid and cfg->wifi_pass.
 *
 * Blocks for up to 15 seconds waiting for an IP address.
 * Logs result at INFO or ERROR level.
 */
void wifi_init_sta(const RoverConfig *cfg);
