# firmware/main/CMakeLists.txt
#
# All source files in the main component are listed here.
# Add new .c files as they are created in later phases.
idf_component_register(
    SRCS
        "app_main.c"
        "motor.c"
        "nvs_config.c"
        "wifi.c"
        "uros_transport.c"
        "velocity_subscriber.c"
        "watchdog.c"
        "status_reporter.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_timer
        driver
        freertos
        micro_ros_espidf_component
)
