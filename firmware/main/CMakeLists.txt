# firmware/main/CMakeLists.txt
#
# All source files in the main component are listed here.
# Add new .c files as they are created in later phases.
idf_component_register(
    SRCS
        "app_main.c"
        "motor.c"
        "nvs_config.c"
        "uros_transport.c"
        "velocity_subscriber.c"
        "watchdog.c"
        "status_reporter.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_timer
        driver
        freertos
        micro_ros_espidf_component
        esp32s2_usbcdc_transport
        espressif__esp_tinyusb
)

# micro-ROS ships its own atomic builtins that duplicate newlib's stdatomic;
# allow both definitions rather than failing at link time.
target_link_options(${COMPONENT_LIB} INTERFACE -Wl,--allow-multiple-definition)
