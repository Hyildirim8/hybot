# firmware/main/CMakeLists.txt
#
# All source files in the main component are listed here.
# Add new .c files as they are created in later phases.
# 008: esp_driver_pcnt and esp_driver_gptimer added to REQUIRES for quadrature encoder support.
idf_component_register(
    SRCS
        "app_main.c"
        "encoder.c"
        "motor.c"
        "nvs_config.c"
        "uros_transport.c"
        "velocity_subscriber.c"
        "watchdog.c"
        "status_reporter.c"
        "wheel_publisher.c"
        "calibration.c"
        "pid_controller.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_timer
        driver
        esp_driver_pcnt
        esp_driver_gptimer
        freertos
        micro_ros_espidf_component
        esp32s2_usbcdc_transport
        espressif__esp_tinyusb
)

# SRCS: encoder.c (008), wheel_publisher.c (008), calibration.c (009) added — all source files are present.

# micro-ROS ships its own atomic builtins that duplicate newlib's stdatomic;
# allow both definitions rather than failing at link time.
target_link_options(${COMPONENT_LIB} INTERFACE -Wl,--allow-multiple-definition)
