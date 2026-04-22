# =============================================================================
# firmware/CMakeLists.txt — ecza-robotu ESP32-S3 firmware
#
# Build system:
#   idf.py build          compile
#   idf.py flash monitor  flash + serial monitor
#
# Components:
#   main/                           — rover firmware application
#   components/micro_ros_espidf_component — micro-ROS for ESP-IDF
# =============================================================================
cmake_minimum_required(VERSION 3.16)

# Register custom components directory (micro-ROS + any future components)
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover_firmware)
