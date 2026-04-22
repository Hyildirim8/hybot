cmake_minimum_required(VERSION 3.8)
project(ecza_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, config, and scripts directories so they are
# available via ament resource index at runtime.
install(DIRECTORY
  urdf
  launch
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install wheel_bridge.py as an executable node
install(PROGRAMS
  scripts/wheel_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
