cmake_minimum_required(VERSION 3.8)
project(ecza_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, and config directories so they are
# available via ament resource index at runtime.
install(DIRECTORY
  urdf
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
