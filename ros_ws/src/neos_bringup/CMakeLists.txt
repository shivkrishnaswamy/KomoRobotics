cmake_minimum_required(VERSION 3.8)

project(neos_bringup)              # Must match <name> in package.xml
find_package(ament_cmake REQUIRED)

# Copy the config folder (ekf.yaml, ...) into DEST
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()