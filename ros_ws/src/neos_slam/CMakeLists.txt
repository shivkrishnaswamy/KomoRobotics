cmake_minimum_required(VERSION 3.8)

project(neos_slam)                 # Must match <name> in package.xml
find_package(ament_cmake REQUIRED)

# Copy the config and launch folders into DEST
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()