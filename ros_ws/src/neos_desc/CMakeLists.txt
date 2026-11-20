cmake_minimum_required(VERSION 3.8)

project(neos_desc)                 # Must match <name> in package.xml
find_package(ament_cmake REQUIRED)

# Copy the launch and urdf folders into DEST
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()