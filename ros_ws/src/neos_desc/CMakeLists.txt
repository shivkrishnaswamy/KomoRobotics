cmake_minimum_required(VERSION 3.8)

project(neos_desc)                 # Must match <name> in package.xml
find_package(ament_cmake REQUIRED)

# Copy the whole urdf folder (contains all xacro files) into DEST
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()