cmake_minimum_required(VERSION 3.8)

project(komo_robotics_sim)         # Names the CMake project komo_robotics_sim (must match <name> in package.xml). Also sets variable called ${PROJECT_NAME} to komo_robotics_sim
find_package(ament_cmake REQUIRED) # Loads the ament_cmake tools which are ROS 2’s extra commands for CMake

# Copy the xacro file into DEST directory
install(
  FILES ros_ws/src/komo_desc/urdf/komo.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Copy the launch folder into DEST
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() # Generates the metadata that makes ROS recognize komo_robotics_sim as a package