cmake_minimum_required(VERSION 3.8)

project(neos_sim)                  # Names the CMake project neos_sim (must match <name> in package.xml). Also sets variable called ${PROJECT_NAME} to neos_sim
find_package(ament_cmake REQUIRED) # Loads the ament_cmake tools which are ROS 2’s extra commands for CMake

# Copy the whole urdf folder (contains all xacro files) into DEST
install(
  DIRECTORY ../neos_desc/urdf
  DESTINATION share/${PROJECT_NAME}
)

# Copy the launch and worlds (gazebo sim world) folder into DEST
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install python nodes as executables into lib/neos_sim so 'ros2 run neos_sim <script>' can find them
install(
  PROGRAMS scripts/sdf_to_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() # Generates the metadata that makes ROS recognize neos_sim as a package