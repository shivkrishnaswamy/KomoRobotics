cmake_minimum_required(VERSION 3.8)

project(neos_sim)                  # Names the CMake project neos_sim (must match <name> in package.xml). Also sets variable called ${PROJECT_NAME} to neos_sim
find_package(ament_cmake REQUIRED) # Loads the ament_cmake tools which are ROS 2’s extra commands for CMake
find_package(rclcpp REQUIRED)      # Libraries the sim_odometry_noise node links against
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Compile the sim odometry noise node
add_executable(sim_odometry_noise src/sim_odometry_noise.cpp)
target_compile_features(sim_odometry_noise PUBLIC cxx_std_17)
ament_target_dependencies(sim_odometry_noise rclcpp nav_msgs sensor_msgs)

# Copy the launch and worlds (gazebo sim world) folders into DEST
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Copy the compiled node to lib/<pkg> where ros can find it
install(
  TARGETS sim_odometry_noise
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() # Generates the metadata that makes ROS recognize neos_sim as a package