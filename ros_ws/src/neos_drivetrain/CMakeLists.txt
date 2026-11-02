cmake_minimum_required(VERSION 3.8)

project(neos_drivetrain) # Must match <name> in package.xml

# Turn on extra compiler warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)   # Loads the ament_cmake tools which are ROS 2’s extra commands for CMake
find_package(rclcpp REQUIRED)        # ROS 2 C++ client library (nodes, publishers, subscribers, timers)
find_package(geometry_msgs REQUIRED) # Provides geometry_msgs/msg/Twist message type

# Creates a build target executable called drivetrain_node from .cpp source
add_executable(drivetrain_node src/drivetrain_node.cpp)

# Adds include/ to the compiler's header search path to let the source use #include "neos_drivetrain/drivetrain_node.hpp"
target_include_directories(
  drivetrain_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # path used while compiling
  $<INSTALL_INTERFACE:include>
)

target_compile_features(drivetrain_node PUBLIC cxx_std_17)

# An ament shortcut that links the executable against rclcpp and geometry_msgs
ament_target_dependencies(drivetrain_node rclcpp geometry_msgs)

# Install the executable to lib/neos_drivetrain so `ros2 run neos_drivetrain drivetrain_node` can find it
install(
  TARGETS drivetrain_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers and the parameter file
# only needed if another package wants to #include this class
#install(DIRECTORY include/ DESTINATION include)
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
