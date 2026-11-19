import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_config = os.path.join(get_package_share_directory('neos_ekf'), 'config', 'ekf.yaml') # EKF config

    # Describes the EKF program 'ekf_node' inside the ROS package 'robot_localization'
    # Gives the running node the name 'ekf_filter_node', and tells it to print to the screen
    # EKF fuses /odom + /imu into /odometry/filtered and publishes the odom -> base_link TF.
    # name must match the top-level key in ekf.yaml
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config])

    return LaunchDescription([ekf])