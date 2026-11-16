import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts the lidar filter on its own: raw /scan in, /scan_filtered out.

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    laser_filter_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'laser_filters.yaml')

    sim = is_true(LaunchConfiguration('sim').perform(context)) # true = running in Gazebo sim, false = real robot

    # Crops the 360 deg lidar to the front arc so beams hitting the robot's body are dropped
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filter_config, {'use_sim_time': sim}],
        remappings=[
            ('scan', '/scan'),                   # input: raw 360 deg scan
            ('scan_filtered', '/scan_filtered'), # output: front arc only
        ]
    )

    return [scan_filter]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Are you running in the Gazebo sim or on the real robot'),
        OpaqueFunction(function=launch_setup),
    ])