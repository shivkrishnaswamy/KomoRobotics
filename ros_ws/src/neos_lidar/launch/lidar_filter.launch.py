import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Starts the lidar filter on its own: raw /scan in, /scan_filtered out.

def generate_launch_description():
    laser_filter_config = os.path.join(get_package_share_directory('neos_lidar'), 'config', 'laser_filters.yaml')

    # Crops the 360 deg lidar to the front arc so beams hitting the robot's body are dropped
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='lidar_crop',
        output='screen',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan'),                   # input: raw 360 deg scan
            ('scan_filtered', '/scan_filtered'), # output: front arc only
        ]
    )

    return LaunchDescription([scan_filter])