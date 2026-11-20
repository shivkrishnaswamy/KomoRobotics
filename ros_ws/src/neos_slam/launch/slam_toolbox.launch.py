import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts slam_toolbox 2D lidar SLAM. Shared by the sim and the real robot
# Uses the lidar only, needs /scan_filtered (start lidar_filter.launch.py first) and the odom -> base_link TF (EKF)
# slam_toolbox.yaml holds the shared config, but the slam_toolbox parameters mode, map_file_name and map_start_at_dock
# are set here because they depend on the map launch arg

MAP = '/workspace/maps/toolbox/map'

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    mapping = is_true(LaunchConfiguration('map').perform(context)) # true = full SLAM (map + localize), false = load saved map and only localize
    slam_config = os.path.join(get_package_share_directory('neos_slam'), 'config', 'slam_toolbox.yaml')    # Get path for SLAM config file
    os.makedirs(os.path.dirname(MAP), exist_ok=True) # Create folder for map files (serialize_map won't create it)

    if mapping:
        # async processes the newest scan and skips ones that arrive while it's busy, so it never falls behind real time
        # (sync would queue every scan and lag on a slow computer)
        executable = 'async_slam_toolbox_node'
        mode_params = { # slam_toolbox parameters
            'mode': 'mapping', # Build a new map
        }
    else:
        executable = 'localization_slam_toolbox_node'
        mode_params = { # slam_toolbox parameters
            'mode': 'localization',    # Only track the robot's pose in the saved map, don't add to it
            'map_file_name': MAP,      # Saved pose graph to load
            'map_start_at_dock': True, # Start at the pose where mapping began, so put the robot back there before launching
        }

    # The SLAM node. Publishes /map and the map -> odom TF
    slam_toolbox = Node(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, mode_params], # Order matters: later entries override earlier ones
    )

    nodes = [slam_toolbox]
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='true',
                              description='Do you want full SLAM or just localization on an existing map'),
        OpaqueFunction(function=launch_setup),
    ])