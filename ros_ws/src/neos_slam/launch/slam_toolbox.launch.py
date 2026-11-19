import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts slam_toolbox 2D lidar SLAM. Shared by the sim and the real robot
# Uses the lidar only, needs /scan and the odom -> base_link TF (EKF)

# slam_toolbox.yaml holds the shared config, but the slam_toolbox parameters use_sim_time, mode, map_file_name and map_start_at_dock
# are set here because they depend on launch args and sim vs real robot differences

SIM_MAP = '/workspace/maps/neos_sim'
REAL_MAP = '/workspace/maps/neos_real'

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    filter_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'laser_filters.yaml') # Get path for lidar filter config file
    slam_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'slam_toolbox.yaml')    # Get path for SLAM config file

    sim = is_true(LaunchConfiguration('sim').perform(context))   # true = running in Gazebo sim, false = real robot
    slam = is_true(LaunchConfiguration('slam').perform(context)) # true = full SLAM (map + localize), false = load saved map and only localize

    # Where the map is saved. Fixed, not a launch argument, so the sim and real maps always land in the same known files
    map_file = SIM_MAP if sim else REAL_MAP
    os.makedirs(os.path.dirname(map_file), exist_ok=True) # Create folder for map files (serialize_map won't create it)

    if slam:
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
            'map_file_name': map_file, # Saved pose graph to load
            'map_start_at_dock': True, # Start at the pose where mapping began, so put the robot back there before launching
        }

    # Crops /scan (360 deg) to the front +-85 deg as /scan_filtered, because wider beams hit the robot's own channel walls
    lidar_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': sim}],
        remappings=[
            ('scan', '/scan'),                   # input: raw 360 deg scan
            ('scan_filtered', '/scan_filtered'), # output: scan cropped to front
        ]
    )

    # The SLAM node. Publishes /map and the map -> odom TF
    slam_toolbox = Node(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, mode_params, {'use_sim_time': sim}], # Order matters: later entries override earlier ones
    )

    nodes = [lidar_filter, slam_toolbox]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Are you running SLAM in the Gazebo sim or on the real robot'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Do you want full SLAM or just localization on an existing map'),
        OpaqueFunction(function=launch_setup),
    ])