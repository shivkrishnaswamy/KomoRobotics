import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts RTAB-Map SLAM. Shared by the sim and the real robot

# rtabmap.yaml holds the shared config, but the 4 RTAB-Map parameters use_sim_time, database_path, Mem/IncrementalMemory, 
# and Mem/InitWMWithAllNodes are set here because they depend on launch args and sim vs real robot differences

SIM_DATABASE = '/workspace/maps/neos_sim.db'
REAL_DATABASE = '/workspace/maps/neos_real.db'

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    rtabmap_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'rtabmap.yaml') # Get path for SLAM config file
    laser_filter_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'laser_filters.yaml') # Lidar filter chain

    sim = is_true(LaunchConfiguration('sim').perform(context))         # true = running in Gazebo sim, false = real robot
    slam = is_true(LaunchConfiguration('slam').perform(context))       # true = full SLAM (map + localize), false = freeze map and only localize
    new_map = is_true(LaunchConfiguration('new_map').perform(context)) # true = delete saved map at startup, false = keep the saved map
    viz = is_true(LaunchConfiguration('viz').perform(context))         # true = open rtabmap_viz GUI, false = no GUI

    # Where the map is saved. Fixed, not a launch argument, so the sim and real maps always land in the same known files
    database_path = SIM_DATABASE if sim else REAL_DATABASE
    os.makedirs(os.path.dirname(database_path), exist_ok=True) # Create folder for database file

    mode_params = { # RTAB-Map parameters
        'Mem/IncrementalMemory': 'true' if slam else 'false',  # Add new snapshots to map (SLAM only)
        'Mem/InitWMWithAllNodes': 'false' if slam else 'true', # Load the whole saved map into memory at startup (localization only)
    }

    rtabmap_params = [rtabmap_config, mode_params, {'use_sim_time': sim, 'database_path': database_path}]  # Order matters: later entries override earlier ones
    rtabmap_args = ['-d'] if (new_map and slam) else [] # '-d' (--delete_db_on_start) wipes the database (for new map in SLAM mode)

    # Crops the 360 deg lidar to the front arc so beams hitting the robot's own channel walls never reach SLAM.
    # Raw /scan in, /scan_filtered out
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

    # Bundles the three camera topics into one message so RTAB-Map gets colour, depth and calibration from the same instant
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync', # name the running node gets
        output='screen',
        # approx sync is to pair messages whose timestamps are close but not exact. Real camera messages from the three topics can be slightly apart
        parameters=[{'use_sim_time': sim, 'approx_sync': True, 'sync_queue_size': 10}],
        remappings=[ # connect topics RTAB-Map expects with actual
            ('rgb/image', '/camera/image'),             # input: colour image
            ('depth/image', '/camera/depth_image'),     # input: depth image (meters)
            ('rgb/camera_info', '/camera/camera_info'), # input: calibration
            ('rgbd_image', '/camera/rgbd_image'),       # output: the bundle
        ]
    )

    # The SLAM node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap', # nodes outputs prefixed with /rtabmap, # except /map which is remapped to the standard name Nav2 expects
        output='screen',
        parameters=rtabmap_params,
        arguments=rtabmap_args,
        remappings=[
            ('rgbd_image', '/camera/rgbd_image'),
            ('scan', '/scan_filtered'),
            ('map', '/map'),
        ]
    )

    nodes = [scan_filter, rgbd_sync, rtabmap]

    # Optional GUI. Shows the camera with detected features, the graph with loop closures, and the 3D map
    if viz:
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': sim,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
            }],
            remappings=[
                ('rgbd_image', '/camera/rgbd_image'),
                ('scan', '/scan_filtered'),
            ]
        ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Are you running SLAM in the Gazebo sim or on the real robot'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Do you want full SLAM or just localization on an existing map'),
        DeclareLaunchArgument('new_map', default_value='false',
                              description='Do you want to delete the saved map when running SLAM'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Do you want the RTAB-Map GUI'),
        OpaqueFunction(function=launch_setup),
    ])