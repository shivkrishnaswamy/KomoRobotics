import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts RTAB-Map SLAM. Shared by the sim and the real robot
# Needs /scan_filtered, so start lidar_filter.launch.py first

# rtabmap.yaml holds the shared config, but the 3 RTAB-Map parameters database_path, Mem/IncrementalMemory, 
# and Mem/InitWMWithAllNodes are set here because they depend on launch args

MAP = '/workspace/maps/rtab/map.db'

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    mapping = is_true(LaunchConfiguration('map').perform(context))     # true = full SLAM (map + localize), false = freeze map and only localize
    wipe_map = is_true(LaunchConfiguration('wipe_map').perform(context)) # true = delete saved map at startup, false = continue mapping saved map
    viz = is_true(LaunchConfiguration('viz').perform(context))         # true = open rtabmap_viz GUI, false = no GUI

    rtabmap_config = os.path.join(get_package_share_directory('neos_slam'), 'config', 'rtabmap.yaml') # Get path for SLAM config file
    os.makedirs(os.path.dirname(MAP), exist_ok=True) # Create folder for database file

    mode_params = { # RTAB-Map parameters
        'Mem/IncrementalMemory': 'true' if mapping else 'false',  # Add new snapshots to map (SLAM only)
        'Mem/InitWMWithAllNodes': 'false' if mapping else 'true', # Load the whole saved map into memory at startup (localization only)
    }
    rtabmap_params = [rtabmap_config, mode_params, {'database_path': MAP}]  # Order matters: later entries override earlier ones
    rtabmap_args = ['-d'] if (wipe_map and mapping) else [] # '-d' (--delete_db_on_start) wipes the database (for new map in SLAM mode)

    # Bundles the three camera topics into one message so RTAB-Map gets colour, depth and calibration from the same instant
    camera_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='camera_sync', # name the running node gets
        output='screen',
        # approx sync is to pair messages whose timestamps are close but not exact. Real camera messages from the three topics can be slightly apart
        parameters=[{'approx_sync': True, 'sync_queue_size': 10}],
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

    nodes = [camera_sync, rtabmap]

    # Optional GUI. Shows the camera with detected features, the graph with loop closures, and the 3D map
    if viz:
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[{
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
        DeclareLaunchArgument('map', default_value='true',
                              description='Do you want full SLAM or just localization on an existing map'),
        DeclareLaunchArgument('wipe_map', default_value='false',
                              description='Do you want to delete the saved map when running SLAM'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Do you want the RTAB-Map GUI'),
        OpaqueFunction(function=launch_setup),
    ])