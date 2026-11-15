import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts RTAB-Map SLAM. Shared by the sim and the real robot: it only needs the camera topics, /scan, and the
# odom -> base_link TF from the EKF to already exist

def is_true(value):
    return value.lower() in ('true', '1', 'yes')

def launch_setup(context):
    rtabmap_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'rtabmap.yaml') # Get path for SLAM config file

    use_sim_time = is_true(LaunchConfiguration('use_sim_time').perform(context)) # To use Gazebo clock for sim
    localization = is_true(LaunchConfiguration('localization').perform(context)) # To freeze map and only localize robot
    new_map = is_true(LaunchConfiguration('new_map').perform(context))           # To delete saved map at startup
    database_path = os.path.expanduser(LaunchConfiguration('database_path').perform(context)) # Where map is saved
    show_viz = is_true(LaunchConfiguration('rtabmap_viz').perform(context))      # To open rtabmap_viz GUI

    os.makedirs(os.path.dirname(database_path), exist_ok=True) # Create folder for database file

    mode_params = { # RTAB-Map parameters
        'Mem/IncrementalMemory': 'false' if localization else 'true',  # Add new snapshots to map
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false', # Load the whole saved mpa into memory at startup
    }

    # '-d' (--delete_db_on_start) wipes the database. For new map and you're not localizing
    rtabmap_args = ['-d'] if (new_map and not localization) else []

    # Bundles the three camera topics into one message so RTAB-Map gets colour, depth and calibration from the same instant.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync', # name the running node gets
        output='screen',
        # approx sync is to pair messages whose timestamps are close but not exact. Real camera messages from the three topics can be slightly apart
        parameters=[{'use_sim_time': use_sim_time, 'approx_sync': True, 'sync_queue_size': 10}],
        remappings=[ # connect topics RTAB-Map expects with actual
            ('rgb/image', '/camera/image'),             # input: colour image
            ('depth/image', '/camera/depth_image'),     # input: depth image (meters)
            ('rgb/camera_info', '/camera/camera_info'), # input: calibration
            ('rgbd_image', '/camera/rgbd_image'),       # output: the bundle
        ])

    # The SLAM node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap', # nodes outputs prefixed with /rtabmap, # except /map which is remapped to the standard name Nav2 expects
        output='screen',
        parameters=[rtabmap_config, mode_params, {'use_sim_time': use_sim_time, 'database_path': database_path}],
        arguments=rtabmap_args,
        remappings=[
            ('rgbd_image', '/camera/rgbd_image'),
            ('scan', '/scan'),
            ('map', '/map'),
        ])

    nodes = [rgbd_sync, rtabmap]

    # Optional GUI. Shows the camera with detected features, the graph with loop closures, and the 3D map
    if show_viz:
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
            }],
            remappings=[
                ('rgbd_image', '/camera/rgbd_image'),
                ('scan', '/scan'),
            ]))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true in sim (use Gazebo /clock), false on the real robot'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='false = build/extend the map, true = only localize in an existing map'),
        DeclareLaunchArgument('new_map', default_value='false',
                              description='true deletes the database at database_path on start. Ignored in localization mode'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Where the map (graph, images, scans) is saved'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true',
                              description='Open the RTAB-Map GUI (shows the graph, loop closures, 3D map)'),
        OpaqueFunction(function=launch_setup),
    ])