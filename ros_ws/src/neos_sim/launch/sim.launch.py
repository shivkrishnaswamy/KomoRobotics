import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# 'ros2 launch' looks for a function with exactly this name and calls it
def generate_launch_description():
    pkg = get_package_share_directory('neos_sim')    # Looks up neos_sim and returns /workspace/install/neos_sim/share/neos_sim
    xacro_file = os.path.join(pkg, 'urdf', 'neos_master.urdf.xacro.xml') # Appends urdf/neos_master.urdf.xacro.xml to path
    ekf_config = os.path.join(get_package_share_directory('neos_bringup'), 'config', 'ekf.yaml') # EKF config

    # Runs xacro <path> and keeps what it prints, which is the plain URDF text. ROS runs the command when the launch starts
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Runs cmd 'ign gazebo -r worldname' -r starts the simulation running, and worldname is the world file loaded
    world_file = os.path.join(pkg, 'worlds', 'hallway.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items())

    # Starts the robot_state_publisher program from the package of the same name
    # robot_description: the URDF text. The program publishes it on the /robot_description topic
    # use_sim_time: True: use Gazebo’s clock
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])

    # Reads /robot_description and spawns the robot into Gazebo world
    # -z 0.12: spawn base_link above ground so the robot drops onto the floor
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'neos_sim', '-z', '0.12'])

    # Copies messages between ROS 2 topics and Gazebo topics. [ means Gazebo to ROS, ] means ROS to Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', # /clock: the simulation time
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist', # /cmd_vel: velocity commands. Line that lets your keyboard drive the robot
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry', # /odom: the robot’s estimated position and velocity
            # '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V', # /tf: odom -> base_link transform from the DiffDrive plugin
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model', # /joint_states: wheel joint angles, used by robot_state_publisher
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image', # /camera/image: Colour image
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image', # /camera/depth_image: Depth image, each pixel is a distance in meters
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo', # /camera/camera_info: Camera calibration data
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan', # /scan: 2D lidar scan
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU', # /imu: orientation, angular velocity, linear acceleration
        ])

    # In ROS 2, robot frames are x forward, y left, z up and the camera optical frames are z forward, x right, y down. On a real robot this isn't an issue, 
    # because the camera driver produces data in optical coordinates and the transform adjusts correctly. In Gazebo, the RGB-D sensor computes its point cloud 
    # in Gazebo's own x-forward axes (same as robot), but we label it with the optical frame. So the data arrives saying "I'm in camera_optical_link" while its 
    # numbers are actually in x-forward (robot frame) axes. RViz trusts the label, applies the optical frame's rotation, and the cloud appears rotated 90°. 
    # Our fix ignores Gazebo's cloud and rebuilds it from the depth image with depth_image_proc, which produces real optical-frame coordinates, so the numbers match the label.
    
    depth_to_points = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image_rect', '/camera/depth_image'),   # input: depth image
                    ('camera_info', '/camera/camera_info'),  # input: calibration
                    ('points', '/camera/points'),            # output: point cloud
                ]),
        ])

    # Describes the EKF program 'ekf_node' inside the ROS package 'robot_localization'
    # Gives the running node the name 'ekf_filter_node', and tells it to print to the screen
    # EKF fuses /odom + /imu into /odometry/filtered and publishes the odom -> base_link TF.
    # name must match the top-level key in ekf.yaml
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}])

    # Starts a WebSocket server that exposes all ROS 2 topics to Foxglove Studio
    # Connect from Foxglove Studio with ws://localhost:8765 (port is published in docker-compose.yml)
    # address 0.0.0.0: listen on all interfaces so the host can reach it from outside the container
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True, 'topic_whitelist': ['/tf', '/tf_static', '/odom', '/robot_description', '/camera/camera_info', '/scan', '/cmd_vel',
                                                                                                    '/imu', '/initialpose', '/joint_states']}])

    # Reads the same SDF world Gazebo loads and publishes it as a MarkerArray on /world_markers,
    # so the walls/objects show up in Foxglove's 3D panel. Gazebo's world origin is where the robot spawned,
    # so the markers are published in the 'odom' frame. ground_plane is skipped since Foxglove already draws a grid
    world_markers = Node(
        package='neos_sim',
        executable='sdf_to_markers.py',
        name='sdf_to_markers',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'frame_id': 'odom',
            'exclude_models': ['ground_plane'],
            'use_sim_time': True,
        }])

    return LaunchDescription([gazebo, rsp, spawn, bridge, depth_to_points, ekf, foxglove, world_markers]) # Hands ros2 launch the list of the programs above and starts all of them