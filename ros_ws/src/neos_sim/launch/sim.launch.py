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
    xacro_file = os.path.join(pkg, 'urdf', 'neos.urdf.xacro.xml') # Appends urdf/neos.urdf.xacro.xml to path

    # Runs xacro <path> and keeps what it prints, which is the plain URDF text. ROS runs the command when the launch starts
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Runs cmd 'ign gazebo -r worldname' -r starts the simulation running, and worldname is the world file loaded
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {os.path.join(pkg, "worlds", "hallway.sdf")}'}.items())

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
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image', # /camera/image: Colour image
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image', # /camera/depth_image: Depth image, each pixel is a distance in meters
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo', # /camera/camera_info: Camera calibration data
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

    return LaunchDescription([gazebo, rsp, spawn, bridge, depth_to_points]) # Hands ros2 launch the list of the programs above and starts all of them