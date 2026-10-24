import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 'ros2 launch' looks for a function with exactly this name and calls it
def generate_launch_description():
    pkg = get_package_share_directory('komo_robotics_sim')    # Looks up komo_robotics_sim and returns /workspace/install/komo_robotics_sim/share/komo_robotics_sim
    xacro_file = os.path.join(pkg, 'urdf', 'komo.urdf.xacro') # Appends urdf/komo_urdf.xacro to path

    # Runs xacro <path> and keeps what it prints, which is the plain URDF text. ROS runs the command when the launch starts
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Runs cmd 'ign gazebo -r empty.sdf' -r starts the simulation running, and empty.sdf is a built-in world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    # Starts the robot_state_publisher program from the package of the same name
    # robot_description: the URDF text. The program publishes it on the /robot_description topic
    # use_sim_time: True: use Gazebo’s clock
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])

    # Reads /robot_description and spawns the robot into Gazebo world
    # -z 0.2: spawn 0.2 m above the ground, so the robot drops onto the floor
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'komo_robot', '-z', '0.2'])

    # Copies messages between ROS 2 topics and Gazebo topics
    # /clock: the simulation time, from Gazebo to ROS ([)
    # /cmd_vel: velocity commands, from ROS to Gazebo (]). Line that lets your keyboard drive the robot
    # /odom: the robot’s estimated position, from Gazebo to ROS ([)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ])

    return LaunchDescription([gazebo, rsp, spawn, bridge]) # Hands ros2 launch the list of the four programs above and starts all of them
