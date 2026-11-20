import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('neos_desc'), 'urdf', 'neos_master.urdf.xacro.xml') # Appends urdf/neos_master.urdf.xacro.xml to path

    # Runs xacro <path> and keeps what it prints, which is the plain URDF text. ROS runs the command when the launch starts
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Starts the robot_state_publisher program from the package of the same name
    # Turns the URDF into TF so other nodes know where each sensor is mounted: fixed joints (base_link -> lidar_link, camera, imu)
    # go on /tf_static, and wheel joints go on /tf using /joint_states (from Gazebo in sim, from the motor driver on the real robot).
    # SLAM, the EKF and Nav2 all need these to use sensor data, so it runs on the real robot too, not just the sim
    # robot_description: the URDF text. The program publishes it on the /robot_description topic
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}])

    return LaunchDescription([rsp])