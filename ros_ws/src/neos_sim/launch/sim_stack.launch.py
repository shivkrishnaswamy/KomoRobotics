import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

# Entry point for the whole sim. Shared launch files (also used on the real robot) don't set use_sim_time,
# so it is set once here and applies to every node in every launch file included below
def include(package, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', launch_file)))

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True), # Must come before the includes, it only applies to nodes after it
        include('neos_sim', 'sim.launch.py'),          # Gazebo, robot model, bridge, camera point cloud
        include('neos_ekf', 'ekf.launch.py'),          # Fuses /odom + /imu, publishes odom -> base_link
        include('neos_lidar', 'lidar_filter.launch.py'), # Filters /scan to to isolate scan portion not hitting the robot interior
    ])