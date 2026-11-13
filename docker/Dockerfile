# 1. Start from the official ROS2 Humble desktop image (built on Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full

# 2. Prevent interactive prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# 3. Registers the OSRF repo with apt so we can install Gazebo Fortress
RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg  \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 4. Install packages
RUN apt-get update && apt-get install -y \
    # Gazebo Fortress
    ignition-fortress \
    # ROS2 gazebo integration bridge
    ros-humble-ros-gz \
    # Converts .xacro to URDF
    ros-humble-xacro \
    # Turns URDF into frame tree RViz displays (prob already in humble)
    ros-humble-robot-state-publisher \
    # Sliders in RViz to turn wheels and check joints
    ros-humble-joint-state-publisher-gui \
    # Drives the robot using keyboard input
    ros-humble-teleop-twist-keyboard \
    # Draws coordinate frame tree
    ros-humble-tf2-tools \
    # Extended Kalman Filter that fuses wheel odometry and IMU
    ros-humble-robot-localization \
    # colcon build command
    python3-colcon-common-extensions \
    # Other useful tools
    bash-completion nano \
    && rm -rf /var/lib/apt/lists/*

# 5. Automatically source ROS2 and Gazebo setups when opening a terminal inside the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 6. Set the default working directory
WORKDIR /workspace
