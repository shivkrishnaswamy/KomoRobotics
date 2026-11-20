# KomoRobotics
The Komo Robotics repository containing all the software for production, simulation, and test

## Build Steps
Run the following commands (within a Linux terminal) to ...

#### Build the Docker container, launch it, and build ROS2 software
```
xhost +local:docker
cd docker
docker compose up -d --build
docker compose exec ros2_gazebo bash
colcon build --symlink-install
source install/setup.bash
```

#### For each new terminal, run
```
docker compose exec ros2_gazebo bash
source install/setup.bash
```

#### Launch Gazebo
```
ros2 launch neos_sim sim.launch.py
```
This also launches the sim, EKF, lidar-crop, and robot state publisher

#### Open teleop keyboard to drive robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Open RViz
```
rviz2
```

#### Run SLAM
Full SLAM (mapping)
```
ros2 launch neos_slam slam_toolbox.launch.py map:=true
```

Save the map before stopping mapping
```
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/workspace/maps/toolbox/map'}"
```

Just localization
```
ros2 launch neos_slam slam_toolbox.launch.py map:=false
```