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

#### Open teleop keyboard to drive robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Open RViz
```
rviz2
```

#### Lidar filter (crops /scan to the front of the robot as /scan_filtered, required by SLAM)
`ros2 launch neos_lidar lidar_filter.launch.py sim:=true` Sim\
`ros2 launch neos_lidar lidar_filter.launch.py` Real robot

#### SLAM
`ros2 launch neos_slam rtabmap.launch.py wipe_map:=true` SLAM from scratch (new map)\
`ros2 launch neos_slam rtabmap.launch.py` SLAM but continue mapping (from saved map)\
`ros2 launch neos_slam rtabmap.launch.py map:=false` Localize in the finished map

Can add `viz:=false` to turn off rtabmap gui\
Note: Supplying launch arguments `map:=false wipe_map:=true` at the same time is pointless (can't localize in a blank map)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/workspace/maps/neos'}"
