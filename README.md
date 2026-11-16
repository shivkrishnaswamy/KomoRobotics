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
`ros2 launch neos_bringup lidar_filter.launch.py sim:=true` Sim\
`ros2 launch neos_bringup lidar_filter.launch.py` Real robot

#### SLAM
FOR SIMULATION ROBOT\
`ros2 launch neos_bringup slam.launch.py sim:=true new_map:=true` SLAM new sim world from scratch (new map)\
`ros2 launch neos_bringup slam.launch.py sim:=true` SLAM but continue mapping the sim world (from saved map)\
`ros2 launch neos_bringup slam.launch.py sim:=true slam:=false` Localize in the finished sim map

FOR REAL ROBOT\
`ros2 launch neos_bringup slam.launch.py new_map:=true` SLAM new real space from scratch\
`ros2 launch neos_bringup slam.launch.py` SLAM but continue mapping the real space (from saved map)\
`ros2 launch neos_bringup slam.launch.py slam:=false` Localize in the finished real map

Can add `viz:=false` to turn off rtabmap gui\
Note: Supplying launch arguments `slam:=false new_map:=true` at the same time either in sim or real is pointless (can't localize in a blank map)
