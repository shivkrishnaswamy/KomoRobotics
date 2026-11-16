# KomoRobotics
The Komo Robotics repository containing all the software for production, simulation, and test

## Simulation Build Steps
_Note: The following steps work are meant for a Linux OS. Mac and Windows versions coming soon..._

Run the following commands to ...

_Build the Docker container, launch it, and build ROS2 software_
```
xhost +local:docker
cd docker
docker compose up -d --build
docker compose exec ros2_gazebo bash //Run this cmd and the two below for each new terminal
colcon build --symlink-install
source install/setup.bash
```
_Launch Gazebo_
```
ros2 launch neos_sim sim.launch.py
```

_Open Keyboard to Drive Robot_
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

_SLAM_
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
