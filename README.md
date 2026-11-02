# KomoRobotics
The Komo Robotics repository containing all the software for production, simulation, and test

## Simulation Build Steps
_Note: The following steps work are meant for a Linux OS. Mac and Windows versions coming soon..._

Run the following commands
```
xhost +local:docker
cd docker
docker compose up -d --build
docker compose exec ros2_gazebo bash
colcon build --symlink-install
source install/setup.bash
ros2 launch komo_robotics_sim sim.launch.py
```
In a secondary terminal, run these commands
```
docker compose exec ros2_gazebo bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```