# KomoRobotics
The Komo Robotics repository containing all the software for production, simulation, and test

### Docker Build Steps
One terminal -> `docker compose run --rm ros2_gazebo` or if the Dockerfile has changed -> `docker compose run --build --rm ros2_gazebo`\
Multiple terminals in same container -> `docker compose up -d` once then `docker compose exec ros2_gazebo bash` in each terminal



docker compose up -d --build
docker compose exec ros2_gazebo bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch komo_robotics_sim sim.launch.py

docker compose exec ros2_gazebo bash
cd /workspace
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard