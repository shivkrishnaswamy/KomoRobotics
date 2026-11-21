# KomoRobotics
The Komo Robotics repository containing all the software for production, simulation, and test

## Build Steps
Run the following commands (within a Linux terminal) to ...

### Build the Docker container, launch it, and build ROS2 software
```
xhost +local:docker
cd docker
docker compose up -d --build
docker compose exec ros2_gazebo bash
colcon build --symlink-install
source install/setup.bash
```

### For each new terminal, run
```
docker compose exec ros2_gazebo bash
source install/setup.bash
```

### Launch Gazebo
```
ros2 launch neos_sim sim.launch.py
```
This also launches the sim, EKF, lidar-crop, and robot state publisher

### Open teleop keyboard to drive robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Open RViz
```
rviz2
```

### Run SLAM
#### Full SLAM (mapping)
```
ros2 launch neos_slam slam_toolbox.launch.py map:=true
```

#### Save the map before stopping mapping
```
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/workspace/maps/toolbox/map'}"
```

#### Just localization
```
ros2 launch neos_slam slam_toolbox.launch.py map:=false
```

## Reference Documentation
### ros_ws/src
This is where all the ROS packages for the robotic software are located.

A **package** is the basic unit ROS uses to organize, build, and find code. It's really a folder with a package.xml at its root, which makes colcon and ros2 treat the folder as a package. Each package is focused on one specific aspect or component of the robot (e.g., LiDAR, camera, IMU, SLAM, etc.).
```
neos_example/        # The example package
├── package.xml      # name, version, dependencies (build/exec), build type
├── CMakeLists.txt   # build and install rules (ament_cmake), or setup.py for Python
├── launch/          # launch files
├── config/          # YAML parameter files
└── other/
```
- **package.xml**: not only declares the directory as a package, but also defines the package name, version, all of its dependencies, and build type.

- **CMakeLists.txt**: holds the build instructions for the package. _colcon build_ runs _CMake_ on it for each package, and it says what to compile, where to install the results, and what to copy over. It also lets you launch files within a package by package name instead of file path.

- **config**: directory for any configuration parameters the nodes or processes within the package rely on.

- **launch**: directory which contains launch files to start the nodes related to the package.

- **other**: placeholder that could contain other package-specific files and/or C++ **nodes** themselves.

### firmware