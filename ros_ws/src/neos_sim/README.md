# Simulation Reference Document
The komo_sim directory contains the files involved in setting up the Gazebo simulation environment

## File Descriptions
#### package.xml
All this file does is essentially mark the directory that it is in as a package, and list other packages it depends on. This is needed to allow colcon to recognize it as something to build.\
The package has a name which is defined within the xml.

#### CMakeLists.txt
This file tells the build to copy certain files and folders into the install directory which is where ROS searches.\
Allows us to run `ros2 launch komo_robotics_sim sim.launch.py` by package name instead of by file path, and lets other files be located more easily.

#### worlds/
This directory contains different worlds we can load into the Gazebo simulation to test the robot in.\
If you want to use a different world (either another one within worlds/ or a default already within Gazebo like empty.sdf), it needs to be specified in the gazebo program def within sim.launch.py (line 22).

#### launch/
Contains the launch file used to launch/start the simulation. Technically, the sim can be started without this file by typing all the commands with all the correct file paths into the cmdline manually, but using this file makes it easier (reproducibility) and allows us to do config in one place.

##### Other Notes
`ament_cmake` is the official, CMake-based build system for C and C++ packages in ROS 2.\
colcon creates three directories by itself: build/ install/ and log/. Necessary files for the simulation are copied into here (by CMakeLists).  