`ament_cmake` is the official, CMake-based build system for C and C++ packages in ROS 2
To make the simulation run and work we need a few files

package.xml
All this file does is essentially mark the folder that it is in as a package and list other packages it depends on
The name is of the package is defined within the xml
Needed to allow colcon to recognize it as something to build

CMakeLists.txt
This file tells the build to copy certain files and folders into the install directory which is where ROS searches
Allows us to run `ros2 launch my_robot sim.launch.py` by package name instead of by file path, and let your launch file locate your xacro with `get_package_share_directory('my_robot')` without hardcoding where it lives


install/komo_robotics_sim/share/komo_robotics_sim/
├── package.xml
├── launch/
│   └── sim.launch.py
└── urdf/
    └── komo_urdf.xacro