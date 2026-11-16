# Bringup Reference Document
The komo_sim directory contains the files involved in setting up the Gazebo simulation environment

## File Descriptions
#### _slam.launch.py_ launch configurations
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
