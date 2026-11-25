# Explanation of the odometry from wheel encoders, imu, the EKF, and the point of this node

## Understanding encoder odometry
The drive wheels have encoders attached to them that output the number of ticks. Importantly, you cannot tell where the robot is just from each wheel's total tick count. The totals tell you its heading, but its position depends on the order in which the ticks happened, because each bit of wheel motion moves the robot in whatever direction it's facing at that moment. Both wheels could have 200 ticks, and the robot would face the same direction, but could be in a different spot depending on how each encoder got there.

Now in the real robot, we will have a driver that publishes an odometry message containing the robot info from the encoders. In the simulation, Gazebo publishes it for you. The message looks like this ( of type `nav_msgs/msg/Odometry`)
```
nav_msgs/msg/Odometry
│
├── header                          std_msgs/Header
│   ├── stamp                       builtin_interfaces/Time   (sec, nanosec): when the reading was taken
│   └── frame_id                    string: frame the POSE is in → "odom"
│
├── child_frame_id                  string: frame the TWIST is in → "base_link"
│
├── pose                            geometry_msgs/PoseWithCovariance
│   ├── pose                        geometry_msgs/Pose
│   │   ├── position                geometry_msgs/Point
│   │   │   ├── x                   float64   m
│   │   │   ├── y                   float64   m
│   │   │   └── z                   float64   m (0 on a flat floor)
│   │   └── orientation             geometry_msgs/Quaternion
│   │       ├── x, y                float64   (0 on a flat floor)
│   │       ├── z                   float64   sin(yaw/2)
│   │       └── w                   float64   cos(yaw/2)
│   └── covariance                  float64[36]   6×6 row-major: x, y, z, roll, pitch, yaw   (m², rad²)
│
└── twist                           geometry_msgs/TwistWithCovariance
    ├── twist                       geometry_msgs/Twist
    │   ├── linear                  geometry_msgs/Vector3
    │   │   ├── x                   float64   m/s forward
    │   │   ├── y                   float64   m/s sideways (≈0 for diff drive)
    │   │   └── z                   float64   m/s up (0)
    │   └── angular                 geometry_msgs/Vector3
    │       ├── x                   float64   rad/s roll rate (0)
    │       ├── y                   float64   rad/s pitch rate (0)
    │       └── z                   float64   rad/s turn rate (yaw rate)
    └── covariance                  float64[36]   6×6 row-major: vx, vy, vz, vroll, vpitch, vyaw   ((m/s)², (rad/s)²)
```

## Understanding the IMU
The imu publishes data about the robots movements. We will also have a driver for the real robot that publishes the imu message, and in the sim, Gazebo does it for us. The message looks like this (of type `sensor_msgs/msg/Imu`)
```
sensor_msgs/msg/Imu
│
├── header                          std_msgs/Header
│   ├── stamp                       builtin_interfaces/Time   (sec, nanosec): when the reading was taken
│   └── frame_id                    string: frame the IMU data is in → "imu_link"
│
├── orientation                     geometry_msgs/Quaternion: which way the IMU is facing
│   ├── x, y, z, w                  float64
├── orientation_covariance          float64[9]   3×3 row-major: roll, pitch, yaw   (rad²)
│                                   [0] = -1 means "orientation not provided"
│
├── angular_velocity                geometry_msgs/Vector3: GYRO
│   ├── x                           float64   rad/s roll rate
│   ├── y                           float64   rad/s pitch rate
│   └── z                           float64   rad/s turn rate (yaw rate)
├── angular_velocity_covariance     float64[9]   3×3 row-major   ((rad/s)²)
│
├── linear_acceleration             geometry_msgs/Vector3: ACCELEROMETER
│   ├── x                           float64   m/s² forward
│   ├── y                           float64   m/s² left
│   └── z                           float64   m/s² up (≈ +9.8 at rest: gravity is included)
└── linear_acceleration_covariance  float64[9]   3×3 row-major   ((m/s²)²)
```
Also, our imu does not have a magnometer so it does not provide any orientation data.

## Purpose of the EKF
We want to know the robots position as accurately as possible. The odometry driver will end up taking the twist velocity and integrating it over time to get the position estimate which goes into pose. But this is inaccurate and not a good estimate since every bit of error in the turn rate, wheel erros, and all the slip gets factored in. The imu however does give a better reading for the turn rate. So what the EKF does is combine them by subscribing to both of these message topics, and using the covariances, weights each input to produce a more accurate estimate of the robots turn rate, and by extension, position.

From the odometry message, the EKF fuses: linear x (forward vel), linear y (sideways vel (0)), and angular z (turn rate)\
From the imu message, it fuses: angular z

So as you can see, the turn rate is what the EKF blends, and the filter will output a message on the /odometry/filtered topic of type nav_msgs/msg/Odometry

## The point of this sim node
On the real robot, we have to fill in the covariance matrices according to specs. 

#### Odometry
In the sim, the values within each message are perfect with no noise. There is no error, slip, and furthermore, Gazebo doesn't add covariances, so it looks like the message is perfect with full confidence. There is no setting in Gazebo Fortress to add noise either.

#### IMU
On the real robot again, these values aren't perfect and in the sim, Gazebo can be configured to add noise and uncertainty to the measurements within the message it outputs.

#### What the node does
The node subscribes to the raw odometry and imu topics

Each time it receives an odometry message:
- Scales linear x (forward vel) to imitate odometry error from the wheels
- Scales angular z (turn rate) to also add error from odometry
- Adds or subtracts a small amount from both to imitate wheel slip. This amount is chosen at random each time on a bell curve
- Adds a simulated covariance matrix for the twist (velocities)
- Republishes the message (on /odom)

Each time it receives an imu message:
- Adds the covariance matrix for gyro (angular vel)
- Republishes the message (on /imu)
- Remember, we dont need to add imu noise cause Gazebo does that

Because the pose field and the linear acc field aren't used in the EKF or anywhere else, they remain untouched and "perfect" from the sim.