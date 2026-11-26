/*
  This node adds noise and error, and covariance matrices to the odometry and imu messages for the simulation  

  Sits between the Gazebo bridge and everything else:
      /sim/odom_raw -> [this node] -> /odom
      /sim/imu_raw  -> [this node] -> /imu
*/

#include <array>
#include <cmath>
#include <memory>
#include <random>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;

// +0.02 = odom assumes a wheel radius 2% bigger than the real one and so over-reports distance by 2%.
// This stays inside slam_toolbox's 0.5 m scan-match window between nodes but adds up enough to need loop closure.
const double wheelRadiusError = 0.02;
// +0.04 = odom assumes a track width 4% wider so it under-reports turning by ~2% (combined with the radius error).
// Without the gyro that is ~7 deg of heading error per full turn; the EKF should mostly remove it.
const double wheelSeparationError = 0.04;

// Speed multipliers which add the error from the error variables above
const double linXScale = 1.0 + wheelRadiusError;
const double angZScale = (1.0 + wheelRadiusError) / (1.0 + wheelSeparationError);

// Standard deviations for odometry twist covariance matrix construction, sized to cover the errors above at our speed limits
const double odomVxStddev = 0.02;    // m/s
const double odomVyStddev = 0.001;   // m/s, very tight since diff drive can't slide sideways, so the measured 0 is a hard constraint
const double odomVyawStddev = 0.05;  // rad/s, loose as turning is where wheels slip most, so the gyro should dominate yaw rate

// Standard deviation for imu angular vel covariance matrix. Must match the noise Gazebo adds in neos_desc/urdf/neos_imu.urdf.xacro.xml
const double imuGyroStddev = 0.00063; // rad/s, white noise from the datasheet

class SimOdometryNoise : public rclcpp::Node
{
  private:
    array<double, 36> twistCov{}; // Covariance matrix for odometry twist
    array<double, 9> gyroCov{};   // Covariance matrix for imu angular vel
    mt19937 rng;  // Random number generator for slip

    // pub and sub objects for odometry and imu
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

    // Helper that makes 6x6 matrix with diagonal of values in covVals array
    array<double, 36> diag6(array<double, 6> covVals)
    {
      array<double, 36> cov{};
      for (size_t i = 0; i < 6; ++i)
      {
        cov[i * 6 + i] = covVals[i];
      }
      return cov;
    }

    // Helper that makes 3x3 matrix with diagonal of covVal
    array<double, 9> diag3(double covVal)
    {
      array<double, 9> cov{};
      for (size_t i = 0; i < 3; ++i)
      {
        cov[i * 3 + i] = covVal;
      }
      return cov;
    }

    // Adds Gaussian noise to speed to emulate wheel slip
    double addSlip(double speed)
    {
      // Don't add slip if the robot isnt moving (stationary wheels don't slip)
      if (speed == 0.0)
        return speed;
      // Create bell curve with mean at 0 and stdev of 1% of robots speed
      normal_distribution<double> dist(0.0, 0.01 * abs(speed));
      // return the speed plus the slip (random num on the distribution)
      return speed + dist(rng);
    }

    // Function that adds error and covariance to the raw odometry message
    void onOdom(nav_msgs::msg::Odometry::UniquePtr msg)
    {
      // Get the forward velocity from the message, scale it for error, then add slip
      const double lin_x_mod = addSlip(msg->twist.twist.linear.x * linXScale);
      // Get the angular velocity around vertical axis (turn rate), and do the same
      const double ang_z_mod = addSlip(msg->twist.twist.angular.z * angZScale);
      // Reset the forward speed and turn rate with the noisy values
      msg->twist.twist.linear.x = lin_x_mod;
      msg->twist.twist.angular.z = ang_z_mod;
      msg->twist.covariance = twistCov; // Add the covariance
      odomPub->publish(move(msg));      // Publish the new odom message
    }

    // Function that sets imu covariance matrix for the raw imu message
    void onImu(sensor_msgs::msg::Imu::UniquePtr msg)
    {
      // REP-145: covariance[0] = -1 means "orientation not provided". The EKF doesn't fuse it anyway
      msg->orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      msg->angular_velocity_covariance = gyroCov; // Add gyro covariance
      imuPub->publish(move(msg));                 // Publish the new imu message
    }

  public:
    SimOdometryNoise() : Node("sim_odometry_noise") // Name the node gets
    {
      rng.seed(random_device{}()); // Seed random num gen at a random value

      // Set odometry twist covariance
      twistCov = diag6({odomVxStddev * odomVxStddev, odomVyStddev * odomVyStddev, 1e-6, 1e-6, 1e-6, odomVyawStddev * odomVyawStddev});
      // Set imu angular vel covariance
      gyroCov = diag3(imuGyroStddev * imuGyroStddev);

      // Initialize odom and imu publishers to the topic they publish
      odomPub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      imuPub = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
      // Initialize odom and imu subscribers to the topics they listen to. Using lambda and to call onOdom/onImu when respective msg arrives
      odomSub = create_subscription<nav_msgs::msg::Odometry>("/sim/odom_raw", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) {onOdom(move(msg));});
      imuSub = create_subscription<sensor_msgs::msg::Imu>("/sim/imu_raw", 10, [this](sensor_msgs::msg::Imu::UniquePtr msg) {onImu(move(msg));});
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                    // start ROS
  rclcpp::spin(make_shared<SimOdometryNoise>()); // create the node (constructor) and run it
  rclcpp::shutdown();                          // clean up after 
  return 0;
}