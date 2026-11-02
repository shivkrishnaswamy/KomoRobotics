#pragma once

#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace neos_drivetrain
{

// Sits between whatever wants to move the robot (teleop, autonomy, etc.) and the drivetrain.
//
//   input_topic (Twist)  -->  [DrivetrainNode]  -->  output_topic (Twist)  -->  drivetrain
//
// The node:
//   - clamps commands to the configured speed limits
//   - republishes the latest command at a fixed rate
//   - sends zero velocity if no command arrives within cmd_timeout (watchdog)
//
// Real vs sim only changes where output_topic goes: in sim, /cmd_vel is bridged to Gazebo's
// DiffDrive plugin; on the real robot, the motor driver should subscribe to the same topic.
class DrivetrainNode : public rclcpp::Node
{
public:
  explicit DrivetrainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Clamp a command and store it as the latest command. It goes out on the next timer tick.
  void send_twist(const geometry_msgs::msg::Twist & cmd);

  // Immediately publish zero velocity and clear the stored command
  void stop();

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg); // Input topic callback
  void on_timer();                                            // Fixed-rate publish + watchdog

  // Keep only linear.x and angular.z (a differential drive can't use the others), within limits
  geometry_msgs::msg::Twist clamp(const geometry_msgs::msg::Twist & cmd) const;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double max_linear_vel_;  // m/s
  double max_angular_vel_; // rad/s
  double cmd_timeout_;     // s
  double publish_rate_;    // Hz

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_; // Latest clamped command
  rclcpp::Time last_cmd_time_;         // When last_cmd_ was received (node clock, so sim time works)
  bool timed_out_{true};               // True while the watchdog is holding the robot stopped
};

}  // namespace neos_drivetrain
