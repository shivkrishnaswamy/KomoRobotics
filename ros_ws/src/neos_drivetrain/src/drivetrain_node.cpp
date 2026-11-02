#include "neos_drivetrain/drivetrain_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

namespace neos_drivetrain
{

DrivetrainNode::DrivetrainNode(const rclcpp::NodeOptions & options)
: Node("drivetrain_node", options)
{
  // Declare parameters with defaults. Override with config/drivetrain.yaml or --ros-args -p name:=value
  input_topic_ = declare_parameter<std::string>("input_topic", "drivetrain/cmd_vel");
  output_topic_ = declare_parameter<std::string>("output_topic", "cmd_vel");
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.5);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.0);
  cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
  publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

  if (publish_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate must be > 0, using 20.0 Hz");
    publish_rate_ = 20.0;
  }

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic_, 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {on_cmd(msg);});

  // Wall timer: keeps publishing even if the sim clock is paused, so the robot gets the zero command
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    [this]() {on_timer();});

  last_cmd_time_ = now();

  RCLCPP_INFO(
    get_logger(), "Drivetrain node started: %s -> %s (max %.2f m/s, %.2f rad/s, timeout %.2f s)",
    input_topic_.c_str(), output_topic_.c_str(), max_linear_vel_, max_angular_vel_, cmd_timeout_);
}

void DrivetrainNode::send_twist(const geometry_msgs::msg::Twist & cmd)
{
  last_cmd_ = clamp(cmd);
  last_cmd_time_ = now();

  if (timed_out_) {
    RCLCPP_INFO(get_logger(), "Receiving commands, drivetrain active");
    timed_out_ = false;
  }
}

void DrivetrainNode::stop()
{
  last_cmd_ = geometry_msgs::msg::Twist();  // Default-constructed Twist is all zeros
  cmd_pub_->publish(last_cmd_);
}

void DrivetrainNode::on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_twist(*msg);
}

void DrivetrainNode::on_timer()
{
  const double since_last_cmd = (now() - last_cmd_time_).seconds();

  // Watchdog: if the command source died or stopped publishing, stop the robot
  if (since_last_cmd > cmd_timeout_) {
    if (!timed_out_) {
      RCLCPP_WARN(get_logger(), "No command for %.2f s, stopping drivetrain", since_last_cmd);
      timed_out_ = true;
    }
    stop();
    return;
  }

  cmd_pub_->publish(last_cmd_);
}

geometry_msgs::msg::Twist DrivetrainNode::clamp(const geometry_msgs::msg::Twist & cmd) const
{
  geometry_msgs::msg::Twist out;  // Everything else stays zero
  out.linear.x = std::clamp(cmd.linear.x, -max_linear_vel_, max_linear_vel_);
  out.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);
  return out;
}

}  // namespace neos_drivetrain

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<neos_drivetrain::DrivetrainNode>();

  // Send one last zero command on Ctrl+C. Runs before the context shuts down, while publishing still works.
  // (Needed because Gazebo's DiffDrive keeps applying the last command it received.)
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak_node = std::weak_ptr<neos_drivetrain::DrivetrainNode>(node)]() {
      if (auto n = weak_node.lock()) {
        n->stop();
      }
    });

  rclcpp::spin(node);  // Blocks, running callbacks, until Ctrl+C
  rclcpp::shutdown();
  return 0;
}
