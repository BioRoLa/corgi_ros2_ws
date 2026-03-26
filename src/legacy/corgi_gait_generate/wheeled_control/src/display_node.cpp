#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "corgi_msgs/msg/steering_state_stamped.hpp"
#include "corgi_msgs/msg/steering_cmd_stamped.hpp"
#include "corgi_msgs/msg/wheel_cmd.hpp"

void debugInfoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("display_node"), "[DEBUG] %s", msg->data.c_str());
}

void steeringCmdCallback(const corgi_msgs::msg::SteeringCmdStamped::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("display_node"), "SteeringCmd => angle=%.3f, voltage=%.3f", msg->angle, msg->voltage);
}

void wheelCmdCallback(const corgi_msgs::msg::WheelCmd::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("display_node"), "WheelCmd => direction=%s, stop=%s, velocity=%.3f",
              (msg->direction ? "FWD" : "BWD"), (msg->stop ? "TRUE" : "FALSE"), msg->velocity);
}

void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("display_node"), "SteeringState => angle=%.3f, state=%s, cmd_finish=%d",
              msg->current_angle, (msg->current_state ? "TRUE" : "FALSE"), msg->cmd_finish);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("display_node");

  auto debug_sub = node->create_subscription<std_msgs::msg::String>(
      "debug_info", 10, debugInfoCallback);
  auto steering_state_sub = node->create_subscription<corgi_msgs::msg::SteeringStateStamped>(
      "steering_state", 10, steeringStateCallback);
  auto steering_cmd_sub = node->create_subscription<corgi_msgs::msg::SteeringCmdStamped>(
      "steering_cmd", 10, steeringCmdCallback);
  auto wheel_cmd_sub = node->create_subscription<corgi_msgs::msg::WheelCmd>(
      "wheel_cmd", 10, wheelCmdCallback);

  (void)debug_sub;
  (void)steering_state_sub;
  (void)steering_cmd_sub;
  (void)wheel_cmd_sub;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
