#ifndef WHEEL_CMD_HPP
#define WHEEL_CMD_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <algorithm>

class WheeledCmd
{
public:
  explicit WheeledCmd(rclcpp::Node::SharedPtr node, std::string control_mode = "joystick");
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void teleopCallback(const std_msgs::msg::String::SharedPtr key_msg);

  void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg);
  // Timers for continuous publishing when commands are active
  void wheelCmdTimerCallback();
  void steerCmdTimerCallback();
  corgi_msgs::msg::SteeringCmdStamped steering_cmd_;
  // Current steering state feedback (from /steer/state)
  corgi_msgs::msg::SteeringStateStamped current_steering_state_;

private:
  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
  rclcpp::Subscription<corgi_msgs::msg::SteeringStateStamped>::SharedPtr steering_state_sub_;

  rclcpp::Publisher<corgi_msgs::msg::WheelCmd>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<corgi_msgs::msg::SteeringCmdStamped>::SharedPtr steering_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  rclcpp::TimerBase::SharedPtr wheel_cmd_timer_;
  rclcpp::TimerBase::SharedPtr steer_cmd_timer_;

  corgi_msgs::msg::WheelCmd last_wheel_cmd_;

  int axis_steer_;
  int axis_move_;
  int axis_accel_;
  int button_toggle_hold_;
  int button_reset_;
  int button_ground_;

  // Control mode ("joystick" or "teleop" or "pure")
  std::string control_mode_;

  // Internal state variables
  bool hold_active_;
  bool was_hold_pressed_;
  bool was_reset_pressed_;
  double current_velocity_;

  // Helper function: clamp a value between a minimum and maximum
  double clamp(double value, double min_val, double max_val);
};

#endif
