#ifndef JOYSTICK_CONTROL_HPP
#define JOYSTICK_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cmath>
#include <string>

// Custom messages
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>

class JoystickControl : public rclcpp::Node
{
public:
  JoystickControl();

private:
  // Callbacks
  void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

  // Timer callback for 1 kHz
  void wheelCmdTimerCallback();
  void steerCmdTimerCallback();

  double clamp(double value, double min_val, double max_val);

  // ROS members
  rclcpp::Publisher<corgi_msgs::msg::SteeringCmdStamped>::SharedPtr steering_cmd_pub_;
  rclcpp::Publisher<corgi_msgs::msg::WheelCmd>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  rclcpp::Subscription<corgi_msgs::msg::SteeringStateStamped>::SharedPtr steering_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Timer at 1 kHz
  rclcpp::TimerBase::SharedPtr wheel_cmd_timer_;
  rclcpp::TimerBase::SharedPtr steering_cmd_timer_;

  // Store the current steering state
  corgi_msgs::msg::SteeringStateStamped current_steering_state_;
  corgi_msgs::msg::SteeringCmdStamped steer;

  // Indices for axes/buttons
  int axis_left_right_;
  int axis_forward_back_;
  int axis_velocity_;
  int button_hold_;
  int button_reset_;

  // Logic
  bool hold_active_;
  bool was_hold_pressed_;
  bool was_reset_pressed_;

  bool last_direction_;
  double current_velocity_;

  // The last WheelCmd we published
  corgi_msgs::msg::WheelCmd last_wheel_cmd_;
};

#endif // JOYSTICK_CONTROL_HPP
