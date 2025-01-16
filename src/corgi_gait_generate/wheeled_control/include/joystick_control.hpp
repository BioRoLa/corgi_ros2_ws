#ifndef JOYSTICK_CONTROL_HPP
#define JOYSTICK_CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/String.h>

#include <corgi_msgs/SteeringStateStamped.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/WheelCmd.h>

enum class ControlMode
{
  UNKNOWN,
  ANALOG,
  DIGITAL
};

class JoystickControl
{
public:
  JoystickControl();
  ~JoystickControl() = default;

private:
  void steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  double clamp(double value, double min_val, double max_val);

  ros::NodeHandle nh_;
  ros::Publisher  steering_cmd_pub_;
  ros::Publisher  wheel_cmd_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber steering_state_sub_;
  ros::Subscriber joy_sub_;

  corgi_msgs::SteeringStateStamped current_steering_state_;

  // Axis indices
  int axis_left_right_;
  int axis_left_right_digital_;
  int axis_forward_back_;
  int axis_velocity_;

  // Button indices
  int button_x_;
  int button_a_;  // NEW: for resetting to defaults

  ControlMode control_mode_;
  double current_velocity_;

  // For direction hold (from previous code)
  bool direction_hold_active_;
  bool last_direction_;
  bool was_x_pressed_;

  // NEW: detect edges for A button
  bool was_a_pressed_;
};

#endif // JOYSTICK_CONTROL_HPP
