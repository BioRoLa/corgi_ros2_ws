#include "joystick_control.hpp"

JoystickControl::JoystickControl()
  : control_mode_(ControlMode::UNKNOWN)
  , current_velocity_(0.0)
  , direction_hold_active_(false)
  , last_direction_(false)
  , was_x_pressed_(false)
  , was_a_pressed_(false)
{
  ros::NodeHandle pnh("~");

  // Default axes + buttons (override via launch if needed)
  pnh.param("axis_left_right",         axis_left_right_,         3);
  pnh.param("axis_left_right_digital", axis_left_right_digital_, 6);
  pnh.param("axis_forward_back",       axis_forward_back_,       7);
  pnh.param("axis_velocity",           axis_velocity_,           1);

  // By default: X=2, A=0 (verify with rostopic echo /joy)
  pnh.param("button_x", button_x_, 2);
  pnh.param("button_a", button_a_, 0);  // NEW

  // Publishers
  steering_cmd_pub_ = nh_.advertise<corgi_msgs::SteeringCmdStamped>("steering_cmd", 1);
  wheel_cmd_pub_    = nh_.advertise<corgi_msgs::WheelCmd>("wheel_cmd", 1);
  debug_pub_ = nh_.advertise<std_msgs::String>("debug_info", 10);

  // Subscribers
  steering_state_sub_ = nh_.subscribe("steering_state", 1,
      &JoystickControl::steeringStateCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1,
      &JoystickControl::joyCallback, this);
}

void JoystickControl::steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg)
{
  current_steering_state_ = *msg;
}

void JoystickControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // =====================================================
  // A) Check A button for resetting to defaults
  // =====================================================
  bool a_pressed_now = false;
  if (button_a_ >= 0 && button_a_ < (int)joy->buttons.size()) {
    a_pressed_now = (joy->buttons[button_a_] == 1);
  }
  // rising edge => from 0 to 1
  if (a_pressed_now && !was_a_pressed_) {
    // Reset everything to defaults
    std_msgs::String debug_msg;
    debug_msg.data = "[JoyCB] A button pressed => resetting to defaults!";
    debug_pub_.publish(debug_msg);

    // 1) Turn off hold
    direction_hold_active_ = false;

    // 2) Steering angle = 0, voltage=4095 (or 0 if you prefer)
    corgi_msgs::SteeringCmdStamped steering_reset;
    steering_reset.header.stamp = ros::Time::now();
    steering_reset.angle   = 0.0;
    steering_reset.voltage = 4095;  // keep consistent with your usage
    steering_cmd_pub_.publish(steering_reset);

    // 3) WheelCmd => direction=false, stop=true, velocity=0
    current_velocity_ = 0.0;
    corgi_msgs::WheelCmd wheel_reset;
    wheel_reset.header.stamp = ros::Time::now();
    wheel_reset.direction = false;
    wheel_reset.stop      = true;
    wheel_reset.velocity  = 0.0f;
    wheel_cmd_pub_.publish(wheel_reset);

    // 4) Also reset the mode to UNKNOWN if you want
    control_mode_ = ControlMode::UNKNOWN;
  }
  was_a_pressed_ = a_pressed_now;  // update for next time

  // ===============================================================
  // B) Check X button for toggling direction hold (existing logic)
  // ===============================================================
  bool x_pressed_now = false;
  if (button_x_ >= 0 && button_x_ < (int)joy->buttons.size()) {
    x_pressed_now = (joy->buttons[button_x_] == 1);
  }
  if (x_pressed_now && !was_x_pressed_) {
    // Toggle direction hold
    direction_hold_active_ = !direction_hold_active_;
    std_msgs::String debug_msg;
    debug_msg.data = std::string("[JoyCB] Toggled direction hold => ") +(direction_hold_active_ ? "ON" : "OFF");
    debug_pub_.publish(debug_msg);
  }
  was_x_pressed_ = x_pressed_now;

  // ===============================================================
  // C) Determine steering mode: analog vs digital
  // ===============================================================
  double lr_analog  = joy->axes[axis_left_right_];
  double lr_digital = joy->axes[axis_left_right_digital_];
  bool   analog_active  = (std::fabs(lr_analog)  > 0.01);
  bool   digital_active = (std::fabs(lr_digital) > 0.01);

  ControlMode new_mode = control_mode_;
  if (digital_active && !analog_active) {
    new_mode = ControlMode::DIGITAL;
  } else if (analog_active && !digital_active) {
    new_mode = ControlMode::ANALOG;
  }
  bool mode_changed = (new_mode != control_mode_ && control_mode_ != ControlMode::UNKNOWN);
  control_mode_ = new_mode;

  // ===============================================================
  // D) SteeringCmd if state == 2, using digital or analog logic
  // ===============================================================
  if (current_steering_state_.current_state == 2 && control_mode_ != ControlMode::UNKNOWN)
  {
    corgi_msgs::SteeringCmdStamped steering_cmd;
    steering_cmd.header.stamp = ros::Time::now();

    double base_angle = 0.0;
    if (!mode_changed && control_mode_ != ControlMode::UNKNOWN) {
      base_angle = current_steering_state_.current_angle;
    }
    double new_angle = base_angle;

    // (a) ANALOG => map [-1..1] to [-10..10]
    if (control_mode_ == ControlMode::ANALOG) {
      new_angle = lr_analog * 10.0; 
    }
    // (b) DIGITAL => ±1 increments
    else if (control_mode_ == ControlMode::DIGITAL) {
      if (lr_digital > 0.1) {
        new_angle += 1.0;
      } else if (lr_digital < -0.1) {
        new_angle -= 1.0;
      }
    }

    // clamp
    new_angle = clamp(new_angle, -10.0, 10.0);
    steering_cmd.angle   = new_angle;
    steering_cmd.voltage = 4095;

    steering_cmd_pub_.publish(steering_cmd);
  }

  // ===============================================================
  // E) WheelCmd => direction + incremental velocity + hold
  // ===============================================================
  corgi_msgs::WheelCmd wheel_cmd;
  wheel_cmd.header.stamp = ros::Time::now();

  // (a) direction hold logic
  double fb_value = joy->axes[axis_forward_back_];
  bool direction_fwd = false;  
  bool stop = false;

  if (direction_hold_active_) {
    // ignore the joystick axis
    direction_fwd = last_direction_;
    stop = false;
  }
  else {
    // normal direction logic
    if (std::fabs(fb_value) < 0.05) {
      stop = true;
      direction_fwd = false;
    } else if (fb_value > 0.0) {
      stop = false;
      direction_fwd = true;
    } else {
      stop = false;
      direction_fwd = false;
    }
    // record last direction if not stopped
    if (!stop) {
      last_direction_ = direction_fwd;
    }
  }
  wheel_cmd.stop      = stop;
  wheel_cmd.direction = direction_fwd;

  // (b) incremental velocity
  double vel_axis = joy->axes[axis_velocity_];
  if (vel_axis > 0.1) {
    current_velocity_ += 0.01;
  } else if (vel_axis < -0.1) {
    current_velocity_ -= 0.01;
  }
  wheel_cmd.velocity = static_cast<float>(current_velocity_);

  // Publish
  wheel_cmd_pub_.publish(wheel_cmd);
}

double JoystickControl::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");
  JoystickControl node;
  ros::spin();
  return 0;
}
