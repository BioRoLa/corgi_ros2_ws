#include "joystick_control.hpp"

JoystickControl::JoystickControl()
    : Node("joystick_control"), hold_active_(false), was_hold_pressed_(false), was_reset_pressed_(false), last_direction_(false), current_velocity_(0.0)
{
  // Load parameters (or set defaults)
  this->declare_parameter("axis_left_right", 3);
  this->declare_parameter("axis_forward_back", 1);
  this->declare_parameter("axis_velocity", 7);
  this->declare_parameter("button_hold", 2);
  this->declare_parameter("button_reset", 0);

  axis_left_right_ = this->get_parameter("axis_left_right").as_int();
  axis_forward_back_ = this->get_parameter("axis_forward_back").as_int();
  axis_velocity_ = this->get_parameter("axis_velocity").as_int();
  button_hold_ = this->get_parameter("button_hold").as_int();
  button_reset_ = this->get_parameter("button_reset").as_int();

  // Publishers
  steering_cmd_pub_ = this->create_publisher<corgi_msgs::msg::SteeringCmdStamped>("/steer/command", 1);
  wheel_cmd_pub_ = this->create_publisher<corgi_msgs::msg::WheelCmd>("wheel_cmd", 1);
  debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_info", 10);

  // Subscribers
  steering_state_sub_ = this->create_subscription<corgi_msgs::msg::SteeringStateStamped>(
      "/steer/state", 1,
      std::bind(&JoystickControl::steeringStateCallback, this, std::placeholders::_1));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&JoystickControl::joyCallback, this, std::placeholders::_1));

  // Create timer at 1 kHz => 1 ms
  wheel_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&JoystickControl::wheelCmdTimerCallback, this));
  steering_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&JoystickControl::steerCmdTimerCallback, this));

  // Initialize last_wheel_cmd_ => stop=true
  last_wheel_cmd_.header.stamp = this->now();
  last_wheel_cmd_.stop = true;
  last_wheel_cmd_.direction = false;
  last_wheel_cmd_.velocity = 0.0f;
}

void JoystickControl::steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg)
{
  current_steering_state_ = *msg;
}

// Called at 1 kHz
void JoystickControl::wheelCmdTimerCallback()
{
  // If stop=false => continuously publish at 1000 Hz
  if (!last_wheel_cmd_.stop)
  {
    last_wheel_cmd_.header.stamp = this->now();
    wheel_cmd_pub_->publish(last_wheel_cmd_);
  }
}

void JoystickControl::steerCmdTimerCallback()
{
  // If stop=false => continuously publish at 1000 Hz
  if (steer.voltage != 0.0)
  {
    steer.header.stamp = this->now();
    steering_cmd_pub_->publish(steer);
  }
}

void JoystickControl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // Example logic:
  //  - Check reset button => set stop=true
  //  - Check hold button => toggles hold + forces stop=false if hold=ON
  //  - If hold=OFF => read direction axis => if near 0 => stop=true, else stop=false
  //  - Velocity increments
  //  - If stop=true => publish once velocity=0, if stop=false => keep re-publishing in timer

  // 1) Reset
  bool reset_now = (button_reset_ >= 0 && button_reset_ < (int)joy->buttons.size() && joy->buttons[button_reset_] == 1);
  if (reset_now && !was_reset_pressed_)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Reset => stop=true, velocity=0, angle=0, hold=OFF";
    debug_pub_->publish(dbg);

    hold_active_ = false;
    current_velocity_ = 0.0;

    // Steering reset => angle=0
    {
      steer.header.stamp = this->now();
      steer.angle = 0.0;
      steer.voltage = 0;
      steering_cmd_pub_->publish(steer);
    }

    // Wheel => stop=true, velocity=0
    corgi_msgs::msg::WheelCmd wheel;
    wheel.header.stamp = this->now();
    wheel.stop = true;
    wheel.direction = false;
    wheel.velocity = 0.0f;
    wheel_cmd_pub_->publish(wheel);

    last_wheel_cmd_ = wheel; // no more re-pub
  }
  was_reset_pressed_ = reset_now;

  // 2) Toggle hold => if hold=ON => stop=false
  bool hold_now = (button_hold_ >= 0 && button_hold_ < (int)joy->buttons.size() && joy->buttons[button_hold_] == 1);
  if (hold_now && !was_hold_pressed_)
  {
    hold_active_ = !hold_active_;
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Toggled hold => " + std::string(hold_active_ ? "ON" : "OFF");
    debug_pub_->publish(dbg);

    if (hold_active_)
    {
      // Force stop=false => keep rolling
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.header.stamp = this->now();
      wheel_cmd_pub_->publish(last_wheel_cmd_);
    }
  }
  was_hold_pressed_ = hold_now;

  // 3) SteeringCmd => e.g. if current_state==true => analog angle
  if (current_steering_state_.current_state)
  {
    steer.header.stamp = this->now();
    if ((int)joy->axes.size() > axis_left_right_)
    {
      double lr = joy->axes[axis_left_right_];
      steer.angle = clamp(lr * -10.0, -8.0, 8.0);
    }
    else
    {
      steer.angle = 0.0;
    }
    steer.voltage = 4095; // or your chosen max
    steering_cmd_pub_->publish(steer);
  }

  // 4) WheelCmd => direction axis => if hold=OFF
  corgi_msgs::msg::WheelCmd wheel;
  wheel.header.stamp = this->now();

  bool stop = true;
  bool dir = false;

  if (hold_active_)
  {
    // hold=ON => always stop=false, direction locked
    stop = false;
    dir = last_wheel_cmd_.direction;
  }
  else
  {
    // read direction axis => if outside ±0.05 => stop=false
    if ((int)joy->axes.size() > axis_forward_back_)
    {
      double fb = joy->axes[axis_forward_back_];
      if (std::fabs(fb) > 0.05)
      {
        stop = false;
        dir = (fb > 0.0);
      }
    }
  }

  // velocity increments
  if ((int)joy->axes.size() > axis_velocity_)
  {
    double vel_axis = joy->axes[axis_velocity_];
    if (vel_axis > 0.1)
    {
      current_velocity_ += 0.05;
    }
    else if (vel_axis < -0.1)
    {
      current_velocity_ -= 0.05;
    }
  }

  wheel.stop = stop;
  wheel.direction = dir;
  wheel.velocity = current_velocity_;

  // Publish once
  wheel_cmd_pub_->publish(wheel);

  // Store => timer uses for re-pub if stop=false
  last_wheel_cmd_ = wheel;
}

double JoystickControl::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}
