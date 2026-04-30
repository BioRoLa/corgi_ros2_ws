#include "corgi_wheeled/wheeled_cmd.hpp"
#include <chrono>

using namespace std::chrono_literals;

WheeledCmd::WheeledCmd(rclcpp::Node::SharedPtr node, std::string control_mode)
    : node_ptr_(std::move(node)),
      hold_active_(false),
      was_hold_pressed_(false),
      was_reset_pressed_(false),
      current_velocity_(0.0),
      control_mode_(std::move(control_mode))
{
  node_ptr_->declare_parameter<int>("axis_steer", 3);
  node_ptr_->declare_parameter<int>("axis_move", 1);
  node_ptr_->declare_parameter<int>("axis_accel", 7);
  node_ptr_->declare_parameter<int>("button_toggle_hold", 2);
  node_ptr_->declare_parameter<int>("button_reset", 0);
  node_ptr_->declare_parameter<int>("button_ground", 3);

  axis_steer_ = node_ptr_->get_parameter("axis_steer").as_int();
  axis_move_ = node_ptr_->get_parameter("axis_move").as_int();
  axis_accel_ = node_ptr_->get_parameter("axis_accel").as_int();
  button_toggle_hold_ = node_ptr_->get_parameter("button_toggle_hold").as_int();
  button_reset_ = node_ptr_->get_parameter("button_reset").as_int();
  button_ground_ = node_ptr_->get_parameter("button_ground").as_int();

  steering_cmd_pub_ = node_ptr_->create_publisher<corgi_msgs::msg::SteeringCmdStamped>("/steer/command", 10);
  wheel_cmd_pub_ = node_ptr_->create_publisher<corgi_msgs::msg::WheelCmd>("wheel_cmd", 10);
  debug_pub_ = node_ptr_->create_publisher<std_msgs::msg::String>("debug_info", 10);

  if (control_mode_ == "teleop")
  {
    teleop_sub_ = node_ptr_->create_subscription<std_msgs::msg::String>(
        "teleop_keys", 10, std::bind(&WheeledCmd::teleopCallback, this, std::placeholders::_1));
  }
  else if (control_mode_ == "pure")
  {
    RCLCPP_WARN(node_ptr_->get_logger(), "Pure code mode selected");
  }
  else
  {
    joy_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&WheeledCmd::joyCallback, this, std::placeholders::_1));
  }

  steering_state_sub_ = node_ptr_->create_subscription<corgi_msgs::msg::SteeringStateStamped>(
      "/steer/state", 10, std::bind(&WheeledCmd::steeringStateCallback, this, std::placeholders::_1));

  wheel_cmd_timer_ = node_ptr_->create_wall_timer(1ms, std::bind(&WheeledCmd::wheelCmdTimerCallback, this));
  steer_cmd_timer_ = node_ptr_->create_wall_timer(1ms, std::bind(&WheeledCmd::steerCmdTimerCallback, this));

  last_wheel_cmd_.header.stamp = node_ptr_->now();
  last_wheel_cmd_.stop = true;
  last_wheel_cmd_.direction = false;
  last_wheel_cmd_.velocity = 0.0f;
  last_wheel_cmd_.ground_rotate = false;

  steering_cmd_.voltage = 0;
  steering_cmd_.angle = 0.0;
}

double WheeledCmd::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}

void WheeledCmd::wheelCmdTimerCallback()
{
  if (!last_wheel_cmd_.stop)
  {
    last_wheel_cmd_.header.stamp = node_ptr_->now();
    wheel_cmd_pub_->publish(last_wheel_cmd_);
  }
}

void WheeledCmd::steerCmdTimerCallback()
{
  if (steering_cmd_.voltage != 0)
  {
    steering_cmd_.header.stamp = node_ptr_->now();
    steering_cmd_pub_->publish(steering_cmd_);
  }
}

void WheeledCmd::steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg)
{
  current_steering_state_ = *msg;
}

void WheeledCmd::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  bool reset_now = (button_reset_ >= 0 && button_reset_ < (int)joy->buttons.size() && joy->buttons[button_reset_] == 1);
  if (reset_now && !was_reset_pressed_)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Reset: stopping, zero velocity and angle, hold OFF";
    debug_pub_->publish(dbg);

    hold_active_ = false;
    current_velocity_ = 0.0;

    steering_cmd_.angle = 0.0;
    steering_cmd_.voltage = 0;
    steering_cmd_.header.stamp = node_ptr_->now();
    steering_cmd_pub_->publish(steering_cmd_);

    last_wheel_cmd_.header.stamp = node_ptr_->now();
    last_wheel_cmd_.stop = true;
    last_wheel_cmd_.direction = false;
    last_wheel_cmd_.velocity = 0.0f;
    last_wheel_cmd_.ground_rotate = false;
    wheel_cmd_pub_->publish(last_wheel_cmd_);
  }
  was_reset_pressed_ = reset_now;

  bool hold_now = (button_toggle_hold_ >= 0 && button_toggle_hold_ < (int)joy->buttons.size() && joy->buttons[button_toggle_hold_] == 1);
  if (hold_now && !was_hold_pressed_)
  {
    hold_active_ = !hold_active_;
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Toggled hold: " + std::string(hold_active_ ? "ON" : "OFF");
    debug_pub_->publish(dbg);

    if (hold_active_)
    {
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.header.stamp = node_ptr_->now();
      wheel_cmd_pub_->publish(last_wheel_cmd_);
    }
  }
  was_hold_pressed_ = hold_now;

  if (current_steering_state_.current_state)
  {
    if ((int)joy->axes.size() > axis_steer_)
    {
      double steer_input = joy->axes[axis_steer_];
      steering_cmd_.angle = clamp(steer_input * -10.0, -10.0, 10.0);
    }
    else
    {
      steering_cmd_.angle = 0.0;
    }
    steering_cmd_.voltage = 4095;
    steering_cmd_.header.stamp = node_ptr_->now();
    steering_cmd_pub_->publish(steering_cmd_);
  }

  bool stop = true;
  bool direction = false;
  if (hold_active_)
  {
    stop = false;
    direction = last_wheel_cmd_.direction;
  }
  else
  {
    if ((int)joy->axes.size() > axis_move_)
    {
      double move_input = joy->axes[axis_move_];
      if (std::fabs(move_input) > 0.05)
      {
        stop = false;
        direction = (move_input > 0.0);
      }
    }
  }

  if ((int)joy->axes.size() > axis_accel_)
  {
    double accel_input = joy->axes[axis_accel_];
    if (accel_input > 0.1)
      current_velocity_ += 0.05;
    else if (accel_input < -0.1)
      current_velocity_ -= 0.05;
  }

  last_wheel_cmd_.ground_rotate = (button_ground_ >= 0 && button_ground_ < (int)joy->buttons.size() && joy->buttons[button_ground_] == 1);

  last_wheel_cmd_.stop = stop;
  last_wheel_cmd_.direction = direction;
  last_wheel_cmd_.velocity = current_velocity_;
  last_wheel_cmd_.header.stamp = node_ptr_->now();
  wheel_cmd_pub_->publish(last_wheel_cmd_);
}

void WheeledCmd::teleopCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "72")
  {
    last_wheel_cmd_.velocity += 0.05;
  }
  else if (msg->data == "80")
  {
    last_wheel_cmd_.velocity -= 0.05;
  }
  else
  {
    char key = msg->data.empty() ? '\0' : msg->data[0];
    std_msgs::msg::String dbg;
    dbg.data = "[TeleopCB] Received key: " + std::string(1, key);
    debug_pub_->publish(dbg);
    switch (key)
    {
    case 'k':
      last_wheel_cmd_.stop = true;
      last_wheel_cmd_.ground_rotate = false;
      break;
    case 'i':
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = true;
      break;
    case ',':
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = false;
      break;
    case 'l':
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = true;
      last_wheel_cmd_.direction = true;
      break;
    case 'j':
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = true;
      last_wheel_cmd_.direction = false;
      break;
    case 'u':
      steering_cmd_.angle = -10;
      steering_cmd_.voltage = 4095;
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = true;
      break;
    case 'm':
      steering_cmd_.angle = -10;
      steering_cmd_.voltage = 4095;
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = false;
      break;
    case 'o':
      steering_cmd_.angle = 10;
      steering_cmd_.voltage = 4095;
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = true;
      break;
    case '.':
      steering_cmd_.angle = 10;
      steering_cmd_.voltage = 4095;
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      last_wheel_cmd_.direction = false;
      break;
    case '1':
      steering_cmd_.angle = std::min(steering_cmd_.angle + 1, 12.0);
      steering_cmd_.voltage = 4095;
      steering_cmd_.header.stamp = node_ptr_->now();
      steering_cmd_pub_->publish(steering_cmd_);
      break;
    case '2':
      steering_cmd_.angle = std::max(steering_cmd_.angle - 1, -12.0);
      steering_cmd_.voltage = 4095;
      steering_cmd_.header.stamp = node_ptr_->now();
      steering_cmd_pub_->publish(steering_cmd_);
      break;
    case '[':
      steering_cmd_.angle = 0;
      steering_cmd_.voltage = 4095;
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.ground_rotate = false;
      steering_cmd_.header.stamp = node_ptr_->now();
      steering_cmd_pub_->publish(steering_cmd_);
      break;
    default:
      break;
    }

    last_wheel_cmd_.header.stamp = node_ptr_->now();
    wheel_cmd_pub_->publish(last_wheel_cmd_);
  }
}
