#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include "corgi_msgs/msg/steering_state_stamped.hpp"

/**
 * A node that publishes SteeringStateStamped based on joystick button presses.
 * - B (button index 1) => set current_state=2
 * - Y (button index 3) => set current_state=1
 * - RB (button index 5) => shutdown
 */
class StatePublisher : public rclcpp::Node
{
public:
  StatePublisher();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

  rclcpp::Publisher<corgi_msgs::msg::SteeringStateStamped>::SharedPtr steering_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  corgi_msgs::msg::SteeringStateStamped current_steering_state_;

  // For indexing the B/Y/RB buttons
  int button_b_;
  int button_y_;
  int button_rb_;
};

StatePublisher::StatePublisher()
    : rclcpp::Node("wheeled_state_publisher")
{
  // Load parameters or use defaults
  this->declare_parameter<int>("button_b", 1);
  this->declare_parameter<int>("button_rb", 5);
  button_b_ = this->get_parameter("button_b").as_int();
  button_rb_ = this->get_parameter("button_rb").as_int();

  // Advertise SteeringState and debug_info
  steering_state_pub_ = this->create_publisher<corgi_msgs::msg::SteeringStateStamped>("/steer/state", 10);
  debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_info", 10);

  // Subscribe to joystick
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&StatePublisher::joyCallback, this, std::placeholders::_1));

  // Initialize the steering state
  current_steering_state_.header.stamp = this->now();
  current_steering_state_.current_angle = 0;
  current_steering_state_.current_state = false; // default
  current_steering_state_.cmd_finish = 0;

  // Publish an initial debug message
  std_msgs::msg::String dbg;
  dbg.data = "[StatePublisher] Initialized. Listening for B/RB presses.";
  debug_pub_->publish(dbg);
}

void StatePublisher::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // Check we have enough buttons
  if (joy->buttons.size() <= static_cast<size_t>(std::max(button_b_, button_rb_)))
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] Not enough joystick buttons for B/RB!";
    debug_pub_->publish(dbg);
    return;
  }

  // Press B => set current_state=2
  if (joy->buttons[button_b_] == 1)
  {
    current_steering_state_.current_state = true;
    current_steering_state_.header.stamp = this->now();
    steering_state_pub_->publish(current_steering_state_);

    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] B pressed => set current_state = true";
    debug_pub_->publish(dbg);
  }

  // Press RB => shutdown
  if (joy->buttons[button_rb_] == 1)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] RB pressed => shutting down!";
    debug_pub_->publish(dbg);
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
