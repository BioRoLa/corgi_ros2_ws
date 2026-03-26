#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>

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

  int button_b_;
  int button_y_;
  int button_rb_;
};

// Node constructor - declares parameters and creates publishers/subscribers
StatePublisher::StatePublisher() : rclcpp::Node("state_publisher")
{
  // Declare parameters with default values
  declare_parameter<int>("button_b", 1);
  declare_parameter<int>("button_rb", 5);
  button_b_ = get_parameter("button_b").as_int();
  button_rb_ = get_parameter("button_rb").as_int();

  // Create publishers for steering state and debug info
  steering_state_pub_ = create_publisher<corgi_msgs::msg::SteeringStateStamped>("/steer/state", 1);
  debug_pub_ = create_publisher<std_msgs::msg::String>("debug_info", 10);

  // Subscribe to joystick input with callback binding
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, std::bind(&StatePublisher::joyCallback, this, std::placeholders::_1));

  // Initialize steering state with ROS2 timestamp
  current_steering_state_.header.stamp = now();
  current_steering_state_.current_angle = 0;
  current_steering_state_.current_state = false;
  current_steering_state_.cmd_finish = 0;

  // Publish initialization message
  std_msgs::msg::String dbg;
  dbg.data = "[StatePublisher] Initialized. Listening for B/RB presses.";
  debug_pub_->publish(dbg);
}

// Joy callback - publishes steering state when B or RB buttons pressed
void StatePublisher::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // Validate joystick has enough buttons
  if (joy->buttons.size() <= std::max({button_b_, button_rb_}))
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] Not enough joystick buttons for B/RB!";
    debug_pub_->publish(dbg);
    return;
  }

  // When B button pressed, set steering state to true
  if (joy->buttons[button_b_] == 1)
  {
    current_steering_state_.current_state = true;
    current_steering_state_.header.stamp = now();
    steering_state_pub_->publish(current_steering_state_);

    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] B pressed => set current_state = true";
    debug_pub_->publish(dbg);
  }

  // When RB button pressed, shutdown the node
  if (joy->buttons[button_rb_] == 1)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] RB pressed => shutting down!";
    debug_pub_->publish(dbg);
    rclcpp::shutdown();
  }
}

// Main entry point - initializes the node and begins spinning
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}
