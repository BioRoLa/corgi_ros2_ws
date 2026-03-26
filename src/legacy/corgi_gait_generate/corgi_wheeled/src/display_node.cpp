#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>

const std::string reset("\033[1;0m");
const std::string black("\033[1;30m");
const std::string red("\033[1;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string blue("\033[1;34m");
const std::string magenta("\033[1;35m");
const std::string cyan("\033[1;36m");
const std::string white("\033[1;37m");

class DisplayNode : public rclcpp::Node
{
public:
  DisplayNode() : Node("display_node")
  {
    debug_sub_ = create_subscription<std_msgs::msg::String>(
        "debug_info", 10, [this](const std_msgs::msg::String::SharedPtr msg)
        { RCLCPP_INFO(get_logger(), "[DEBUG] %s%s", msg->data.c_str(), reset.c_str()); });

    steering_cmd_sub_ = create_subscription<corgi_msgs::msg::SteeringCmdStamped>(
        "steering_cmd", 10, [this](const corgi_msgs::msg::SteeringCmdStamped::SharedPtr msg)
        { RCLCPP_INFO(get_logger(), "SteeringCmd => angle=%f, voltage=%d%s",
                      msg->angle, msg->voltage, green.c_str()); });

    steering_state_sub_ = create_subscription<corgi_msgs::msg::SteeringStateStamped>(
        "steering_state", 10, [this](const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg)
        { RCLCPP_INFO(get_logger(), "SteeringState => state=%d, angle=%f%s",
                      msg->current_state, msg->current_angle, green.c_str()); });

    wheel_cmd_sub_ = create_subscription<corgi_msgs::msg::WheelCmd>(
        "wheel_cmd", 10, [this](const corgi_msgs::msg::WheelCmd::SharedPtr msg)
        { RCLCPP_INFO(get_logger(), "WheelCmd => vel=%f, dir=%d, stop=%d%s",
                      msg->velocity, msg->direction, msg->stop, cyan.c_str()); });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_sub_;
  rclcpp::Subscription<corgi_msgs::msg::SteeringCmdStamped>::SharedPtr steering_cmd_sub_;
  rclcpp::Subscription<corgi_msgs::msg::SteeringStateStamped>::SharedPtr steering_state_sub_;
  rclcpp::Subscription<corgi_msgs::msg::WheelCmd>::SharedPtr wheel_cmd_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplayNode>());
  rclcpp::shutdown();
  return 0;
}
