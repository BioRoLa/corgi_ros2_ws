#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include "std_msgs/msg/string.hpp"

#include "corgi_msgs/msg/steering_state_stamped.hpp"
#include "corgi_msgs/msg/steering_cmd_stamped.hpp"
#include "corgi_msgs/msg/wheel_cmd.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"

class simple_test : public rclcpp::Node
{
private:
  void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg);
  rclcpp::Publisher<corgi_msgs::msg::SteeringCmdStamped>::SharedPtr steering_cmd_pub_;
  rclcpp::Publisher<corgi_msgs::msg::WheelCmd>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Subscription<corgi_msgs::msg::SteeringStateStamped>::SharedPtr steering_state_sub_;
  corgi_msgs::msg::SteeringStateStamped current_steering_state_;
  corgi_msgs::msg::MotorStateStamped current_motor_state_;
  int state;
  int add_wheel;
  void collaborate1();
  void collaborate2();
  void set_zero();

public:
  simple_test();
  ~simple_test() = default;
};

simple_test::simple_test()
    : Node("wheeled_simple")
{
  // Publishers
  steering_cmd_pub_ = this->create_publisher<corgi_msgs::msg::SteeringCmdStamped>("/steer/command", 10);
  wheel_cmd_pub_ = this->create_publisher<corgi_msgs::msg::WheelCmd>("wheel_cmd", 10);
  debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_info", 10);

  // Subscribers
  steering_state_sub_ = this->create_subscription<corgi_msgs::msg::SteeringStateStamped>(
      "/steer/state", 10, std::bind(&simple_test::steeringStateCallback, this, std::placeholders::_1));
  state = -1;
}

void simple_test::steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::SharedPtr msg)
{
  current_steering_state_ = *msg;
  if (current_steering_state_.cmd_finish == 1)
  {
    std::cout << "Wrong cmd! " << std::endl;
  }
  else if (current_steering_state_.current_state == true && state == -1)
  {
    state = 0;
    collaborate1();
    std::cout << "sending1" << std::endl;
  }
  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 && state == 0)
  {
    collaborate1();
    std::cout << "sending1" << std::endl;
  }

  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 && state == 0)
  {
    state = 1;
    std::cout << "finished1" << std::endl;
  }

  else if (current_steering_state_.current_state == true && state == 1)
  {
    state = 2;
    collaborate2();
    std::cout << "sending2" << std::endl;
  }
  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 && state == 2)
  {
    collaborate2();
    std::cout << "sending3" << std::endl;
  }

  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 && state == 2 && current_steering_state_.current_angle == -10)
  {
    state = 3;
    std::cout << "finished4" << std::endl;
  }

  else if (current_steering_state_.current_state == true && state == 3)
  {
    state = 4;
    set_zero();
    std::cout << "sending5" << std::endl;
  }

  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 && state == 4)
  {
    set_zero();
    std::cout << "sending6" << std::endl;
  }

  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 && state == 4)
  {
    state = 5;
    std::cout << "finished7" << std::endl;
  }
}

void simple_test::collaborate1()
{
  corgi_msgs::msg::SteeringCmdStamped current_steering_cmd1;
  current_steering_cmd1.header.stamp = this->now();
  current_steering_cmd1.voltage = 4000;
  current_steering_cmd1.angle = 10;
  steering_cmd_pub_->publish(current_steering_cmd1);
  // std::cout << current_steering_cmd1<< std::endl;
}

void simple_test::collaborate2()
{
  corgi_msgs::msg::SteeringCmdStamped current_steering_cmd2;
  current_steering_cmd2.header.stamp = this->now();
  current_steering_cmd2.voltage = 4000;
  current_steering_cmd2.angle = -10;
  steering_cmd_pub_->publish(current_steering_cmd2);
  // std::cout << current_steering_cmd2<< std::endl;
}

void simple_test::set_zero()
{
  corgi_msgs::msg::SteeringCmdStamped current_steering_cmd3;
  current_steering_cmd3.header.stamp = this->now();
  current_steering_cmd3.voltage = 4000;
  current_steering_cmd3.angle = 0;
  steering_cmd_pub_->publish(current_steering_cmd3);
  // std::cout << current_steering_cmd3 << std::endl;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_test>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}