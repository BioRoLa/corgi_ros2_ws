#ifndef WHEELED_HPP
#define WHEELED_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "joystick_control.hpp"

#include <corgi_msgs/msg/wheel_cmd.hpp>
#include <corgi_msgs/msg/motor_state.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>

class Wheeled : public rclcpp::Node
{
public:
    Wheeled();
    ~Wheeled() = default;

private:
    void wheelCmdCallback(const corgi_msgs::msg::WheelCmd::SharedPtr msg);
    void motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void steerStateCallback(const corgi_msgs::msg::SteeringCmdStamped::SharedPtr msg);

    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::WheelCmd>::SharedPtr wheel_cmd_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::SteeringCmdStamped>::SharedPtr steer_cmd_sub_;

    corgi_msgs::msg::MotorStateStamped current_motor_state_;
    corgi_msgs::msg::MotorCmdStamped current_motor_cmd_;
    corgi_msgs::msg::WheelCmd current_wheel_cmd_;
    corgi_msgs::msg::SteeringCmdStamped current_steer_cmd_;

    std::vector<corgi_msgs::msg::MotorState *> motor_state_modules = {
        &current_motor_state_.module_a,
        &current_motor_state_.module_b,
        &current_motor_state_.module_c,
        &current_motor_state_.module_d};

    std::vector<corgi_msgs::msg::MotorCmd *> motor_cmds = {
        &current_motor_cmd_.module_a,
        &current_motor_cmd_.module_b,
        &current_motor_cmd_.module_c,
        &current_motor_cmd_.module_d};
};

#endif