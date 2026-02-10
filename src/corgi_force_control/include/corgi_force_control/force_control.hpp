#ifndef FORCE_CONTROL_HPP
#define FORCE_CONTROL_HPP

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "corgi_force_estimation/force_estimation.hpp"

#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/force_state_stamped.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"


class ForceControlNode : public rclcpp::Node {
public:
    ForceControlNode();
    void run();

private:
    // Callback functions
    void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg);
    void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg);
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timer_cb();
    
    // Control functions
    void force_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                      Eigen::MatrixXd phi_vel_prev_, 
                      corgi_msgs::msg::MotorState* motor_state_, 
                      corgi_msgs::msg::ForceState* force_state_, 
                      corgi_msgs::msg::MotorCmd* motor_cmd_, 
                      double pitch);
    
    void position_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                         corgi_msgs::msg::MotorCmd* motor_cmd_);
    
    static void quaternion_to_euler(const Eigen::Quaterniond& q, 
                                    double& roll, 
                                    double& pitch, 
                                    double& yaw);
    
    // Kinematics helper
    KinematicsHelper kinematics_;
    
    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    corgi_msgs::msg::ForceStateStamped force_state_;
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::MotorCmdStamped motor_cmd_;
    sensor_msgs::msg::Imu imu_;
    
    // ROS2 interfaces
    rclcpp::Subscription<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ForceStateStamped>::SharedPtr force_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_pub_;
    
    // State variables
    std::vector<Eigen::MatrixXd> phi_vel_prev_modules_;
    std::vector<Eigen::MatrixXd> phi_prev_modules_;
    
    // Physical parameters
    const bool sim_ = false;
    const std::vector<double> friction_ = {0.625, 0.44, 0.662, 0.499, 0.623, 0.409, 0.677, 0.356};
    
    int loop_count_ = 0;
};

#endif
