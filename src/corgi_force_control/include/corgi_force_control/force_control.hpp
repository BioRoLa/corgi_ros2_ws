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
#include "corgi_msgs/msg/imu_stamped.hpp"


class ForceControlNode : public rclcpp::Node {
public:
    ForceControlNode();
    void run();

private:
    // Callback functions
    void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg);
    void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg);
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
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
    corgi_msgs::msg::ImuStamped imu_;
    
    // ROS2 interfaces
    rclcpp::Subscription<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ForceStateStamped>::SharedPtr force_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_pub_;
    
    // State variables
    std::vector<Eigen::MatrixXd> phi_vel_prev_modules_;
    std::vector<Eigen::MatrixXd> phi_prev_modules_;
    
    // Physical parameters
    const bool sim_ = false;
    const std::vector<double> friction_ = {0.625, 0.44, 0.662, 0.499, 0.623, 0.409, 0.677, 0.356};

    // Friction feedforward shaping. See the block in force_control.cpp:
    // the raw comparison reads CAN quantisation rather than direction and
    // chatters +/-0.625 N.m at up to 1 kHz.
    //
    // fric_dir_[module][0]=left, [1]=right. 0 until real motion is seen,
    // so a parked leg carries no bias.
    int fric_dir_[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    // Slew limit on the PUBLISHED command, deg/s. 0 disables an axis.
    // Off by default -- see slew.py. Limits the consequence (a
    // full-range step in one tick) rather than the cause.
    double cmd_slew_theta_ = 0.0;
    double cmd_slew_beta_ = 0.0;
    bool cmd_slew_primed_ = false;   // first tick seeds, never limits
    double cmd_slew_prev_th_[4] = {0, 0, 0, 0};
    double cmd_slew_prev_be_[4] = {0, 0, 0, 0};
    long cmd_slew_hits_ = 0;         // ticks the limiter actually bit
    double state_probe_t0_ = 0.0;  // window for the per-leg theta probe
    // False until the first impedance command arrives. Until then the
    // node publishes NOTHING: a zero-initialised ImpedanceCmd takes the
    // position_control branch, which commands theta=0 with kp=50 --
    // fold the legs, hard. See no_zero_cmd.py.
    // Ticks on which at least one leg fell into position_control.
    // Reported and reset every 2 s; a throttled warning without a
    // count cannot distinguish two events from eight thousand.
    unsigned long long zero_gain_ticks_ = 0;
    bool imp_cmd_seen_ = false;
    // True while the impedance stream has been silent long enough that
    // this node has stopped publishing. See two_writer_fix.py.
    bool imp_stream_dead_ = false;
    double imp_wait_t0_ = 0.0;   // when the wait started, for the report
    long imp_rx_count_ = 0;   // impedance messages since the last report
    double imp_rx_t0_ = 0.0;  // when that report window opened
    double imp_rx_last_ = 0.0;       // arrival time of the previous message
    double imp_rx_worst_gap_ = 0.0;  // worst gap this window, seconds
    double friction_ff_scale_ = 1.0;      // 0.0 disables the term entirely
    double friction_deadband_ = 0.00288;  // rad; one CAN LSB is 0.1648 deg
    
    int loop_count_ = 0;
};

#endif
