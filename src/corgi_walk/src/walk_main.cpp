#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_walk/walk_gait.hpp"
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("walk_test");
    auto motor_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std_msgs::msg::Int32MultiArray phase_msg;
    phase_msg.data.resize(4, 0);
    std::array<corgi_msgs::msg::MotorCmd *, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d};
    for (int i = 0; i < 4; i++)
    {
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    } // end for

    double CoM_bias = 0.0;
    int sampling_rate = 1000;
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246, 0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334}; // normal
    WalkGait walk_gait(true, CoM_bias, sampling_rate);
    walk_gait.initialize(init_eta);
    std::array<std::array<double, 4>, 2> eta_list;
    double velocity = 0.1;
    double stand_height = 0.2;
    double step_length = 0.3;
    double step_height = 0.05;
    double curvature = 0.0;
    int count = 0;

    // --- Synchronization Setup ---
    // Define control period based on sampling_rate (seconds = 1 / sampling_rate)
    rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(sampling_rate));

    // Wait for the ROS 2 clock to start (important if simulation is paused)
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    while (rclcpp::ok())
    {
        // 1. Process callbacks to try to receive /clock messages
        rclcpp::spin_some(node);

        // 2. Check if the current time is greater than 0 (indicates that the clock has been received)
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break; // Successfully synchronized, exit waiting loop
        }

        // 3. Sleep briefly to avoid 100% CPU usage (Wall time is fine while waiting for the connection)
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto start_time = node->now();
    rclcpp::Time next_time = start_time;
    while (rclcpp::ok())
    {
        // for (int count=0; count<200000; ){
        count++;
        velocity = 0.2 * cos(count / 1711.0);
        walk_gait.set_velocity(velocity);
        stand_height = 0.25 + 0.05 * cos(count / 1211.0);
        walk_gait.set_stand_height(stand_height);
        step_length = (count++ / 3311) % 2 == 0 ? 0.3 : 0.1;
        walk_gait.set_step_length(step_length);
        step_height = (count++ / 2311) % 2 == 0 ? 0.08 : 0.04;
        walk_gait.set_step_height(step_height);
        curvature = 1.0 * sin(count / 3911.0);
        walk_gait.set_curvature(curvature);
        eta_list = walk_gait.step();
        // Publish motor commands
        for (int i = 0; i < 4; i++)
        {
            if (eta_list[0][i] > M_PI * 160.0 / 180.0)
            {
                std::cout << "Exceed upper bound." << std::endl;
            } // end if
            if (eta_list[0][i] < M_PI * 17.0 / 180.0)
            {
                std::cout << "Exceed lower bound." << std::endl;
            } // end if
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
        }
        motor_pub->publish(motor_cmd);
            auto swing_phase = walk_gait.get_swing_phase();
            for (int i = 0; i < 4; i++) { phase_msg.data[i] = swing_phase[i]; }
            phase_pub->publish(phase_msg);
        // --- Synchronized Sleep ---
        next_time += period;
        if (!node->get_clock()->sleep_until(next_time))
        {
            // If the clock jumps or we miss a cycle, warn but continue
            RCLCPP_WARN(node->get_logger(), "Missed control cycle or clock jump");
            next_time = node->now(); // Reset baseline
        }
    }
    auto end_time = node->now();
    auto duration = end_time - start_time;
    std::cout << "time: " << duration.seconds() << " seconds" << std::endl;

    rclcpp::shutdown();
    return 0;
} // end main
