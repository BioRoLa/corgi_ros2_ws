#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_transform/wheel_to_leg.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("transform_main");
    auto motor_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::array<corgi_msgs::msg::MotorCmd *, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d};

    for (int i = 0; i < 4; i++)
    {
        motor_cmd_modules[i]->kp_r = 90.0;
        motor_cmd_modules[i]->kp_l = 90.0;
        motor_cmd_modules[i]->ki_r = 0.0;
        motor_cmd_modules[i]->ki_l = 0.0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0.0;
        motor_cmd_modules[i]->torque_l = 0.0;
    }

    double init_eta[8] = {17 / 180.0 * M_PI, 0, 17 / 180.0 * M_PI, 55 / 180.0 * M_PI, 17 / 180.0 * M_PI, 0, 17 / 180.0 * M_PI, 0};
    WheelToLegTransformer WheelToLegTransformer(true);
    WheelToLegTransformer.initialize(init_eta);

    std::array<std::array<double, 4>, 2> eta_list;

    // --- Synchronization Setup ---
    // Define 1ms control period (1,000,000 nanoseconds)
    rclcpp::Duration period(0, 1000000);

    // Wait for the ROS 2 clock to start (important if simulation is paused)
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    while (rclcpp::ok())
    {
        // 1. 處理一下 callback，嘗試接收 /clock
        rclcpp::spin_some(node);

        // 2. 檢查現在時間是否大於 0 (代表收到 clock 了)
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break; // 成功對時，跳出等待
        }

        // 3. 小睡一下避免 CPU 100% (這裡可以用 Wall Rate 因為只是在等連線)
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto start_time = node->now();
    rclcpp::Time next_time = start_time;

    while (rclcpp::ok())
    {
        if (WheelToLegTransformer.transform_finished)
        {
            break;
        }

        eta_list = WheelToLegTransformer.step();

        for (int i = 0; i < 4; i++)
        {
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
            // std::cout << i << ": " << eta_list[0][i] << ", " << eta_list[1][i] << std::endl;
        }
        // std::cout << std::endl;

        motor_pub->publish(motor_cmd);

        // --- Synchronized Sleep ---
        next_time += period;
        if (!node->get_clock()->sleep_until(next_time))
        {
            // If the clock jumps or we miss a cycle, warn and exit the loop for consistency
            RCLCPP_WARN(node->get_logger(), "Missed control cycle or clock jump");
            break;
        }
    }

    auto end_time = node->now();
    auto duration = end_time - start_time;

    std::cout << "Transformation complete." << std::endl;
    std::cout << "Total Simulation Time: " << duration.seconds() << " seconds" << std::endl;

    rclcpp::shutdown();

    return 0;
}