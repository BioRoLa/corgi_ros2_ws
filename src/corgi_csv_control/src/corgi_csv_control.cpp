#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_csv_control");
    // rclcpp::Time now = node->now();
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    
    while (rclcpp::ok()) {
        // 1. 處理一下 callback，嘗試接收 /clock
        rclcpp::spin_some(node);
        
        // 2. 檢查現在時間是否大於 0 (代表收到 clock 了)
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break; // 成功對時，跳出等待
        }
        
        // 3. 小睡一下避免 CPU 100% (這裡可以用 Wall Rate 因為只是在等連線)
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    // rclcpp::Rate rate(1000);
    // use_sim_time setting
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmds = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    if (argc < 2){
        RCLCPP_INFO(node->get_logger(), "Please input csv file path\n");
        return 1;
    }
    
    std::string csv_file_path;
    csv_file_path = std::getenv("HOME");
    csv_file_path += "/corgi_ws/corgi_ros_ws/input_csv/";
    csv_file_path += argv[1];
    csv_file_path += ".csv";
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        RCLCPP_INFO(node->get_logger(), "Failed to open the CSV file\n");
        return 1;
    }

    std::string line;
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Starts\n");
    
    for (int i=0; i<5000; i++){
        std::getline(csv_file, line);
        std::vector<double> columns;
        std::stringstream ss(line);
        std::string item;
        
        for (auto& cmd : motor_cmds){
            std::getline(ss, item, ',');
            cmd->theta = std::stod(item);
            RCLCPP_INFO(node->get_logger(), item.c_str());
            std::getline(ss, item, ',');
            cmd->beta = std::stod(item);
            RCLCPP_INFO(node->get_logger(), item.c_str());

            cmd->kp_r = 90;
            cmd->kp_l = 90;
            cmd->ki_r = 0;
            cmd->ki_l = 0;
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
        motor_cmd.header.stamp = node->now();
        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        // rate.sleep();
        next_time += period;
        if(!node->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
    }
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Finished\n");

    
    while (rclcpp::ok()){
        rclcpp::spin_some(node);

        if (trigger){
            RCLCPP_INFO(node->get_logger(), "CSV Trajectory Starts\n");

            int seq = 0;
            next_time = node->now();
            while (rclcpp::ok() && std::getline(csv_file, line)) {
                std::vector<double> columns;
                std::stringstream ss(line);
                std::string item;
                
                for (auto& cmd : motor_cmds){
                    std::getline(ss, item, ',');
                    cmd->theta = std::stod(item);

                    std::getline(ss, item, ',');
                    cmd->beta = std::stod(item);

                    cmd->kp_r = 90;
                    cmd->kp_l = 90;
                    cmd->ki_r = 0;
                    cmd->ki_l = 0;
                    cmd->kd_r = 1.75;
                    cmd->kd_l = 1.75;
                }

                motor_cmd.header.seq = seq;

                motor_cmd_pub->publish(motor_cmd);

                seq++;

                // rate.sleep();
                next_time += period;
                if(!node->get_clock()->sleep_until(next_time)){
                    RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                    break;
                }
            }
            break;
        }
    }

    RCLCPP_INFO(node->get_logger(), "CSV Trajectory Finished\n");

    rclcpp::shutdown();
    
    return 0;
}
