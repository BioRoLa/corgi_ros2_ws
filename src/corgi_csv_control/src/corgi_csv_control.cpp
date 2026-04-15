#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

bool trigger = false;
std::ofstream timing_log;

int64_t wall_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void timing_signal_handler(int signum) {
    if (timing_log.is_open()) {
        timing_log.flush();
        timing_log.close();
    }
    rclcpp::shutdown();
    exit(signum);
}

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_csv_control");

    // Only wait for Webots /clock when running in simulation mode.
    // In real-hardware mode (use_sim_time=false) the system wall clock is used
    // and node->now() is already valid — no need to wait.
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);

    if (use_sim_time) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (node->now().seconds() > 0.0) {
                RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "Real hardware mode: using system wall clock.");
    }
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);

    {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&t, &tm_buf);
        std::ostringstream timing_file_path;
        timing_file_path << std::string(getenv("HOME"))
                         << "/corgi_ws/corgi_ros2_ws/log_file/timing_csv_control_"
                         << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
                         << ".csv";

        timing_log.open(timing_file_path.str());
        if (timing_log.is_open()) {
            timing_log << "loop_wall_start_ns,seq,ros_stamp_sec,ros_stamp_nsec,"
                          "parse_ns,pre_publish_wall_ns,post_publish_wall_ns,publish_call_ns,"
                          "period_target_ns,period_actual_ns,deadline_slip_ns,sleep_ok\n";
            RCLCPP_INFO(node->get_logger(), "CSV control timing log: %s", timing_file_path.str().c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to open CSV control timing log: %s", timing_file_path.str().c_str());
        }
    }

    signal(SIGINT, timing_signal_handler);

    // rclcpp::Rate rate(1000);
    // use_sim_time setting
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = node->now();
    const int64_t period_target_ns = 1000000;
    int64_t prev_post_publish_wall_ns = 0;
    bool has_prev_post_publish = false;

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
    std::string input_arg = argv[1];

    // If absolute/relative path is provided, use it directly
    if (input_arg.find('/') != std::string::npos) {
        csv_file_path = input_arg;
        if (csv_file_path.size() < 4 || csv_file_path.substr(csv_file_path.size() - 4) != ".csv") {
            csv_file_path += ".csv";
        }
    } else {
        csv_file_path = std::getenv("HOME");
        csv_file_path += "/corgi_ws/corgi_ros2_ws/input_csv/";
        csv_file_path += input_arg;
        if (csv_file_path.size() < 4 || csv_file_path.substr(csv_file_path.size() - 4) != ".csv") {
            csv_file_path += ".csv";
        }
    }
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        RCLCPP_INFO(node->get_logger(), "Failed to open the CSV file\n");
        return 1;
    }

    std::string line;
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Starts\n");
    
    for (int i=0; i<5000; i++){
        int64_t loop_wall_start_ns = wall_now_ns();
        auto parse_start = std::chrono::steady_clock::now();

        std::getline(csv_file, line);
        std::vector<double> columns;
        std::stringstream ss(line);
        std::string item;
        
        for (auto& cmd : motor_cmds){
            std::getline(ss, item, ',');
            cmd->theta = std::stod(item);
            RCLCPP_DEBUG(node->get_logger(), item.c_str());
            std::getline(ss, item, ',');
            cmd->beta = std::stod(item);
            RCLCPP_DEBUG(node->get_logger(), item.c_str());

            cmd->kp_r = 90;
            cmd->kp_l = 90;
            cmd->ki_r = 0;
            cmd->ki_l = 0;
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
        int64_t parse_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - parse_start).count();

        motor_cmd.header.stamp = node->now();
        motor_cmd.header.seq = -4999+i;

        int32_t published_seq = motor_cmd.header.seq;
        int64_t deadline_slip_ns = wall_now_ns() - next_time.nanoseconds();
        int64_t pre_publish_wall_ns = wall_now_ns();
        auto publish_call_start = std::chrono::steady_clock::now();

        motor_cmd_pub->publish(motor_cmd);
        int64_t publish_call_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - publish_call_start).count();
        int64_t post_publish_wall_ns = wall_now_ns();
        int64_t period_actual_ns = has_prev_post_publish ? (post_publish_wall_ns - prev_post_publish_wall_ns) : 0;
        prev_post_publish_wall_ns = post_publish_wall_ns;
        has_prev_post_publish = true;

        next_time += period;
        bool sleep_ok = node->get_clock()->sleep_until(next_time);

        if (timing_log.is_open()) {
            timing_log << loop_wall_start_ns << ","
                       << published_seq << ","
                       << motor_cmd.header.stamp.sec << ","
                       << motor_cmd.header.stamp.nanosec << ","
                       << parse_ns << ","
                       << pre_publish_wall_ns << ","
                       << post_publish_wall_ns << ","
                       << publish_call_ns << ","
                       << period_target_ns << ","
                       << period_actual_ns << ","
                       << deadline_slip_ns << ","
                       << (sleep_ok ? 1 : 0) << "\n";
        }

        if(!sleep_ok){
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
    }
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Finished\n");

    
    while (rclcpp::ok() && !trigger) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "CSV Trajectory Starts\n");

        int seq = 0;
        next_time = node->now();
        while (rclcpp::ok() && std::getline(csv_file, line)) {
            int64_t loop_wall_start_ns = wall_now_ns();
            rclcpp::spin_some(node);

            if (!trigger) {
                RCLCPP_INFO(node->get_logger(), "Trigger False detect, CSV Trajectory pause\n");
                while (rclcpp::ok() && !trigger) {
                    rclcpp::spin_some(node);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
                if (!rclcpp::ok()) {
                    break;
                }
            }

            auto parse_start = std::chrono::steady_clock::now();
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
            int64_t parse_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - parse_start).count();

            motor_cmd.header.seq = seq;
            motor_cmd.header.stamp = node->now();

            int32_t published_seq = motor_cmd.header.seq;
            int64_t deadline_slip_ns = wall_now_ns() - next_time.nanoseconds();
            int64_t pre_publish_wall_ns = wall_now_ns();
            auto publish_call_start = std::chrono::steady_clock::now();

            motor_cmd_pub->publish(motor_cmd);
            int64_t publish_call_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - publish_call_start).count();
            int64_t post_publish_wall_ns = wall_now_ns();
            int64_t period_actual_ns = has_prev_post_publish ? (post_publish_wall_ns - prev_post_publish_wall_ns) : 0;
            prev_post_publish_wall_ns = post_publish_wall_ns;
            has_prev_post_publish = true;

            seq++;

            // rate.sleep();
            next_time += period;
            bool sleep_ok = node->get_clock()->sleep_until(next_time);

            if (timing_log.is_open()) {
                timing_log << loop_wall_start_ns << ","
                           << published_seq << ","
                           << motor_cmd.header.stamp.sec << ","
                           << motor_cmd.header.stamp.nanosec << ","
                           << parse_ns << ","
                           << pre_publish_wall_ns << ","
                           << post_publish_wall_ns << ","
                           << publish_call_ns << ","
                           << period_target_ns << ","
                           << period_actual_ns << ","
                           << deadline_slip_ns << ","
                           << (sleep_ok ? 1 : 0) << "\n";
            }

            if(!sleep_ok){
                RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "CSV Trajectory Finished\n");

    if (timing_log.is_open()) {
        timing_log.flush();
        timing_log.close();
    }

    rclcpp::shutdown();
    
    return 0;
}
