#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

// Publishes each phase transition on /experiment/phase so external tooling
// (e.g. experiment_entrypoint.sh) can wait on a real ROS signal instead of
// grepping this node's human-readable log output. transient_local durability
// means a late-joining subscriber still gets the most recent phase value
// immediately, even if it started subscribing after the message was sent.
void publish_phase(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
    const std::string &phase)
{
    std_msgs::msg::String msg;
    msg.data = phase;
    pub->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_csv_control");

    // Only wait for Webots /clock when running in simulation mode.
    // In real-hardware mode (use_sim_time=false) the system wall clock is used
    // and node->now() is already valid — no need to wait.
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);

    double leg_kp = node->declare_parameter("leg_kp", 120.0);
    double leg_kd = node->declare_parameter("leg_kd", 1.75);
    double leg_ki = node->declare_parameter("leg_ki", 0.0);
    double gamma_kp = node->declare_parameter("gamma_kp", 150.0);
    double gamma_kd = node->declare_parameter("gamma_kd", 1.75);

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
    auto phase_pub = node->create_publisher<std_msgs::msg::String>(
        "experiment/phase", rclcpp::QoS(1).transient_local());
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
    std::string input_arg = argv[1];

    // If absolute path (starts with '/'), use it directly; otherwise prepend input_csv dir
    if (!input_arg.empty() && input_arg[0] == '/') {
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
    publish_phase(phase_pub, "leg_transform_start");
    
    for (int i=0; i<5000; i++){
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

            cmd->kp_r = leg_kp;
            cmd->kp_l = leg_kp;
            cmd->kp_h = gamma_kp;
            cmd->ki_r = leg_ki;
            cmd->ki_l = leg_ki;
            cmd->ki_h = 0;
            cmd->kd_r = leg_kd;
            cmd->kd_l = leg_kd;
            cmd->kd_h = gamma_kd;
        }
        // Read 4 gamma (ABAD) columns: A_γ, B_γ, C_γ, D_γ
        for (auto& cmd : motor_cmds){
            std::getline(ss, item, ',');
            cmd->gamma = std::stod(item);
            RCLCPP_DEBUG(node->get_logger(), item.c_str());
        }
        motor_cmd.header.stamp = node->now();
        motor_cmd.header.seq = -4999+i;

        motor_cmd_pub->publish(motor_cmd);

        next_time += period;
        if(!node->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
    }
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Finished\n");
    publish_phase(phase_pub, "leg_transform_done");

    
    while (rclcpp::ok() && !trigger) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "CSV Trajectory Starts\n");
        publish_phase(phase_pub, "trajectory_start");

        int seq = 0;
        next_time = node->now();
        while (rclcpp::ok() && std::getline(csv_file, line)) {
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

            std::vector<double> columns;
            std::stringstream ss(line);
            std::string item;
            
            for (auto& cmd : motor_cmds){
                std::getline(ss, item, ',');
                cmd->theta = std::stod(item);

                std::getline(ss, item, ',');
                cmd->beta = std::stod(item);

                cmd->kp_r = leg_kp;
                cmd->kp_l = leg_kp;
                cmd->kp_h = gamma_kp;
                cmd->ki_r = leg_ki;
                cmd->ki_l = leg_ki;
                cmd->ki_h = 0;
                cmd->kd_r = leg_kd;
                cmd->kd_l = leg_kd;
                cmd->kd_h = gamma_kd;
            }
            // Read 4 gamma (ABAD) columns: A_γ, B_γ, C_γ, D_γ
            for (auto& cmd : motor_cmds){
                std::getline(ss, item, ',');
                cmd->gamma = std::stod(item);
            }

            motor_cmd.header.seq = seq;
            motor_cmd.header.stamp = node->now();

            motor_cmd_pub->publish(motor_cmd);

            seq++;

            // rate.sleep();
            next_time += period;
            if(!node->get_clock()->sleep_until(next_time)){
                RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "CSV Trajectory Finished\n");
    publish_phase(phase_pub, "trajectory_done");
    rclcpp::spin_some(node);  // give the transient_local publish a chance to go out before shutdown

    rclcpp::shutdown();
    
    return 0;
}
