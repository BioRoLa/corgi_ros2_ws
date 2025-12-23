#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include <fstream>
#include <mutex>
#include <sys/stat.h>

bool trigger = false;

std::mutex mutex_;
corgi_msgs::msg::ImuStamped imu_info;

std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";

bool file_exists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
{
    trigger = msg->enable;
    
    output_file_name = msg->output_filename;
    output_file_name += "_imu_data";

    if (trigger && msg->output_filename != "") {
        const char* home = std::getenv("HOME");
        output_file_path = std::string(home) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path + ".csv";
        while (file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + ".csv";
            index++;
        }
        if (index != 1) output_file_name += "_" + std::to_string(index - 1);
        output_file_name += ".csv";

        output_file_path = file_path_with_extension;

        if (!output_file.is_open()) {
            output_file.open(output_file_path);
            output_file << "seq" << "," << "t.sec" << "," << "t.nsec" << ","
                       << "a.x" << "," << "a.y" << "," << "a.z" << ","
                       << "w.x" << "," << "w.y" << "," << "w.z" << ","
                       << "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w"
                       << "\n";
            RCLCPP_INFO(rclcpp::get_logger("imu_node_listener"), "Recording imu data to %s", output_file_name.c_str());
        }
    }
    else {
        if (output_file.is_open()) {
            output_file.close();
            RCLCPP_INFO(rclcpp::get_logger("imu_node_listener"), "Stopped recording data");
        }
    }
}

void write_data()
{
    if (!output_file.is_open()) {
        if (output_file_name != "")
            RCLCPP_INFO(rclcpp::get_logger("imu_node_listener"), "Output file is not opened");
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    output_file << imu_info.header.seq << ","
                << imu_info.header.stamp.sec << ","
                << imu_info.header.stamp.nanosec << ","
                << imu_info.linear_acceleration.x << ","
                << imu_info.linear_acceleration.y << ","
                << imu_info.linear_acceleration.z << ","
                << imu_info.angular_velocity.x << ","
                << imu_info.angular_velocity.y << ","
                << imu_info.angular_velocity.z << ","
                << imu_info.orientation.x << ","
                << imu_info.orientation.y << ","
                << imu_info.orientation.z << ","
                << imu_info.orientation.w << "\n";
    output_file.flush();
}

void imu_info_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    imu_info = *msg;
}

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("imu_node_listener"), "IMU Listener Starts");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node_listener");

    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 1000, trigger_cb);
    
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 1000, imu_info_cb);

    rclcpp::Rate rate(1000);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (trigger) {
            write_data();
        }

        rate.sleep();
    }

    if (output_file.is_open()) {
        output_file.close();
    }

    rclcpp::shutdown();

    return 0;
}