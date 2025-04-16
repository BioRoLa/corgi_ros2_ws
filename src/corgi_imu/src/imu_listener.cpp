// #include "ros/ros.h"
// #include "sensor_msgs/Imu.h"
// #include "corgi_msgs/imu.h" // This is the service file
// #include "corgi_msgs/Headers.h" // This is the header file
// #include <fstream>
// #include <mutex>

// std::mutex mutex_;
// sensor_msgs::Imu imu_info;
// void imu_info_cb(sensor_msgs::Imu msg)
// {
//     mutex_.lock();
//     imu_info = msg;
//     mutex_.unlock();
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "imu_node_listener");
//     ros::NodeHandle nh;
//     ros::Rate rate(1000);
//     ros::Subscriber sub = nh.subscribe("imu", 1000, imu_info_cb);
//     std::ofstream file("imu.csv");
//     file << "seq" << "," <<"t.sec" << "," << "t.usec" << "," << 
//             "a.x" << "," << "a.y" << "," << "a.z" << "," << 
//             "w.x" << "," << "w.y" << "," << "w.z" << "," << 
//             "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w" << 
//             "\n";
//     while(ros::ok) { 
//         ros::spinOnce();
//         mutex_.lock();
//         // std::cout << imu_info.orientation().x() << "\t" << imu_info.orientation().y() << "\t" << imu_info.orientation().z() << "\t" << imu_info.orientation().w() << "\n";
//         // std::cout << imu_info.acceleration().x() << "\t" << imu_info.acceleration().y() << "\t" << imu_info.acceleration().z() << "\n";
//         file << imu_info.header.seq << ",";
//         file << imu_info.header.stamp.sec << ",";
//         file << imu_info.header.stamp.nsec << ",";
//         file << imu_info.linear_acceleration.x << ",";
//         file << imu_info.linear_acceleration.y << ",";
//         file << imu_info.linear_acceleration.z << ",";
//         file << imu_info.angular_velocity.x << ",";
//         file << imu_info.angular_velocity.y << ",";
//         file << imu_info.angular_velocity.z << ",";
//         file << imu_info.orientation.x << ",";
//         file << imu_info.orientation.y << ",";
//         file << imu_info.orientation.z << ",";
//         file << imu_info.orientation.w << "\n";
//         mutex_.unlock();
//         rate.sleep();
//     }
//     file.close();
// }

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "corgi_msgs/imu.h" // This is the service file
#include "corgi_msgs/Headers.h" // This is the header file
#include "corgi_msgs/TriggerStamped.h"
#include <fstream>
#include <mutex>
#include <sys/stat.h>

bool trigger = false;


std::mutex mutex_;
sensor_msgs::Imu imu_info;

std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";

bool file_exists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
    
    output_file_name = msg.output_filename;

    output_file_name += + "_imu_data";

    if (trigger && msg.output_filename != "") {
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path +".csv";
        while (file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + ".csv";
            index++;
        }
        if (index != 1) output_file_name += "_" + std::to_string(index-1);
        output_file_name += ".csv";

        output_file_path = file_path_with_extension;

        if (!output_file.is_open()) {
            output_file.open(output_file_path);
            output_file << "seq" << "," <<"t.sec" << "," << "t.usec" << "," << 
            "a.x" << "," << "a.y" << "," << "a.z" << "," << 
            "w.x" << "," << "w.y" << "," << "w.z" << "," << 
            "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w" << 
            "\n";
            ROS_INFO("Recording imu data to %s\n", output_file_name.c_str());
        }
    }
    else {
        if (output_file.is_open()) {
            output_file.close();
            ROS_INFO("Stopped recording data\n");
        }
    }
}

void write_data() {
    if (!output_file.is_open()){
        if (output_file_name != "") ROS_INFO("Output file is not opened\n");
        return;
    }
    mutex_.lock();
    // std::cout << imu_info.orientation().x() << "\t" << imu_info.orientation().y() << "\t" << imu_info.orientation().z() << "\t" << imu_info.orientation().w() << "\n";
    // std::cout << imu_info.acceleration().x() << "\t" << imu_info.acceleration().y() << "\t" << imu_info.acceleration().z() << "\n";
    output_file << imu_info.header.seq << ","
                << imu_info.header.stamp.sec << ","
                << imu_info.header.stamp.nsec << ","
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
    mutex_.unlock();

    output_file.flush();

}

void imu_info_cb(sensor_msgs::Imu msg)
{
    mutex_.lock();
    imu_info = msg;
    mutex_.unlock();
}

int main(int argc, char **argv) {
    ROS_INFO("IMU Listener Starts\n");

    ros::init(argc, argv, "imu_node_listener");

    ros::NodeHandle nh;
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);
    ros::Subscriber sub = nh.subscribe("imu", 1000, imu_info_cb);

    while(ros::ok()) { 
        ros::spinOnce();

        if (trigger) {
            write_data();
        }
        rate.sleep();
    }

    if (output_file.is_open()) {
        output_file.close();
    }

    ros::shutdown();

    return 0;
}