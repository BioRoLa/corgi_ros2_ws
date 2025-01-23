#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "corgi_msgs/imu.h" // This is the service file
#include "corgi_msgs/Headers.h" // This is the header file
#include <fstream>
#include <mutex>

std::mutex mutex_;
sensor_msgs::Imu imu_info;
void imu_info_cb(sensor_msgs::Imu msg)
{
    mutex_.lock();
    imu_info = msg;
    mutex_.unlock();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_node_listener");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    ros::Subscriber sub = nh.subscribe("imu", 1000, imu_info_cb);
    std::ofstream file("imu.csv");
    file << "seq" << "," <<"t.sec" << "," << "t.usec" << "," << 
            "a.x" << "," << "a.y" << "," << "a.z" << "," << 
            "w.x" << "," << "w.y" << "," << "w.z" << "," << 
            "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w" << 
            "\n";
    while(1) { 
        ros::spinOnce();
        mutex_.lock();
        // std::cout << imu_info.orientation().x() << "\t" << imu_info.orientation().y() << "\t" << imu_info.orientation().z() << "\t" << imu_info.orientation().w() << "\n";
        // std::cout << imu_info.acceleration().x() << "\t" << imu_info.acceleration().y() << "\t" << imu_info.acceleration().z() << "\n";
        file << imu_info.header.seq << ",";
        file << imu_info.header.stamp.sec << ",";
        file << imu_info.header.stamp.nsec << ",";
        file << imu_info.linear_acceleration.x << ",";
        file << imu_info.linear_acceleration.y << ",";
        file << imu_info.linear_acceleration.z << ",";
        file << imu_info.angular_velocity.x << ",";
        file << imu_info.angular_velocity.y << ",";
        file << imu_info.angular_velocity.z << ",";
        file << imu_info.orientation.x << ",";
        file << imu_info.orientation.y << ",";
        file << imu_info.orientation.z << ",";
        file << imu_info.orientation.w << "\n";
        mutex_.unlock();
        rate.sleep();
    }
    file.close();
}