#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "cx5.hpp"
#include <sys/time.h>
#include <mutex>
#include <thread>

std::shared_ptr<CX5_AHRS> imu;
std::mutex cb_lock;
sensor_msg::SensorMode mode;

void cb(sensor_msg::SensorRequest request, sensor_msg::SensorReply &reply) {
    cb_lock.lock();
    switch (request.mode()) {
        case sensor_msg::REST:
            mode = sensor_msg::REST;
            reply.set_mode(sensor_msg::REST);
            break;
        case sensor_msg::CALIBRATION:
            imu->calibrate(1000);  // 1 second averaging
            mode = sensor_msg::SENSOR;
            break;
        case sensor_msg::RESET:
            mode = sensor_msg::RESET;
            reply.set_mode(sensor_msg::RESET);
            break;
        case sensor_msg::SENSOR:
            mode = sensor_msg::SENSOR;
            reply.set_mode(sensor_msg::SENSOR);
            break;
    }
    cb_lock.unlock();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;

    imu = std::make_shared<CX5_AHRS>("/dev/ttyTHS0", 921600, 1000, 500);
    ros::Rate rate(1000);

    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::ServiceServer srv = nh.advertiseService("imu_service", cb);

    std::thread imu_thread([&]() {
        imu->start();
    });

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu_base";

    Eigen::Vector3f acceleration, twist;
    Eigen::Quaternionf orientation;
    int seq = 0;

    while (ros::ok()) {
        imu->get(acceleration, twist, orientation);

        timeval currentTime;
        gettimeofday(&currentTime, nullptr);

        imu_msg.header.seq = seq++;
        imu_msg.header.stamp.sec = currentTime.tv_sec;
        imu_msg.header.stamp.nsec = currentTime.tv_usec * 1000;

        imu_msg.linear_acceleration.x = acceleration.x();
        imu_msg.linear_acceleration.y = acceleration.y();
        imu_msg.linear_acceleration.z = acceleration.z();
        imu_msg.angular_velocity.x = twist.x();
        imu_msg.angular_velocity.y = twist.y();
        imu_msg.angular_velocity.z = twist.z();
        imu_msg.orientation.x = orientation.x();
        imu_msg.orientation.y = orientation.y();
        imu_msg.orientation.z = orientation.z();
        imu_msg.orientation.w = orientation.w();

        pub.publish(imu_msg);

        rate.sleep();
    }

    return 0;
}
