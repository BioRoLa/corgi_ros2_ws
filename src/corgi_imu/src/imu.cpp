#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "corgi_msgs/msg/headers.hpp"
#include "corgi_msgs/srv/imu.hpp"
#include "corgi_imu/cx5.hpp"
#include <sys/time.h>
#include <mutex>
#include <thread>
#include <memory>

std::shared_ptr<CX5_AHRS> imu;
std::mutex cb_lock;

enum SensorMode {
    REST = 0,
    CALIBRATION = 1,
    SENSOR = 2,
    RESET = 3
};

int mode = REST;

void handle_imu_service(
    const std::shared_ptr<corgi_msgs::srv::Imu::Request> req,
    std::shared_ptr<corgi_msgs::srv::Imu::Response> res,
    rclcpp::Logger logger)
{
    std::lock_guard<std::mutex> lock(cb_lock);
    switch (req->mode) {
        case REST:
            mode = REST;
            res->mode = REST;
            RCLCPP_INFO(logger, "Mode set to REST");
            break;
        case CALIBRATION:
            RCLCPP_INFO(logger, "Calibrating...");
            mode = SENSOR;
            res->mode = SENSOR;
            imu->calibrate(1000);  // 1 second averaging
            break;
        case RESET:
            mode = RESET;
            res->mode = RESET;
            RCLCPP_INFO(logger, "Mode set to RESET");
            break;
        case SENSOR:
            mode = SENSOR;
            res->mode = SENSOR;
            RCLCPP_INFO(logger, "Mode set to SENSOR");
            break;
        default:
            RCLCPP_INFO(logger, "Invalid mode");
            break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node");

    printf("Starting IMU node\n");

    imu = std::make_shared<CX5_AHRS>("/dev/ttyTHS1", 921600, 1000, 500);

    auto pub = node->create_publisher<corgi_msgs::msg::ImuStamped>("imu", 1000);

    auto srv = node->create_service<corgi_msgs::srv::Imu>(
        "imu_service",
        [node](const std::shared_ptr<corgi_msgs::srv::Imu::Request> req,
               std::shared_ptr<corgi_msgs::srv::Imu::Response> res) {
            handle_imu_service(req, res, node->get_logger());
        }
    );

    std::thread imu_thread([&]() {
        imu->start();
    });

    rclcpp::Rate rate(1000);

    corgi_msgs::msg::ImuStamped imu_msg;
    corgi_msgs::msg::Headers headers_msg;
    headers_msg.frame_id = "imu_base";

    Eigen::Vector3f acceleration, twist;
    Eigen::Quaternionf orientation;
    int seq = 0;

    while (rclcpp::ok()) {
        imu->get(acceleration, twist, orientation);

        timeval currentTime;
        gettimeofday(&currentTime, nullptr);

        headers_msg.seq = seq++;
        headers_msg.stamp.sec = currentTime.tv_sec;
        headers_msg.stamp.nanosec = currentTime.tv_usec * 1000;
        
        imu_msg.header = headers_msg;

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

        pub->publish(imu_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down IMU node...");
    imu->stop();
    imu_thread.join();
    rclcpp::shutdown();
    return 0;
}