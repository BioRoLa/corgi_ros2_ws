#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "corgi_msgs/msg/headers.hpp"
#include "corgi_msgs/srv/imu.hpp"
#include "corgi_imu/cv7.hpp"
#include <sys/time.h>
#include <mutex>
#include <thread>
#include <memory>

std::shared_ptr<CV7_AHRS> imu;
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

    // Rollback / A-B only: the pre-2026-11-16 quaternion (pitch and yaw
    // mirrored against the gyro). Consumers no longer compensate for it.
    const bool legacy_orientation = node->declare_parameter<bool>("legacy_orientation", false);
    if (legacy_orientation) {
        RCLCPP_WARN(node->get_logger(),
            "LEGACY ORIENTATION: quaternion pitch and yaw are mirrored against the gyro. "
            "force_estimation, force_control and gslip_pronk expect the corrected frame.");
    } else {
        RCLCPP_INFO(node->get_logger(),
            "IMU frame: x forward, y left, z up; orientation and angular_velocity agree in sign "
            "(CONVENTIONS.md).");
    }

    imu = std::make_shared<CV7_AHRS>("/dev/ttyTHS1", 921600, 1000, 500);

    auto pub = node->create_publisher<corgi_msgs::msg::ImuStamped>("imu", 1);

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

        // /imu convention (CONVENTIONS.md): body frame x forward, y left,
        // z up; orientation = that body frame in a z-up world, so roll,
        // pitch, yaw are rotations about +x, +y, +z and agree in sign with
        // angular_velocity (nose DOWN is +pitch and +wy).
        //
        // Vectors: the driver hands over (x, -y, -z) of the sensor's axes and
        // this flips x and z, so body = Rz(pi) * sensor. Checked on the robot:
        // the published rate passes the push test and Vicon as FLU.
        imu_msg.linear_acceleration.x = -acceleration.x();
        imu_msg.linear_acceleration.y = acceleration.y();
        imu_msg.linear_acceleration.z = -acceleration.z();

        imu_msg.angular_velocity.x = -twist.x();
        imu_msg.angular_velocity.y = twist.y();
        imu_msg.angular_velocity.z = -twist.z();

        // Orientation: q_raw is the sensor in the device's z-DOWN world.
        // Until 2026-11-16 this published q_raw * Ry(pi), which left the
        // world z-down: pitch and yaw came out mirrored against the gyro
        // (hardware bags 2026-09-04 and 2026-10-29: quaternion-derived rate
        // vs published rate r = +0.95 on x, -0.98 on y, -0.99 on z; vault
        // Open Issues #47) and every consumer patched it differently.
        // Correct: q_body = Rx(pi) * q_raw * Rz(pi) -- Rx(pi) turns the world
        // z-up, Rz(pi) is the same mount the vectors use. That equals the old
        // output with y and z negated, which is how it is written here.
        const Eigen::Quaternionf q_mount(0.0f, 0.0f, 1.0f, 0.0f);
        Eigen::Quaternionf body_orientation = (orientation * q_mount).normalized();
        if (!legacy_orientation) {
            body_orientation.y() = -body_orientation.y();
            body_orientation.z() = -body_orientation.z();
        }
        imu_msg.orientation.x = body_orientation.x();
        imu_msg.orientation.y = body_orientation.y();
        imu_msg.orientation.z = body_orientation.z();
        imu_msg.orientation.w = body_orientation.w();

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
