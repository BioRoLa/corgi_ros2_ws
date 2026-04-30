#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/imu_raw_stamped.hpp"
#include "corgi_msgs/msg/headers.hpp"
#include "corgi_imu/cx5_raw.hpp"
#include <thread>
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_raw_node");

    printf("Starting IMU raw node (1000 Hz scaled sensor data)\n");

    auto imu = std::make_shared<CX5_RAW>("/dev/ttyTHS1", 921600, 1000);

    auto pub = node->create_publisher<corgi_msgs::msg::ImuRawStamped>("imu_raw", 5);

    // Capture steady_clock ↔ ROS time pair for timestamp conversion
    auto steady_epoch = std::chrono::steady_clock::now();
    rclcpp::Time ros_epoch = node->now();
    const int64_t ros_epoch_ns = ros_epoch.nanoseconds();

    // IMU driver thread (runs device->update() loop)
    std::thread imu_thread([&imu]() {
        imu->start();
    });

    // ROS executor thread (handles service callbacks independently)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    corgi_msgs::msg::ImuRawStamped msg;
    corgi_msgs::msg::Headers hdr;
    hdr.frame_id = "imu_base";

    Eigen::Vector3f accel, gyro;
    int seq = 0;

    // Main thread: tight data→publish loop, no spin_some overhead
    while (rclcpp::ok()) {
        std::chrono::steady_clock::time_point data_time;

        // Block until the next sensor sample arrives (≤5 ms timeout)
        if (!imu->wait_and_get(accel, gyro, data_time))
            continue;

        // Convert steady_clock → ROS time (nanosecond precision)
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(data_time - steady_epoch);
        int64_t stamp_ns = ros_epoch_ns + elapsed.count();
        hdr.stamp.sec     = static_cast<int32_t>(stamp_ns / 1000000000LL);
        hdr.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        hdr.seq = seq++;

        msg.header = hdr;

        msg.linear_acceleration.x = accel.x();
        msg.linear_acceleration.y = accel.y();
        msg.linear_acceleration.z = accel.z();

        msg.angular_velocity.x = gyro.x();
        msg.angular_velocity.y = gyro.y();
        msg.angular_velocity.z = gyro.z();

        pub->publish(msg);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down IMU raw node...");
    imu->stop();
    executor.cancel();
    ros_thread.join();
    imu_thread.join();
    rclcpp::shutdown();
    return 0;
}
