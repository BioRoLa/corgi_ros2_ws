/**
 * @file imu_noise_sim.cpp
 * @brief ROS2 node that adds realistic IMU noise to simulation ground-truth IMU.
 *
 * Subscribes to clean ImuStamped from simulator, applies noise via
 * ImuNoiseSimulator, and republishes with the original timestamp preserved.
 *
 * Used in simulation launch files so that corgi_leg_odom (unchanged) receives
 * noisy IMU data matching real-robot characteristics.
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/imu_stamped.hpp>
#include "ImuNoiseSimulator.hpp"

class ImuNoiseSimNode : public rclcpp::Node {
public:
    ImuNoiseSimNode()
        : Node("imu_noise_sim")
    {
        // Declare parameters
        this->declare_parameter<int>("seed", 0);
        this->declare_parameter<double>("sample_rate", 1000.0);
        this->declare_parameter<std::string>("input_topic", "imu_raw");
        this->declare_parameter<std::string>("output_topic", "imu");

        int seed = this->get_parameter("seed").as_int();
        double sample_rate = this->get_parameter("sample_rate").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        double dt = 1.0 / sample_rate;
        noise_sim_ = std::make_unique<ImuNoiseSimulator>(dt, static_cast<uint64_t>(seed));

        // Publisher
        pub_ = this->create_publisher<corgi_msgs::msg::ImuStamped>(output_topic, 10);

        // Subscriber
        sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
            input_topic, 1,
            std::bind(&ImuNoiseSimNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Noise Simulator Node Started");
        RCLCPP_INFO(this->get_logger(), "  Input:  %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Sample rate: %.1f Hz, seed: %d", sample_rate, seed);
    }

private:
    void callback(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
        // Copy the message (preserves header, orientation, etc.)
        auto noisy_msg = *msg;

        // Extract acc/gyro into Eigen vectors
        Eigen::Vector3f acc(
            static_cast<float>(noisy_msg.linear_acceleration.x),
            static_cast<float>(noisy_msg.linear_acceleration.y),
            static_cast<float>(noisy_msg.linear_acceleration.z));
        Eigen::Vector3f gyro(
            static_cast<float>(noisy_msg.angular_velocity.x),
            static_cast<float>(noisy_msg.angular_velocity.y),
            static_cast<float>(noisy_msg.angular_velocity.z));

        // Apply noise
        noise_sim_->apply(acc, gyro);

        // Write back
        noisy_msg.linear_acceleration.x = acc.x();
        noisy_msg.linear_acceleration.y = acc.y();
        noisy_msg.linear_acceleration.z = acc.z();
        noisy_msg.angular_velocity.x = gyro.x();
        noisy_msg.angular_velocity.y = gyro.y();
        noisy_msg.angular_velocity.z = gyro.z();

        // Publish with original timestamp
        pub_->publish(noisy_msg);
    }

    std::unique_ptr<ImuNoiseSimulator> noise_sim_;
    rclcpp::Publisher<corgi_msgs::msg::ImuStamped>::SharedPtr pub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNoiseSimNode>());
    rclcpp::shutdown();
    return 0;
}
