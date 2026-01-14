// ROS2 Velocity Estimator Node
// Subscribes to position, computes velocity via differentiation and filtering, then publishes

#include <iostream>
#include <cmath>
#include <chrono>
#include <signal.h>
#include <deque>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/sim_data_stamped.hpp>

using namespace std::chrono_literals;

// Global node pointer for signal handler
rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * @brief Low-pass filter for velocity estimation
 */
class LowPassFilter {
public:
    LowPassFilter(double cutoff_freq, double sample_rate)
        : alpha_(0.0), initialized_(false), filtered_value_(0.0)
    {
        // Calculate filter coefficient
        double rc = 1.0 / (2.0 * M_PI * cutoff_freq);
        double dt = 1.0 / sample_rate;
        alpha_ = dt / (rc + dt);
    }
    
    double filter(double raw_value) {
        if (!initialized_) {
            filtered_value_ = raw_value;
            initialized_ = true;
            return filtered_value_;
        }
        
        // First-order low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        filtered_value_ = alpha_ * raw_value + (1.0 - alpha_) * filtered_value_;
        return filtered_value_;
    }
    
    void reset() {
        initialized_ = false;
        filtered_value_ = 0.0;
    }
    
    double get_filtered_value() const {
        return filtered_value_;
    }
    
private:
    double alpha_;
    bool initialized_;
    double filtered_value_;
};

/**
 * @brief Velocity Estimator Node
 * Subscribes to position (from SimDataStamped), differentiates and filters to compute velocity
 */
class VelocityEstimatorNode : public rclcpp::Node {
public:
    VelocityEstimatorNode() 
        : Node("velocity_estimator"),
          vx_filter_(30.0, 1000.0),  // 30 Hz cutoff, 1000 Hz sample rate
          vy_filter_(30.0, 1000.0),
          vz_filter_(30.0, 1000.0),
          position_initialized_(false),
          last_position_x_(0.0),
          last_position_y_(0.0),
          last_position_z_(0.0)
    {
        // Declare parameters
        this->declare_parameter<double>("cutoff_freq", 30.0);
        this->declare_parameter<double>("sample_rate", 1000.0);
        this->declare_parameter<std::string>("position_topic", "sim/data");
        this->declare_parameter<std::string>("velocity_topic", "odometry/velocity");
        this->declare_parameter<std::string>("position_output_topic", "odometry/position");
        
        // Get parameters
        double cutoff_freq = this->get_parameter("cutoff_freq").as_double();
        double sample_rate = this->get_parameter("sample_rate").as_double();
        std::string position_topic = this->get_parameter("position_topic").as_string();
        std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
        std::string position_output_topic = this->get_parameter("position_output_topic").as_string();
        
        // Reinitialize filters with parameters
        vx_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        vy_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        vz_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        
        dt_ = 1.0 / sample_rate;
        
        // Create subscriber to SimDataStamped
        sim_data_sub_ = this->create_subscription<corgi_msgs::msg::SimDataStamped>(
            position_topic,
            10,
            std::bind(&VelocityEstimatorNode::sim_data_callback, this, std::placeholders::_1)
        );
        
        // Create publishers
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            velocity_topic,
            10
        );
        
        position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            position_output_topic,
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Velocity Estimator Node Started");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", position_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing velocity to: %s", velocity_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing position to: %s", position_output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Filter cutoff frequency: %.1f Hz", cutoff_freq);
        RCLCPP_INFO(this->get_logger(), "  Sample rate: %.1f Hz", sample_rate);
    }
    
private:
    void sim_data_callback(const corgi_msgs::msg::SimDataStamped::SharedPtr msg) {
        // Extract position from message
        double current_x = msg->position.x;
        double current_y = msg->position.y;
        double current_z = msg->position.z;
        
        // Publish position (extracted from SimDataStamped)
        geometry_msgs::msg::Vector3 position_msg;
        position_msg.x = current_x;
        position_msg.y = current_y;
        position_msg.z = current_z;
        position_pub_->publish(position_msg);
        
        // Initialize previous position on first callback
        if (!position_initialized_) {
            last_position_x_ = current_x;
            last_position_y_ = current_y;
            last_position_z_ = current_z;
            last_time_ = msg->header.stamp;
            position_initialized_ = true;
            return;
        }
        
        // Calculate time difference
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        
        // Avoid division by zero
        if (dt < 1e-6) {
            return;
        }
        
        // Compute raw velocities via differentiation
        double raw_vx = (current_x - last_position_x_) / dt;
        double raw_vy = (current_y - last_position_y_) / dt;
        double raw_vz = (current_z - last_position_z_) / dt;
        
        // Apply low-pass filtering
        double filtered_vx = vx_filter_.filter(raw_vx);
        double filtered_vy = vy_filter_.filter(raw_vy);
        double filtered_vz = vz_filter_.filter(raw_vz);
        
        // Publish filtered velocity
        geometry_msgs::msg::Vector3 velocity_msg;
        velocity_msg.x = filtered_vx;
        velocity_msg.y = filtered_vy;
        velocity_msg.z = filtered_vz;
        velocity_pub_->publish(velocity_msg);
        
        // Update previous values
        last_position_x_ = current_x;
        last_position_y_ = current_y;
        last_position_z_ = current_z;
        last_time_ = current_time;
        
        // Log periodically
        message_count_++;
        if (message_count_ % 1000 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Velocity [m/s]: vx=%.3f, vy=%.3f, vz=%.3f", 
                filtered_vx, filtered_vy, filtered_vz);
        }
    }
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<corgi_msgs::msg::SimDataStamped>::SharedPtr sim_data_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    
    // Low-pass filters
    LowPassFilter vx_filter_;
    LowPassFilter vy_filter_;
    LowPassFilter vz_filter_;
    
    // State tracking
    bool position_initialized_;
    double last_position_x_;
    double last_position_y_;
    double last_position_z_;
    rclcpp::Time last_time_;
    double dt_;
    size_t message_count_ = 0;
};

void signal_handler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt received, shutting down...");
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    signal(SIGINT, signal_handler);
    
    g_node = std::make_shared<VelocityEstimatorNode>();
    
    try {
        rclcpp::spin(g_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
