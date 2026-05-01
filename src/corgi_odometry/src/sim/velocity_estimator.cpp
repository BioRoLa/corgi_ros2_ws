// ROS2 Velocity Estimator Node
// Subscribes to position, computes velocity via differentiation and filtering, then publishes

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <deque>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "common/Config.hpp"

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
 * Subscribes to TF (odom->base_link), differentiates and filters to compute velocity
 */
class VelocityEstimatorNode : public rclcpp::Node {
public:
    VelocityEstimatorNode() 
        : Node("velocity_estimator"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          vx_filter_(30.0, corgi::Config::ONLINE_LOOP_RATE),  // 30 Hz cutoff, 1000 Hz sample rate
          vy_filter_(30.0, corgi::Config::ONLINE_LOOP_RATE),
          vz_filter_(30.0, corgi::Config::ONLINE_LOOP_RATE),
          position_initialized_(false),
          last_position_x_(0.0),
          last_position_y_(0.0),
          last_position_z_(0.0)
    {
        // Declare parameters
        this->declare_parameter<double>("cutoff_freq", 30.0);
        this->declare_parameter<double>("sample_rate", corgi::Config::ONLINE_LOOP_RATE);
        this->declare_parameter<std::string>("velocity_topic", "sim/velocity");
        this->declare_parameter<std::string>("position_topic", "sim/position");
        this->declare_parameter<std::string>("parent_frame", "odom");
        this->declare_parameter<std::string>("child_frame", "base_link");
        
        // Get parameters
        double cutoff_freq = this->get_parameter("cutoff_freq").as_double();
        double sample_rate = this->get_parameter("sample_rate").as_double();
        std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
        std::string position_topic = this->get_parameter("position_topic").as_string();
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        
        // Reinitialize filters with parameters
        vx_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        vy_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        vz_filter_ = LowPassFilter(cutoff_freq, sample_rate);
        
        // Store sample rate for loop control
        sample_rate_ = sample_rate;
        
        // Create publishers
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            velocity_topic,
            10
        );
        
        position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            position_topic,
            10
        );

        body_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            corgi::Config::TOPIC_SIM_BODY_VELOCITY,
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Velocity Estimator Node Started");
        RCLCPP_INFO(this->get_logger(), "  Listening to TF: %s -> %s", parent_frame_.c_str(), child_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing velocity to: %s", velocity_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing position to: %s", position_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing body velocity to: %s", corgi::Config::TOPIC_SIM_BODY_VELOCITY);
        RCLCPP_INFO(this->get_logger(), "  Filter cutoff frequency: %.1f Hz", cutoff_freq);
        RCLCPP_INFO(this->get_logger(), "  Sample rate: %.1f Hz", sample_rate);
    }
    
    // Main processing method (called from external loop)
    void process() {
        geometry_msgs::msg::TransformStamped transform;
        
        try {
            // Look up the transform from parent_frame to child_frame
            transform = tf_buffer_.lookupTransform(
                parent_frame_, child_frame_,
                tf2::TimePointZero  // Get the latest available transform
            );
        } catch (const tf2::TransformException &ex) {
            // Only log warning periodically to avoid spam
            message_count_++;
            if (message_count_ % int(corgi::Config::ONLINE_LOOP_RATE) == 0) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            }
            return;
        }
        
        // Extract position and timestamp from transform
        double current_x = transform.transform.translation.x;
        double current_y = transform.transform.translation.y;
        double current_z = transform.transform.translation.z;
        rclcpp::Time current_time = rclcpp::Time(transform.header.stamp);
        // RCLCPP_INFO

        if (current_time.nanoseconds() == last_time_.nanoseconds()) {
            RCLCPP_DEBUG(this->get_logger(), "Received identical timestamp");
            RCLCPP_DEBUG(this->get_logger(), "  nanoseconds: %.9ld", current_time.nanoseconds());
            return;  // Skip this iteration
        }
                
        // Publish position (global position from TF)
        geometry_msgs::msg::Vector3 position_msg;
        position_msg.x = current_x;
        position_msg.y = current_y;
        position_msg.z = current_z;
        position_pub_->publish(position_msg);
        
        // Initialize previous position and time on first callback
        if (!position_initialized_) {
            last_position_x_ = current_x;
            last_position_y_ = current_y;
            last_position_z_ = current_z;
            last_time_ = current_time;
            position_initialized_ = true;
            RCLCPP_WARN(this->get_logger(), "Initialized position tracking.");
            return;
        }
        
        // Calculate time difference using TF timestamps
        double dt = (current_time - last_time_).seconds();
        
        // Avoid division by zero
        if (dt < 1e-7) {
            RCLCPP_WARN(this->get_logger(), "Time difference too small: dt=%.9f", dt);
            dt = 1e-6;
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

        // Convert odom-frame velocity to body-frame velocity using current yaw.
        const auto &q = transform.transform.rotation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        geometry_msgs::msg::Vector3 body_velocity_msg;
        body_velocity_msg.x = cos_yaw * filtered_vx + sin_yaw * filtered_vy;
        body_velocity_msg.y = -sin_yaw * filtered_vx + cos_yaw * filtered_vy;
        body_velocity_msg.z = filtered_vz;
        body_velocity_pub_->publish(body_velocity_msg);
        
        // Update previous values
        last_position_x_ = current_x;
        last_position_y_ = current_y;
        last_position_z_ = current_z;
        last_time_ = current_time;
        
        // Log periodically
        message_count_++;
        if (message_count_ % int(corgi::Config::ONLINE_LOOP_RATE) == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Velocity [m/s]: vx=%.3f, vy=%.3f, vz=%.3f", 
                filtered_vx, filtered_vy, filtered_vz);
        }
    }
    
    double get_sample_rate() const {
        return sample_rate_;
    }
    
private:
    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_velocity_pub_;
    
    // TF2 listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Frame names
    std::string parent_frame_;
    std::string child_frame_;
    
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
    size_t message_count_ = 0;
    double sample_rate_;
};

void handle_signal(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt received, shutting down...");
    }
    rclcpp::shutdown();
    exit(signum);
}

/**
 * @brief Wait for simulation clock to sync
 * @param node ROS2 node pointer
 */
void wait_for_clock_sync(const rclcpp::Node::SharedPtr& node) {
    RCLCPP_INFO(node->get_logger(), "Waiting for simulation clock...");
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    signal(SIGINT, handle_signal);
    
    g_node = std::make_shared<VelocityEstimatorNode>();
    
    // Wait for clock sync if using simulation time
    bool use_sim_time = false;
    g_node->get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time) {
        wait_for_clock_sync(g_node);
    }
    
    // Get sample rate for loop control
    auto node_ptr = std::dynamic_pointer_cast<VelocityEstimatorNode>(g_node);
    rclcpp::Duration period(0, static_cast<int>(1e9 / corgi::Config::ONLINE_LOOP_RATE));
    try {
        // Manual spin loop with controlled rate
        rclcpp::Time next_time = g_node->now();

        while (rclcpp::ok()) {
            rclcpp::spin_some(g_node);  // Process all pending callbacks
            
            // Process TF and compute velocity
            node_ptr->process();
            
            // // Sleep to maintain desired loop rate

            next_time += period;
            if(!g_node->get_clock()->sleep_until(next_time)){
                RCLCPP_WARN(g_node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
