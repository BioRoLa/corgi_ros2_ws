// ROS2 Online Contact Leg Estimator
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/imu_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/gmo_contact_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "common/Config.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "common/ParamsIO.hpp"

using namespace std::chrono_literals;

// Global node pointer for signal handler
rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * @brief Contact Leg Estimator Node
 */
class ContactLegEstimatorNode : public rclcpp::Node {
public:
    ContactLegEstimatorNode() 
        : Node(corgi::Config::NODE_NAME),
          params_([]() -> corgi::Params {
              try {
                  const std::string pkg =
                      ament_index_cpp::get_package_share_directory("corgi_odometry");
                  return corgi::load_params(pkg + "/config/config_online.yaml");
              } catch (const std::exception& e) {
                  RCLCPP_WARN(rclcpp::get_logger("contact_leg_estimator"),
                              "Config load failed (%s), using defaults", e.what());
                  return corgi::Params{};
              }
          }()),
          processor_(corgi::Config::DT, params_.encoder_cutoff_freq),
          observer_(
              corgi::Config::DT,
              params_.observer_cutoff_freq,
              corgi::Config::DOF,
              false,  // CSV logging
              ""      // No CSV filename
          )
    {
        contact_rm_threshold_high_   = params_.contact_rm_threshold_high;
        contact_rm_threshold_low_    = params_.contact_rm_threshold_low;
        contact_beta_threshold_high_ = params_.contact_beta_threshold_high;
        contact_beta_threshold_low_  = params_.contact_beta_threshold_low;
        // Create subscribers
        motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            corgi::Config::TOPIC_MOTOR_STATE,
            corgi::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::cb_motor_state, this, std::placeholders::_1)
        );
        
        imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
            corgi::Config::TOPIC_IMU,
            corgi::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::cb_imu, this, std::placeholders::_1)
        );
        
        position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            corgi::Config::TOPIC_ODOMETRY_POSITION,
            corgi::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::cb_position, this, std::placeholders::_1)
        );
        
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            corgi::Config::TOPIC_ODOMETRY_VELOCITY,
            corgi::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::cb_velocity, this, std::placeholders::_1)
        );
        
        trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
            corgi::Config::TOPIC_TRIGGER,
            corgi::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::cb_trigger, this, std::placeholders::_1)
        );

        // Create publishers
        contact_state_pub_ = this->create_publisher<corgi_msgs::msg::GMOContactStateStamped>(
            corgi::Config::TOPIC_CONTACT_STATE,
            corgi::Config::QUEUE_SIZE_PUB
        );
        
        RCLCPP_INFO(this->get_logger(), "Contact Leg Estimator Node Started");
        RCLCPP_INFO(this->get_logger(), "Loop rate: %.1f Hz", corgi::Config::ONLINE_LOOP_RATE);
        
        // Initialize data received flags
        motor_state_received_ = false;
        imu_received_ = false;
        position_received_ = false;
        velocity_received_ = false;
        is_triggered_ = false;
        
        iteration_count_ = 0;
    }
    
    // Main processing method (called from external loop)
    void process() {
        // Check if all data is available
        if (!motor_state_received_ || !imu_received_ || !position_received_ || !velocity_received_) {
            if (iteration_count_ % int(corgi::Config::ONLINE_LOOP_RATE) == 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "Waiting for data... Motor: %d, IMU: %d, Position: %d, Velocity: %d",
                    motor_state_received_, imu_received_, position_received_, velocity_received_);
            }
            iteration_count_++;
            return;
        }

        if (!is_triggered_) {
            if (iteration_count_ % int(corgi::Config::ONLINE_LOOP_RATE) == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting for trigger...");
            }
            iteration_count_++;
            // Reset observer when not triggered
            observer_.reset();
            return;
        }
        
        // Process data
        auto processed = processor_.process_realtime_data(position_, velocity_, imu_, motor_state_);
        
        // Estimate disturbance
        auto disturbance = observer_.estimate_disturbance(
            processed.q,
            processed.q_dot,
            processed.tau,
            processed.I_c,
            iteration_count_,
            true  // Don't print detailed info
        );
        
        // Publish contact state
        publish_contact_state(disturbance);
        
        iteration_count_++;
    }
    
private:
    // Callbacks for subscribers
    void cb_motor_state(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
        motor_state_time_ = rclcpp::Time(msg->header.stamp);
        motor_state_received_ = true;
    }
    
    void cb_imu(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
        imu_ = *msg;
        imu_time_ = rclcpp::Time(msg->header.stamp);
        imu_received_ = true;
    }
    
    void cb_position(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        position_ = *msg;
        position_time_ = this->now();  // Position topic has no header, use current time
        position_received_ = true;
    }
    
    void cb_velocity(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        velocity_ = *msg;
        velocity_time_ = this->now();  // Velocity topic has no header, use current time
        velocity_received_ = true;
    }

    void cb_trigger(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
        is_triggered_ = msg->enable;
    }
    
    /**
     * @brief Detect contact state using thershold and publish message
     * @param disturbance Disturbance observer output vector
     */
    inline void publish_contact_state(const Eigen::VectorXd& disturbance) {
        corgi_msgs::msg::GMOContactStateStamped contact_msg;
        contact_msg.header.stamp = this->now();

        // Indices for Rm forces in the disturbance vector
        constexpr int rm_force_indices[4] = {5, 7, 9, 11}; // LF, RF, RH, LH
        constexpr int beta_torque_indices[4] = {4, 6, 8, 10};

        // Schmitt trigger for contact detection
        for (int i = 0; i < 4; ++i) {
            double rm_force = disturbance(rm_force_indices[i]);
            double beta_torque = disturbance(beta_torque_indices[i]);
            
            if (!leg_contact_state_[i]) { // Currently no contact
                if (std::abs(rm_force) > contact_rm_threshold_high_ || std::abs(beta_torque) > contact_beta_threshold_high_) {
                    leg_contact_state_[i] = true;
                }
            } else { // Currently in contact
                if (std::abs(rm_force) < contact_rm_threshold_low_ && std::abs(beta_torque) < contact_beta_threshold_low_) {
                    leg_contact_state_[i] = false;
                }
            }
        }
        
        // Populate message
        contact_msg.module_a.contact = leg_contact_state_[0];
        contact_msg.module_a.rm_force = disturbance(rm_force_indices[0]);
        contact_msg.module_a.beta_torque = disturbance(beta_torque_indices[0]);

        contact_msg.module_b.contact = leg_contact_state_[1];
        contact_msg.module_b.rm_force = disturbance(rm_force_indices[1]);
        contact_msg.module_b.beta_torque = disturbance(beta_torque_indices[1]);

        contact_msg.module_c.contact = leg_contact_state_[2];
        contact_msg.module_c.rm_force = disturbance(rm_force_indices[2]);
        contact_msg.module_c.beta_torque = disturbance(beta_torque_indices[2]);

        contact_msg.module_d.contact = leg_contact_state_[3];
        contact_msg.module_d.rm_force = disturbance(rm_force_indices[3]);
        contact_msg.module_d.beta_torque = disturbance(beta_torque_indices[3]);

        contact_state_pub_->publish(contact_msg);
    }
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    rclcpp::Publisher<corgi_msgs::msg::GMOContactStateStamped>::SharedPtr contact_state_pub_;
    
    // Data storage
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ImuStamped imu_;
    geometry_msgs::msg::Vector3 position_;
    geometry_msgs::msg::Vector3 velocity_;
    
    // Message timestamps
    rclcpp::Time motor_state_time_;
    rclcpp::Time imu_time_;
    rclcpp::Time position_time_;
    rclcpp::Time velocity_time_;
    
    // Data received flags
    bool motor_state_received_;
    bool imu_received_;
    bool position_received_;
    bool velocity_received_;
    bool is_triggered_;
    
    // Leg contact states (for Schmitt trigger)
    std::array<bool, 4> leg_contact_state_{{false, false, false, false}};
    
    // Contact thresholds (loaded from YAML config)
    double contact_rm_threshold_high_   = 25.0;
    double contact_rm_threshold_low_    = 15.0;
    double contact_beta_threshold_high_ = 10.0;
    double contact_beta_threshold_low_  =  1.0;
    
    // YAML config + processing components
    // NOTE: params_ must be declared before processor_ and observer_
    //       so the member init list can use params_.* to init them.
    corgi::Params params_;
    DataProcessor processor_;
    corgi::DisturbanceObserver observer_;
    
    // Iteration counter
    size_t iteration_count_;
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
    
    g_node = std::make_shared<ContactLegEstimatorNode>();
    
    // Wait for clock sync if using simulation time
    bool use_sim_time = false;
    g_node->get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time) {
        wait_for_clock_sync(g_node);
    }
    
    // Calculate loop period based on configured rate
    rclcpp::Duration period(0, static_cast<int>(1e9 / corgi::Config::ONLINE_LOOP_RATE)); // in nanoseconds
    
    try {
        // Manual spin loop with controlled rate
        rclcpp::Time next_time = g_node->now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(g_node);  // Process all pending callbacks
            
            // Process data if available
            std::dynamic_pointer_cast<ContactLegEstimatorNode>(g_node)->process();
            
            // Sleep to maintain desired loop rate
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
