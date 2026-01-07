// ROS2 Online Contact Leg Estimator
#include <iostream>
#include <cmath>
#include <chrono>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/contact_state.hpp>
#include "DisturbanceObserver.hpp"
#include "Config.hpp"

using namespace std::chrono_literals;

// Global node pointer for signal handler
rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * @brief Data Processor for real-time processing
 * Converts ROS messages to observer input format
 */
class DataProcessor {
public:
    DataProcessor(double dt) : dt_(dt) {
        // No need for state tracking since velocities come from motor_state
    }
    
    struct ProcessedData {
        Eigen::VectorXd q;      // (12,) - generalized coordinates
        Eigen::VectorXd q_dot;  // (12,) - generalized velocities
        Eigen::VectorXd tau;    // (8,) - joint torques
        Eigen::VectorXd I_c;    // (4,) - leg inertias
    };
    
    /**
     * @brief Process real-time data from ROS messages
     */
    ProcessedData process_realtime_data(
        const geometry_msgs::msg::Vector3& position,
        const geometry_msgs::msg::Vector3& velocity,
        const sensor_msgs::msg::Imu& imu,
        const corgi_msgs::msg::MotorStateStamped& motor_state
    ) {
        ProcessedData result;
        
        // Initialize vectors
        result.q = Eigen::VectorXd::Zero(12);
        result.q_dot = Eigen::VectorXd::Zero(12);
        result.tau = Eigen::VectorXd(8);
        result.I_c = Eigen::VectorXd(4);
        
        // Base position (x, z)
        result.q(0) = position.x;
        result.q(1) = position.z;
        
        // Base velocities from subscribed velocity topic
        result.q_dot(0) = velocity.x;
        result.q_dot(1) = velocity.z;
        
        // IMU to Euler angles (Quaternion -> Euler)
        double roll, pitch;
        quaternion_to_euler(
            imu.orientation.x, imu.orientation.y,
            imu.orientation.z, imu.orientation.w,
            roll, pitch
        );
        result.q(2) = roll;
        result.q(3) = pitch;
        
        // Base angular velocity (from IMU)
        result.q_dot(2) = imu.angular_velocity.x;  // roll rate
        result.q_dot(3) = imu.angular_velocity.y;  // pitch rate
        
        // Extract motor states
        double theta_a = motor_state.module_a.theta;
        double beta_a = motor_state.module_a.beta;
        double theta_b = motor_state.module_b.theta;
        double beta_b = motor_state.module_b.beta;
        double theta_c = motor_state.module_c.theta;
        double beta_c = motor_state.module_c.beta;
        double theta_d = motor_state.module_d.theta;
        double beta_d = motor_state.module_d.beta;
        
        // Leg joint angles - Convert theta to Rm
        result.q(4) = beta_a;
        result.q(5) = theta_to_Rm(theta_a);
        result.q(6) = -beta_b;  // RF needs negation
        result.q(7) = theta_to_Rm(theta_b);
        result.q(8) = -beta_c;  // RH needs negation
        result.q(9) = theta_to_Rm(theta_c);
        result.q(10) = beta_d;
        result.q(11) = theta_to_Rm(theta_d);
        
        // Leg joint velocities from motor_state (velocity_r, velocity_l)
        // Convert motor velocities to joint velocities: phi_r, phi_l -> theta, beta
        // theta_dot = (- velocity_r + velocity_l)/2, beta_dot = (velocity_r + velocity_l)/2
        double theta_a_dot = (- motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
        double beta_a_dot = (motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
        
        double theta_b_dot = (- motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
        double beta_b_dot = (motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
        beta_b_dot = -beta_b_dot;  // RF beta velocity needs negation
        
        double theta_c_dot = (- motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
        double beta_c_dot = (motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
        beta_c_dot = -beta_c_dot;  // RH beta velocity needs negation
        
        double theta_d_dot = (- motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
        double beta_d_dot = (motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
        
        // Convert theta_dot to Rm_dot
        result.q_dot(4) = beta_a_dot;
        result.q_dot(5) = theta_dot_to_Rm_dot(theta_a, theta_a_dot);
        result.q_dot(6) = beta_b_dot;
        result.q_dot(7) = theta_dot_to_Rm_dot(theta_b, theta_b_dot);
        result.q_dot(8) = beta_c_dot;
        result.q_dot(9) = theta_dot_to_Rm_dot(theta_c, theta_c_dot);
        result.q_dot(10) = beta_d_dot;
        result.q_dot(11) = theta_dot_to_Rm_dot(theta_d, theta_d_dot);
        
        // Convert motor torques to joint space torques
        double torque_beta_a, F_Rm_a;
        calculate_motor_to_joint_torque(theta_a, 
            motor_state.module_a.torque_r, motor_state.module_a.torque_l, 
            torque_beta_a, F_Rm_a);
        
        double torque_beta_b, F_Rm_b;
        calculate_motor_to_joint_torque(theta_b,
            motor_state.module_b.torque_r, motor_state.module_b.torque_l,
            torque_beta_b, F_Rm_b);
        
        double torque_beta_c, F_Rm_c;
        calculate_motor_to_joint_torque(theta_c,
            motor_state.module_c.torque_r, motor_state.module_c.torque_l,
            torque_beta_c, F_Rm_c);
        
        double torque_beta_d, F_Rm_d;
        calculate_motor_to_joint_torque(theta_d,
            motor_state.module_d.torque_r, motor_state.module_d.torque_l,
            torque_beta_d, F_Rm_d);
        
        // Joint torques in generalized coordinates
        result.tau(0) = torque_beta_a;
        result.tau(1) = F_Rm_a;
        result.tau(2) = -torque_beta_b;  // RF beta torque needs negation
        result.tau(3) = F_Rm_b;
        result.tau(4) = -torque_beta_c;  // RH beta torque needs negation
        result.tau(5) = F_Rm_c;
        result.tau(6) = torque_beta_d;
        result.tau(7) = F_Rm_d;
        
        // Compute leg inertia
        result.I_c(0) = theta_to_Ic(theta_a);
        result.I_c(1) = theta_to_Ic(theta_b);
        result.I_c(2) = theta_to_Ic(theta_c);
        result.I_c(3) = theta_to_Ic(theta_d);
        
        return result;
    }
    
private:
    double dt_;
    
    void quaternion_to_euler(double x, double y, double z, double w, double& roll, double& pitch) {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);
    }
    
    double theta_to_Rm(double theta) {
        const auto& A = quadruped::Config::RM_COEFF;
        double Rm = 0.0;
        double theta_power = 1.0;
        for (int i = A.size() - 1; i >= 0; --i) {
            Rm += A[i] * theta_power;
            theta_power *= theta;
        }
        return Rm;
    }
    
    double theta_dot_to_Rm_dot(double theta, double theta_dot) {
        const auto& A = quadruped::Config::RM_COEFF;
        double dRm_dtheta = 0.0;
        double theta_power = 1.0;
        
        for (size_t i = 1; i < A.size(); ++i) {
            dRm_dtheta += A[i] * (A.size() - 1 - i) * theta_power;
            theta_power *= theta;
        }
        
        return dRm_dtheta * theta_dot;
    }
    
    double theta_to_Ic(double theta) {
        const auto& B = quadruped::Config::IC_COEFF;
        double Ic = 0.0;
        double theta_power = 1.0;
        for (int i = B.size() - 1; i >= 0; --i) {
            Ic += B[i] * theta_power;
            theta_power *= theta;
        }
        return Ic;
    }
    
    void calculate_motor_to_joint_torque(double theta, double torque_right, double torque_left,
                                        double& torque_beta, double& force_Rm) {
        const auto& A = quadruped::Config::RM_COEFF;
        double dRm_dtheta = 0.0;
        double theta_power = 1.0;
        
        for (size_t i = 1; i < A.size(); ++i) {
            dRm_dtheta += A[i] * (A.size() - 1 - i) * theta_power;
            theta_power *= theta;
        }
        
        // Virtual work transformation
        double temp1 = torque_left - torque_right;
        double temp2 = torque_left + torque_right;
        
        if (std::abs(dRm_dtheta) < 1e-9) {
            force_Rm = 0.0;
            torque_beta = 0.0;
            return;
        }
        
        force_Rm = temp1 / dRm_dtheta;
        torque_beta = temp2;
    }
};

/**
 * @brief Contact Leg Estimator Node
 */
class ContactLegEstimatorNode : public rclcpp::Node {
public:
    ContactLegEstimatorNode() 
        : Node(quadruped::Config::NODE_NAME),
          processor_(quadruped::Config::DT),
          observer_(
              quadruped::Config::DT,
              quadruped::Config::OBSERVER_CUTOFF_FREQ,
              quadruped::Config::DOF,
              false,  // Disable CSV logging in online mode
              ""      // No CSV filename
          )
    {
        // Create subscribers
        motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            quadruped::Config::TOPIC_MOTOR_STATE,
            quadruped::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::motor_state_callback, this, std::placeholders::_1)
        );
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            quadruped::Config::TOPIC_IMU,
            quadruped::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::imu_callback, this, std::placeholders::_1)
        );
        
        position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            quadruped::Config::TOPIC_ODOMETRY_POSITION,
            quadruped::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::position_callback, this, std::placeholders::_1)
        );
        
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            quadruped::Config::TOPIC_ODOMETRY_VELOCITY,
            quadruped::Config::QUEUE_SIZE_SUB,
            std::bind(&ContactLegEstimatorNode::velocity_callback, this, std::placeholders::_1)
        );
        
        // Create publishers
        contact_state_pub_ = this->create_publisher<corgi_msgs::msg::ContactState>(
            quadruped::Config::TOPIC_CONTACT_STATE,
            quadruped::Config::QUEUE_SIZE_PUB
        );
        
        RCLCPP_INFO(this->get_logger(), "Contact Leg Estimator Node Started");
        RCLCPP_INFO(this->get_logger(), "Loop rate: %.1f Hz", quadruped::Config::ONLINE_LOOP_RATE);
        
        // Create timer for main processing loop
        auto period = std::chrono::microseconds(static_cast<int>(1e6 / quadruped::Config::ONLINE_LOOP_RATE));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&ContactLegEstimatorNode::timer_callback, this)
        );
        
        // Initialize data received flags
        motor_state_received_ = false;
        imu_received_ = false;
        position_received_ = false;
        velocity_received_ = false;
        
        iteration_count_ = 0;
    }
    
private:
    // Callbacks for subscribers
    void motor_state_callback(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
        motor_state_received_ = true;
    }
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_ = *msg;
        imu_received_ = true;
    }
    
    void position_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        position_ = *msg;
        position_received_ = true;
    }
    
    void velocity_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        velocity_ = *msg;
        velocity_received_ = true;
    }
    
    // Main processing loop
    void timer_callback() {
        // Check if all data is available
        if (!motor_state_received_ || !imu_received_ || !position_received_ || !velocity_received_) {
            if (iteration_count_ % 1000 == 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "Waiting for data... Motor: %d, IMU: %d, Position: %d, Velocity: %d",
                    motor_state_received_, imu_received_, position_received_, velocity_received_);
            }
            iteration_count_++;
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
            false  // Don't print detailed info
        );
        
        // // Publish contact state (simplified - just use disturbance magnitude as indicator)
        // // TODO: Implement proper contact detection logic
        // corgi_msgs::msg::ContactState contact_msg;
        
        // // Simple threshold-based contact detection
        // double threshold = 5.0;  // N - adjust based on actual data //FIXME: Tune threshold
        // contact_msg.contact = (std::abs(disturbance(2)) > threshold ||
        //                       std::abs(disturbance(5)) > threshold ||
        //                       std::abs(disturbance(8)) > threshold ||
        //                       std::abs(disturbance(11)) > threshold);
        // contact_msg.score = disturbance.norm();
        
        // contact_state_pub_->publish(contact_msg);
        
        // iteration_count_++;
        
        // // Log periodically
        // if (iteration_count_ % 1000 == 0) {
        //     RCLCPP_INFO(this->get_logger(), 
        //         "Processed %ld iterations, Contact: %d, Score: %.3f",
        //         iteration_count_, contact_msg.contact, contact_msg.score);
        // }
    }
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub_;
    rclcpp::Publisher<corgi_msgs::msg::ContactState>::SharedPtr contact_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data storage
    corgi_msgs::msg::MotorStateStamped motor_state_;
    sensor_msgs::msg::Imu imu_;
    geometry_msgs::msg::Vector3 position_;
    geometry_msgs::msg::Vector3 velocity_;
    
    // Data received flags
    bool motor_state_received_;
    bool imu_received_;
    bool position_received_;
    bool velocity_received_;
    
    // Processing components
    DataProcessor processor_;
    quadruped::DisturbanceObserver observer_;
    
    // Iteration counter
    size_t iteration_count_;
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
    
    g_node = std::make_shared<ContactLegEstimatorNode>();
    
    try {
        rclcpp::spin(g_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
