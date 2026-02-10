#ifndef LEG_ODOMETRY_NODE_HPP
#define LEG_ODOMETRY_NODE_HPP

#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/imu_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/contact_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>

#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "kinematic/ContactMap.hpp"
#include "Config.hpp"

/**
 * @brief Leg Odometry Node
 *
 * Responsibilities:
 *   1. Subscribe to IMU, motor state, position, velocity, trigger
 *   2. Run disturbance observer → publish contact state
 *   3. (TODO) Run ES-EKF for odometry estimation → publish odometry
 */
class LegOdometryNode : public rclcpp::Node {
public:
    LegOdometryNode();

    /// @brief Main processing method, called from the external spin loop
    void process();

private:
    // ============================================================
    // ROS callbacks
    // ============================================================
    void cb_motor_state(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void cb_imu(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
    void cb_position(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void cb_velocity(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void cb_trigger(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);

    // ============================================================
    // Contact estimation
    // ============================================================

    /// @brief Schmitt-trigger contact detection + publish contact state
    void publish_contact_state(const Eigen::VectorXd& disturbance);

    // ============================================================
    // Leg factory
    // ============================================================
    static Leg createLeg(double x_sign, double y_sign);

    // ============================================================
    // Leg kinematic models  (order: LF, RF, RH, LH)
    // ============================================================
    Leg lf_leg_ = createLeg( 1,  1);
    Leg rf_leg_ = createLeg( 1, -1);
    Leg rh_leg_ = createLeg(-1, -1);
    Leg lh_leg_ = createLeg(-1,  1);

    // ============================================================
    // ROS subscribers & publishers
    // ============================================================
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr          imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          velocity_sub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr      trigger_sub_;

    rclcpp::Publisher<corgi_msgs::msg::ContactStateStamped>::SharedPtr    contact_state_pub_;
    // TODO: Add nav_msgs/Odometry publisher

    // ============================================================
    // Latest message storage
    // ============================================================
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ImuStamped        imu_;
    geometry_msgs::msg::Vector3        position_;
    geometry_msgs::msg::Vector3        velocity_;

    // ============================================================
    // Data-ready flags
    // ============================================================
    bool motor_state_received_ = false;
    bool imu_received_         = false;
    bool position_received_    = false;
    bool velocity_received_    = false;
    bool is_triggered_         = false;

    // ============================================================
    // Contact state (Schmitt trigger memory)
    // ============================================================
    std::array<bool, 4> leg_contact_state_{{false, false, false, false}};

    // ============================================================
    // Processing components
    // ============================================================
    DataProcessor              processor_;
    corgi::DisturbanceObserver observer_;

    // TODO: Add ESEKF instance
    // TODO: Add ContactMap instances (one per leg)

    // ============================================================
    // Misc
    // ============================================================
    size_t iteration_count_ = 0;
};

#endif // LEG_ODOMETRY_NODE_HPP
