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
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "kinematic/ContactMap.hpp"
#include "es_ekf/ESEKF.hpp"
#include "Config.hpp"

/**
 * @brief Leg Odometry Node
 *
 * Responsibilities:
 *   1. Subscribe to IMU, motor state, position, velocity, trigger
 *   2. Run disturbance observer → publish contact state
 *   3. Run ES-EKF for odometry estimation
 *
 * TODO: Once ES-EKF output is validated, remove position_sub_ and
 *       velocity_sub_ (currently only used by disturbance observer).
 *       Publish nav_msgs/Odometry so this node becomes a fully
 *       closed-loop estimator that does not depend on external
 *       position/velocity topics.
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
    // ES-EKF helpers
    // ============================================================

    /// @brief Extract per-leg encoder state from motor_state_ message
    ///        and build LegObservation for ES-EKF update.
    ///        Handles sign conventions per leg side.
    /// @param module   motor state of one leg module
    /// @param leg      pointer to corresponding Leg kinematic model
    /// @param leg_idx  index in the 4-leg array (0=LF,1=RF,2=RH,3=LH)
    /// @param w_y      IMU pitch angular velocity [rad/s]
    /// @return fully populated LegObservation
    estimation_model::LegObservation build_leg_observation(
        const corgi_msgs::msg::MotorState& module,
        Leg* leg, int leg_idx, float w_y);

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

    /// Leg pointers in indexed order [LF, RF, RH, LH]
    std::array<Leg*, 4> legs_ = {&lf_leg_, &rf_leg_, &rh_leg_, &lh_leg_};

    // ============================================================
    // ROS subscribers & publishers
    // ============================================================
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr          imu_sub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr      trigger_sub_;

    // TODO: Remove position/velocity subs once ESEKF output is validated
    //       and disturbance observer is decoupled or fed from ESEKF state.
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          velocity_sub_;

    rclcpp::Publisher<corgi_msgs::msg::ContactStateStamped>::SharedPtr    contact_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                 ekf_pub_;

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
    // Processing components — disturbance observer
    // ============================================================
    DataProcessor              processor_;
    corgi::DisturbanceObserver observer_;

    // ============================================================
    // ES-EKF
    // ============================================================
    estimation_model::ESEKF esekf_;

    /// ContactMap (shared, stateless lookup — one instance is enough)
    estimation_model::ContactMap contact_map_;

    /// Per-leg accumulated contact_beta [rad]
    /// contact_beta += (beta_d + omega_y) * dt
    std::array<float, 4> contact_beta_{{0.f, 0.f, 0.f, 0.f}};

    /// Whether the ESEKF has been initialized (on first triggered tick)
    bool esekf_initialized_ = false;

    // ============================================================
    // Debug
    // ============================================================

    /// @brief Publish per-leg observation inputs and z_leg debug topics.
    /// @param observations  4-leg observation vector (const ref, but FK is re-run internally)
    /// @param w_m           raw IMU angular velocity [rad/s]
    void publish_debug(
        std::vector<estimation_model::LegObservation>& observations,
        const Eigen::Vector3f& w_m);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_leg_obs_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_zleg_pub_;

    // ============================================================
    // Misc
    // ============================================================
    size_t iteration_count_ = 0;
};

#endif // LEG_ODOMETRY_NODE_HPP
