#ifndef LEG_ODOMETRY_NODE_HPP
#define LEG_ODOMETRY_NODE_HPP

#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <corgi_msgs/msg/imu_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/gmo_contact_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>

#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "kinematic/ContactMap.hpp"
#include "es_ekf/ESEKF.hpp"
#include "common/Config.hpp"
#include "common/Params.hpp"

/**
 * @brief Leg Odometry Node
 *
 * Responsibilities:
 *   1. Subscribe to IMU, motor state, position, velocity, trigger
 *   2. Run disturbance observer → publish contact state
 *   3. Run ES-EKF for odometry estimation
 *
 * When use_esekf_state=true (config_online.yaml), GMO inputs are overridden
 * with the ESEKF estimated state instead of external velocity_estimator
 * topics (sim/position, sim/velocity). position_sub_ and
 * velocity_sub_ remain active for use_esekf_state=false (legacy mode).
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
    /// @return fully populated LegObservation
    estimation_model::LegObservation build_leg_observation(
        const corgi_msgs::msg::MotorState& module,
        Leg* leg, int leg_idx);

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

    rclcpp::Publisher<corgi_msgs::msg::GMOContactStateStamped>::SharedPtr    contact_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr             ekf_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr             ekf_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr          ekf_orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr             ekf_ba_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr             ekf_bw_pub_;

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
    // IMU timestamp tracking (for dynamic ESEKF dt)
    // Stored as raw sec/nsec to match offline pipeline arithmetic exactly.
    // ============================================================
    int32_t  last_esekf_imu_sec_  = 0;
    uint32_t last_esekf_imu_nsec_ = 0;
    bool     last_esekf_imu_time_valid_ = false;

    // ESEKF decimation counter (runs at ESEKF_RATE = 500Hz in a 1000Hz loop)
    size_t esekf_tick_ = 0;

    // ============================================================
    // IMU trapezoidal averaging for ESEKF predict
    // ============================================================
    /// Previous tick's IMU (intermediate sample between ESEKF triggers)
    Eigen::Vector3f prev_imu_a_{0.f, 0.f, 0.f};
    Eigen::Vector3f prev_imu_w_{0.f, 0.f, 0.f};
    bool prev_imu_valid_ = false;

    // ============================================================
    // Contact state (Schmitt trigger memory)
    // ============================================================
    std::array<bool, 4> leg_contact_state_{{false, false, false, false}};

    // Contact thresholds (loaded from YAML config)
    double contact_rm_threshold_high_   = 25.0;
    double contact_rm_threshold_low_    = 15.0;
    double contact_beta_threshold_high_ = 10.0;
    double contact_beta_threshold_low_  =  1.0;

    // ============================================================
    // YAML config (loaded once at startup via yaml-cpp)
    // ============================================================
    corgi::Params params_;

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
    // Misc
    // ============================================================
    bool   use_esekf_state_ = false;
    size_t iteration_count_ = 0;
};

#endif // LEG_ODOMETRY_NODE_HPP
