#ifndef FORCE_ESTIMATION_HPP
#define FORCE_ESTIMATION_HPP

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "corgi_force_estimation/frame_transform.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/fitted_coefficient.hpp"

#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/force_state_stamped.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/contact_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"


class KinematicsHelper {
public:
    explicit KinematicsHelper(bool sim);

    // How a foot (rim 1) contact point moves with the joints.
    //   Axis: the foot arc stays aligned with the O-G leg axis without loading
    //         the motors (the LegModel geometry; the original Jacobian).
    //   F_l / F_r: the foot is held on that lower link, so moments about G go
    //         through that link's chain. corgi_sim servoes its foot (G) joint
    //         on one lower link, which behaves as F_l.
    enum class FootLink { Axis, F_l, F_r };
    void set_foot_link(FootLink link) { foot_link_ = link; }
    FootLink foot_link() const { return foot_link_; }
    // "axis", "F_l" or "F_r"; returns false for anything else.
    static bool parse_foot_link(const std::string& name, FootLink& link);

    // Contact Jacobian in model coordinates (rows x, y, z of the leg frame;
    // columns phi_L, phi_R, gamma). Call leg_model.contact_map_3d first.
    Eigen::Matrix3d contact_jacobian_3d(double theta, double beta, double gamma);

    // Kinematics computations
    Eigen::MatrixXd calculate_P_poly(int rim, double alpha);
    Eigen::MatrixXd calculate_P_poly_3d(double alpha);
    Eigen::MatrixXd calculate_P_poly_3d(int rim, double alpha);
    // contact_radius: radius of the contact point about the rim arc centre
    // (LegModel::contact_radius). The tyre is a torus, so it drops below
    // foot_radius once the wheel is tilted.
    Eigen::MatrixXd calculate_P_poly_3d(int rim, double alpha, double contact_radius);
    Eigen::MatrixXd calculate_jacobian(const Eigen::MatrixXd& P_theta,
                                       const Eigen::MatrixXd& P_theta_deriv,
                                       double beta);
    Eigen::MatrixXd calculate_jacobian_3d(const Eigen::MatrixXd& P_theta,
                                          const Eigen::MatrixXd& P_theta_deriv,
                                          double beta,
                                          double gamma,
                                          double d_wheel = 0.0);

    // Accessors
    LegModel& get_leg_model() { return leg_model_; }
    const LegModel& get_leg_model() const { return leg_model_; }
    bool is_sim() const { return sim_; }

private:
    void initialize_coefficients();

    bool sim_;
    LegModel leg_model_;
    LegModel foot_model_;  // scratch model for the foot-link Jacobian
    FootLink foot_link_ = FootLink::Axis;

    // Polynomial coefficient matrices (2x8 each)
    Eigen::MatrixXd H_l_coef_;
    Eigen::MatrixXd H_r_coef_;
    Eigen::MatrixXd F_l_coef_;
    Eigen::MatrixXd F_r_coef_;
    Eigen::MatrixXd U_l_coef_;
    Eigen::MatrixXd U_r_coef_;
    Eigen::MatrixXd L_l_coef_;
    Eigen::MatrixXd L_r_coef_;
    Eigen::MatrixXd G_coef_;
    Eigen::MatrixXd O_r_coef_;   // Foot point coefficients
    Eigen::MatrixXd J_l_coef_;   // Left upper point coefficients
    Eigen::MatrixXd J_r_coef_;   // Right upper point coefficients
};


class ForceEstimator {
public:
    explicit ForceEstimator(bool sim);
    Eigen::MatrixXd estimate(double theta, double beta, double gamma, double torque_l, double torque_r, double torque_h);
    // Return world-frame ground reaction; input angles/torques are motor coordinates.
    Eigen::MatrixXd estimate(double theta, double beta, double gamma, double torque_l, double torque_r, double torque_h, const Eigen::Matrix3d& world_from_body, const corgi_force::LegFrame& frame);

    KinematicsHelper& kinematics() { return kinematics_; }
    // Leg weight removed from the motor torques before solving for the force.
    void set_leg_gravity(const corgi_force::LegGravity& gravity) { gravity_ = gravity; }

private:
    KinematicsHelper kinematics_;
    corgi_force::LegGravity gravity_;
};

// Leg model options shared by force estimation and force control, as ROS
// parameters (defaults differ between simulation and hardware):
//   foot_link       "F_l" (sim) / "axis" (real), see KinematicsHelper::FootLink
//   leg_gravity     true (sim) / false (real): compensate the leg's own weight
//   leg_m_cz, leg_m_cz_theta, abad_m_d   LegGravity mass moments; the
//                   defaults are fitted to the corgi_sim robot on force plates
// Returns false (and logs) for an invalid foot_link.
bool declare_leg_model_parameters(rclcpp::Node& node, bool sim, KinematicsHelper& kinematics,
                                  corgi_force::LegGravity& gravity);


class ForceEstimationNode : public rclcpp::Node {
public:
    ForceEstimationNode();
    void run();

private:
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
    void timer_cb();


    // Force estimator
    std::unique_ptr<ForceEstimator> estimator_;

    // Message
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ForceStateStamped force_state_;
    corgi_msgs::msg::ImuStamped imu_;

    // ROS2
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Publisher<corgi_msgs::msg::ForceStateStamped>::SharedPtr force_state_pub_;
    std::array<corgi_force::LegFrame, 4> leg_frames_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr directions_sub_;
    bool directions_received_ = false;
    bool motor_state_received_ = false;
    bool imu_received_ = false;

    // Physical parameters
    bool sim_;
    double mass_;
    const double gravity_ = -9.81;

    // Dynamic friction compensation coefficients (AR, AL, BR, BL, CR, CL, DR, DL)
    const std::vector<double> friction_ = {0.625, 0.44, 0.662, 0.499, 0.623, 0.409, 0.677, 0.356};


    // Previous phi values for each module [phi_r, phi_l]
    std::vector<Eigen::MatrixXd> phi_prev_modules_;
};

#endif
