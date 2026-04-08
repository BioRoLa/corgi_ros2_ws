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
    
    // Kinematics computations
    Eigen::MatrixXd calculate_P_poly(int rim, double alpha);
    Eigen::MatrixXd calculate_P_poly_3d(double alpha);
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
    Eigen::MatrixXd estimate(double theta, double beta, double torque_r, double torque_l);

private:
    KinematicsHelper kinematics_;
};


class ForceEstimationNode : public rclcpp::Node {
public:
    ForceEstimationNode();
    void run();

private:
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg);
    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg);
    void timer_cb();
    static void quaternion_to_euler(const Eigen::Quaterniond& q, 
                                    double& roll, 
                                    double& pitch, 
                                    double& yaw);

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
    rclcpp::TimerBase::SharedPtr timer_;
    
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