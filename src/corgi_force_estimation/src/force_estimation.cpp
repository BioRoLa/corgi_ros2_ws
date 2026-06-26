#include "corgi_force_estimation/force_estimation.hpp"

KinematicsHelper::KinematicsHelper(bool sim)
    : sim_(sim),
      leg_model_(sim),
      H_l_coef_(2, 8),
      H_r_coef_(2, 8),
      F_l_coef_(2, 8),
      F_r_coef_(2, 8),
      U_l_coef_(2, 8),
      U_r_coef_(2, 8),
      L_l_coef_(2, 8),
      L_r_coef_(2, 8),
      G_coef_(2, 8),
      O_r_coef_(2, 8),
      J_l_coef_(2, 8),
      J_r_coef_(2, 8)
{
    initialize_coefficients();
}

void KinematicsHelper::initialize_coefficients() {
    for (int i=0; i<8; i++){
        H_l_coef_(0, i) = H_x_coef[i];
        H_l_coef_(1, i) = H_y_coef[i];

        H_r_coef_(0, i) = -H_x_coef[i];
        H_r_coef_(1, i) = H_y_coef[i];

        U_l_coef_(0, i) = U_x_coef[i];
        U_l_coef_(1, i) = U_y_coef[i];

        U_r_coef_(0, i) = -U_x_coef[i];
        U_r_coef_(1, i) = U_y_coef[i];
        
        L_l_coef_(0, i) = L_x_coef[i];
        L_l_coef_(1, i) = L_y_coef[i];
        
        L_r_coef_(0, i) = -L_x_coef[i];
        L_r_coef_(1, i) = L_y_coef[i];
        
        F_l_coef_(0, i) = F_x_coef[i];
        F_l_coef_(1, i) = F_y_coef[i];
        
        F_r_coef_(0, i) = -F_x_coef[i];
        F_r_coef_(1, i) = F_y_coef[i];
        
        G_coef_(0, i) = 0;
        G_coef_(1, i) = G_y_coef[i];
        
        // Foot point O_r coefficients (O_r = G + offset in x by R, same y as G)
        O_r_coef_(0, i) = 0;  // O_r x-coefficient (pure vertical geometry from G)
        O_r_coef_(1, i) = O_y_coef[i];  // O_r y-coordinate follows G
        
        // Upper point J_l coefficients (140 degrees)
        J_l_coef_(0, i) = J_x_coef[i];
        J_l_coef_(1, i) = J_y_coef[i];
        
        // Upper point J_r coefficients (140 degrees, symmetric)
        J_r_coef_(0, i) = -J_x_coef[i];
        J_r_coef_(1, i) = J_y_coef[i];
    }
}

Eigen::MatrixXd KinematicsHelper::calculate_P_poly(int rim, double alpha) {
    Eigen::MatrixXd P_poly(2, 8);

    double scaled_radius = leg_model_.radius / leg_model_.R;

    if (rim == 1 && alpha > -M_PI*2.0/3.0) {
        Eigen::Rotation2D<double> rotation(alpha+M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_l_coef_-U_l_coef_) * scaled_radius + U_l_coef_;
    }
    else if (rim == 2) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef_-L_l_coef_) * scaled_radius + L_l_coef_;
    }
    else if (rim == 3) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef_-L_l_coef_) * leg_model_.r / leg_model_.R + G_coef_;
    }
    else if (rim == 4) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef_-L_r_coef_) * scaled_radius + L_r_coef_;
    }
    else if (rim == 5 && alpha < M_PI*2.0/3.0) {
        Eigen::Rotation2D<double> rotation(alpha-M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_r_coef_-U_r_coef_) * scaled_radius + U_r_coef_;
    }
    else {
        P_poly = Eigen::MatrixXd::Zero(2, 8);
    }
    
    return P_poly;
}

Eigen::MatrixXd KinematicsHelper::calculate_P_poly_3d(double alpha) {
    Eigen::MatrixXd P_poly(2, 8);
    
    // Convert alpha from degrees to radians and normalize to [-180, 180]
    double alpha_rad = alpha * M_PI / 180.0;
    double a_mod = std::fmod((alpha + 180.0), 360.0) - 180.0;
    
    double scaled_radius = leg_model_.foot_radius / leg_model_.R;
    
    // 3-rim structure based on alpha ranges
    if (a_mod >= -40.0 && a_mod <= 40.0) {
        // Rim 1: Foot rim - centered at O_r, direction toward G
        Eigen::Rotation2D<double> rotation(alpha_rad);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef_ - O_r_coef_) * scaled_radius + O_r_coef_;
    }
    else if (a_mod > 40.0 && a_mod <= 180.0) {
        // Rim 2: Upper RHS - centered at U_r, direction toward J_r
        double angle_offset = (a_mod - 40.0) * M_PI / 180.0;
        Eigen::Rotation2D<double> rotation(angle_offset);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (J_r_coef_ - U_r_coef_) * scaled_radius + U_r_coef_;
    }
    else {
        // Rim 3: Upper LHS - centered at U_l, direction toward J_l
        double angle_offset = (a_mod + 40.0) * M_PI / 180.0;
        Eigen::Rotation2D<double> rotation(angle_offset);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (J_l_coef_ - U_l_coef_) * scaled_radius + U_l_coef_;
    }
    
    return P_poly;
}

Eigen::MatrixXd KinematicsHelper::calculate_jacobian(const Eigen::MatrixXd& P_theta, 
                                                      const Eigen::MatrixXd& P_theta_deriv, 
                                                      double beta) {
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    double dtheta_dphiR = -0.5;
    double dtheta_dphiL =  0.5;
    double dbeta_dphiR  =  0.5;
    double dbeta_dphiL  =  0.5;

    double dPx_dtheta = P_theta_deriv(0, 0)*cos_beta - P_theta_deriv(1, 0)*sin_beta;
    double dPy_dtheta = P_theta_deriv(0, 0)*sin_beta + P_theta_deriv(1, 0)*cos_beta;
    double dPx_dbeta  = P_theta(0, 0)*(-sin_beta) - P_theta(1, 0)*cos_beta;
    double dPy_dbeta  = P_theta(0, 0)*cos_beta + P_theta(1, 0)*(-sin_beta);

    double J11 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J12 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J21 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;
    double J22 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;

    Eigen::MatrixXd jacobian(2, 2);
    jacobian << J11, J12, J21, J22;

    return jacobian;
}

Eigen::MatrixXd KinematicsHelper::calculate_jacobian_3d(const Eigen::MatrixXd& P_theta, 
                                                      const Eigen::MatrixXd& P_theta_deriv, 
                                                      double beta,
                                                      double gamma,
                                                      double d_wheel) {
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    double dtheta_dphiR = -0.5;
    double dtheta_dphiL =  0.5;
    double dbeta_dphiR  =  0.5;
    double dbeta_dphiL  =  0.5;

    double dPx_dtheta = P_theta_deriv(0, 0)*cos_beta - P_theta_deriv(1, 0)*sin_beta;
    double dPy_dtheta = P_theta_deriv(0, 0)*sin_beta + P_theta_deriv(1, 0)*cos_beta;
    double dPx_dbeta  = P_theta(0, 0)*(-sin_beta) - P_theta(1, 0)*cos_beta;
    double dPy_dbeta  = P_theta(0, 0)*cos_beta + P_theta(1, 0)*(-sin_beta);

    double J11 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J12 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J21 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;
    double J22 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;

    double sin_g = std::sin(gamma);
    double cos_g = std::cos(gamma);
    double z_2D = P_theta(1,0);

    Eigen::MatrixXd jacobian(3, 3);
    jacobian << J11, J12, 0,
               -J21 * sin_g, -J22 * sin_g, -d_wheel * sin_g - z_2D * cos_g,
                J21 * cos_g,  J22 * cos_g,  d_wheel * cos_g - z_2D * sin_g;

    return jacobian;
}

// ============================================================================
// ForceEstimator Implementation
// ============================================================================

ForceEstimator::ForceEstimator(bool sim)
    : kinematics_(sim)
{
}

Eigen::MatrixXd ForceEstimator::estimate(double theta, double beta, double gamma, double torque_l, double torque_r, double torque_h) {
    LegModel& leg_model = kinematics_.get_leg_model();
    leg_model.contact_map_3d(theta, beta, gamma);

    Eigen::MatrixXd P_poly = kinematics_.calculate_P_poly_3d(leg_model.alpha);
    Eigen::MatrixXd P_poly_deriv(2, 7);

    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(theta, i); 
    
    Eigen::MatrixXd jacobian(3, 3);
    jacobian = kinematics_.calculate_jacobian_3d(P_theta, P_theta_deriv, beta, gamma, leg_model.d_wheel);
    
    Eigen::MatrixXd force_est(3, 1);

    if (jacobian.isZero(1e-6)) {
        force_est.setZero();
    } else {
        Eigen::MatrixXd torque(3, 1);
        torque << torque_l, torque_r, torque_h;
        force_est = -jacobian.inverse().transpose() * torque;
    }

    return force_est;
}

// ============================================================================
// ForceEstimationNode Implementation
// ============================================================================

ForceEstimationNode::ForceEstimationNode()
    : Node("force_estimation"),
      sim_(false),
      mass_(0.0),
      phi_prev_modules_(4, Eigen::MatrixXd::Zero(2, 1))
{
    RCLCPP_INFO(this->get_logger(), "Force Estimation Starts");

    this->get_parameter_or("use_sim_time", sim_, false);
    mass_ = 0.9;
    estimator_ = std::make_unique<ForceEstimator>(sim_);

    if (sim_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (this->now().seconds() > 0.0) {
                RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Real hardware mode: using system wall clock.");
    }

    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10, 
        std::bind(&ForceEstimationNode::motor_state_cb, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 1, 
        std::bind(&ForceEstimationNode::imu_cb, this, std::placeholders::_1));
    
    force_state_pub_ = this->create_publisher<corgi_msgs::msg::ForceStateStamped>("force/state", 10);
    
    auto rate = std::chrono::milliseconds(1);
    timer_ = this->create_wall_timer(
        rate, 
        std::bind(&ForceEstimationNode::timer_cb, this));
}

void ForceEstimationNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

void ForceEstimationNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void ForceEstimationNode::quaternion_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void ForceEstimationNode::timer_cb() {
    Eigen::Quaterniond body_angle_quat(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    quaternion_to_euler(body_angle_quat, roll, pitch, yaw);

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state_.module_a,
        &motor_state_.module_b,
        &motor_state_.module_c,
        &motor_state_.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state_.module_a,
        &force_state_.module_b,
        &force_state_.module_c,
        &force_state_.module_d
    };

    if (!sim_){
        pitch *= -1;
        yaw *= -1;

        // dynamic friction compensation
        for (int i=0; i<4; i++) {
            double phi_l = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
            double phi_r = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;

            if (phi_r > phi_prev_modules_[i](1, 0)){
                motor_state_modules[i]->torque_r += friction_[2*i];
            }
            else {
                motor_state_modules[i]->torque_r -= friction_[2*i];
            }

            if (phi_l > phi_prev_modules_[i](0, 0)){
                motor_state_modules[i]->torque_l += friction_[2*i+1];
            }
            else {
                motor_state_modules[i]->torque_l -= friction_[2*i+1];
            }

            phi_prev_modules_[i] << phi_l, phi_r;
        }
    }

    for (int i=0; i<4; i++){
        Eigen::MatrixXd force_est;

        if (i == 1 || i == 2) {
            force_est = estimator_->estimate(motor_state_modules[i]->theta, 
                                            motor_state_modules[i]->beta - pitch,
                                            motor_state_modules[i]->gamma,
                                            motor_state_modules[i]->torque_l, 
                                            motor_state_modules[i]->torque_r,
                                            motor_state_modules[i]->torque_h);
        }
        else {
            force_est = estimator_->estimate(motor_state_modules[i]->theta, 
                                            motor_state_modules[i]->beta + pitch,
                                            motor_state_modules[i]->gamma,
                                            motor_state_modules[i]->torque_l, 
                                            motor_state_modules[i]->torque_r,
                                            motor_state_modules[i]->torque_h);
        }

        if (i == 1 || i == 2) { 
            force_state_modules[i]->fx = -force_est(0, 0); 
            force_state_modules[i]->fy = -force_est(1, 0);
        }
        else { 
            force_state_modules[i]->fx = force_est(0, 0); 
            force_state_modules[i]->fy = -force_est(1, 0);
        }
        
        force_state_modules[i]->fz = -force_est(2, 0) - mass_ * gravity_;
    }

    force_state_.header.stamp = this->now();
    force_state_pub_->publish(force_state_);
}

void ForceEstimationNode::run() {
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        timer_cb();
        
        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceEstimationNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
