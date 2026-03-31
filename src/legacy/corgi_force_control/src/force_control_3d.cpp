#include "corgi_force_control/force_control_3d.hpp"

ForceControl3DNode::ForceControl3DNode()
    : Node("force_control_3d"),
      kinematics_(sim_),
      phi_vel_prev_modules_(4, Eigen::MatrixXd::Zero(3, 1)),
      phi_prev_modules_(4, Eigen::MatrixXd::Zero(3, 1))
{
    RCLCPP_INFO(this->get_logger(), "Force Control 3D Starts");
    
    // Wait for clock synchronization
    RCLCPP_INFO(this->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (this->now().seconds() > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    imp_cmd_sub_ = this->create_subscription<corgi_msgs::msg::ImpedanceCmdStamped>(
        "impedance/command", 10, 
        std::bind(&ForceControl3DNode::imp_cmd_cb, this, std::placeholders::_1));
    
    force_state_sub_ = this->create_subscription<corgi_msgs::msg::ForceStateStamped>(
        "force/state", 10, 
        std::bind(&ForceControl3DNode::force_state_cb, this, std::placeholders::_1));
    
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10, 
        std::bind(&ForceControl3DNode::motor_state_cb, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 10, 
        std::bind(&ForceControl3DNode::imu_cb, this, std::placeholders::_1));
    
    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
        "motor/command", 10);
}

void ForceControl3DNode::imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg) {
    imp_cmd_ = *msg;
}

void ForceControl3DNode::force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
    force_state_ = *msg;
}

void ForceControl3DNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

void ForceControl3DNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void ForceControl3DNode::quaternion_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void ForceControl3DNode::force_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                       Eigen::MatrixXd phi_vel_prev_, 
                                       corgi_msgs::msg::MotorState* motor_state_, 
                                       corgi_msgs::msg::ForceState* force_state_, 
                                       corgi_msgs::msg::MotorCmd* motor_cmd_, 
                                       double pitch) {
    // force command (3D)
    Eigen::MatrixXd force_des(3, 1);
    force_des << imp_cmd_->fx, imp_cmd_->fy, imp_cmd_->fz;

    Eigen::MatrixXd force_err(3, 1);
    force_err << force_des(0, 0)-force_state_->fx, 
                 force_des(1, 0)-force_state_->fy, 
                 force_des(2, 0)-force_state_->fz;

    // position command and state (3D)
    Eigen::MatrixXd pos_des(3, 1);

    LegModel& legmodel = kinematics_.get_leg_model();
    
    // Note: Use contact_map_3d and 3D position logic
    legmodel.contact_map_3d(imp_cmd_->theta, imp_cmd_->beta, imp_cmd_->gamma);

    int target_rim = legmodel.rim;
    int target_alpha = legmodel.alpha;

    pos_des << legmodel.contact_p_3d[0], legmodel.contact_p_3d[1], legmodel.contact_p_3d[2];

    Eigen::MatrixXd pos_fb(3, 1);

    legmodel.contact_map_3d(motor_state_->theta, motor_state_->beta + pitch, motor_state_->gamma);
    pos_fb << legmodel.contact_p_3d[0], legmodel.contact_p_3d[1], legmodel.contact_p_3d[2];

    if (target_rim == 0 || target_rim > 5) {
        motor_cmd_->theta = imp_cmd_->theta;
        motor_cmd_->beta = imp_cmd_->beta;
        motor_cmd_->gamma = imp_cmd_->gamma;
        motor_cmd_->kp_r = 50;
        motor_cmd_->kp_l = 50;
        motor_cmd_->kp_h = 50;
        motor_cmd_->kd_r = 1;
        motor_cmd_->kd_l = 1;
        motor_cmd_->kd_h = 1;
        motor_cmd_->torque_r = 0;
        motor_cmd_->torque_l = 0;
        motor_cmd_->torque_h = 0;
        return;
    }

    Eigen::MatrixXd pos_err(3, 1);
    pos_err = pos_des - pos_fb;
    
    // calculate jacobian
    Eigen::MatrixXd P_poly = Eigen::MatrixXd::Zero(2, 8);
    Eigen::MatrixXd P_poly_deriv = Eigen::MatrixXd::Zero(2, 7);
    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    P_poly = kinematics_.calculate_P_poly(legmodel.rim, legmodel.alpha);
    
    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(motor_state_->theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(motor_state_->theta, i); 
    
    Eigen::MatrixXd J_fb(3, 3);
    // Note: Assumes kinematics_.calculate_jacobian_3d exists and takes gamma & d_wheel
    J_fb = kinematics_.calculate_jacobian_3d(P_theta, P_theta_deriv, motor_state_->beta + pitch, motor_state_->gamma, legmodel.d_wheel);

    Eigen::MatrixXd phi_vel(3, 1);
    phi_vel << motor_state_->velocity_r, motor_state_->velocity_l, motor_state_->velocity_h;

    Eigen::MatrixXd vel_fb = J_fb.transpose() * phi_vel;
    Eigen::MatrixXd acc_fb = (vel_fb - J_fb.transpose() * phi_vel_prev_) * 1000;

    // impedance control
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd eta_cmd(3, 1);
    Eigen::MatrixXd trq_cmd_base(3, 1);
    Eigen::MatrixXd trq_cmd(3, 1);

    M(0, 0) = imp_cmd_->mx; M(1, 1) = imp_cmd_->my; M(2, 2) = imp_cmd_->mz;
    B(0, 0) = imp_cmd_->bx; B(1, 1) = imp_cmd_->by; B(2, 2) = imp_cmd_->bz;
    K(0, 0) = imp_cmd_->kx; K(1, 1) = imp_cmd_->ky; K(2, 2) = imp_cmd_->kz;

    eta_cmd << imp_cmd_->theta, imp_cmd_->beta, imp_cmd_->gamma;
    trq_cmd_base = J_fb.transpose() * (force_des + M*(-acc_fb));

    // calculate phi command
    Eigen::MatrixXd phi_des(3, 1);
    phi_des << eta_cmd(1, 0) - eta_cmd(0, 0) + 17/180.0*M_PI,
               eta_cmd(1, 0) + eta_cmd(0, 0) - 17/180.0*M_PI,
               eta_cmd(2, 0);

    Eigen::MatrixXd phi_fb(3, 1);
    phi_fb << motor_state_->beta + pitch - motor_state_->theta + 17/180.0*M_PI,
              motor_state_->beta + pitch + motor_state_->theta - 17/180.0*M_PI,
              motor_state_->gamma;

    Eigen::MatrixXd phi_err = phi_des-phi_fb;

    // Equivalent joint PD gains
    Eigen::MatrixXd K_joint = J_fb.transpose() * K * J_fb;
    Eigen::MatrixXd B_joint = J_fb.transpose() * B * J_fb;

    // Extract diagonal terms for actual PD controller
    double kp_r_cmd = K_joint(0, 0);
    double kp_l_cmd = K_joint(1, 1);
    double kp_h_cmd = K_joint(2, 2);

    double kd_r_cmd = B_joint(0, 0);
    double kd_l_cmd = B_joint(1, 1);
    double kd_h_cmd = B_joint(2, 2);

    // Initial torque command
    trq_cmd = trq_cmd_base;

    // Coupling compensation
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            if (i != j) {
                trq_cmd(i) += K_joint(i, j) * phi_err(j);
                trq_cmd(i) += B_joint(i, j) * (-phi_vel(j));
            }
        }
    }

    // send to motor command
    motor_cmd_->theta = eta_cmd(0, 0);
    motor_cmd_->beta = eta_cmd(1, 0);
    motor_cmd_->gamma = eta_cmd(2, 0);
    motor_cmd_->kp_r = kp_r_cmd;
    motor_cmd_->kp_l = kp_l_cmd;
    motor_cmd_->kp_h = kp_h_cmd;
    motor_cmd_->kd_r = kd_r_cmd;
    motor_cmd_->kd_l = kd_l_cmd;
    motor_cmd_->kd_h = kd_h_cmd;
    motor_cmd_->torque_r = trq_cmd(0, 0);
    motor_cmd_->torque_l = trq_cmd(1, 0);
    motor_cmd_->torque_h = trq_cmd(2, 0);
}

void ForceControl3DNode::position_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                        corgi_msgs::msg::MotorCmd* motor_cmd_) {
    motor_cmd_->theta = imp_cmd_->theta;
    motor_cmd_->beta = imp_cmd_->beta;
    motor_cmd_->gamma = imp_cmd_->gamma;
    motor_cmd_->kp_r = 50;
    motor_cmd_->kp_l = 50;
    motor_cmd_->kp_h = 50;
    motor_cmd_->kd_r = 1;
    motor_cmd_->kd_l = 1;
    motor_cmd_->kd_h = 1;
    motor_cmd_->torque_r = 0;
    motor_cmd_->torque_l = 0;
    motor_cmd_->torque_h = 0;
}

void ForceControl3DNode::timer_cb() {
    Eigen::Quaterniond body_angle_quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    body_angle_quat = {imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z};
    quaternion_to_euler(body_angle_quat, roll, pitch, yaw);

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state_.module_a,
        &force_state_.module_b,
        &force_state_.module_c,
        &force_state_.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state_.module_a,
        &motor_state_.module_b,
        &motor_state_.module_c,
        &motor_state_.module_d
    };

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd_.module_a,
        &motor_cmd_.module_b,
        &motor_cmd_.module_c,
        &motor_cmd_.module_d
    };

    for (int i=0; i<4; i++){
        if (imp_cmd_modules[i]->kx == 0 && imp_cmd_modules[i]->ky == 0 && imp_cmd_modules[i]->kz == 0 && 
            imp_cmd_modules[i]->bx == 0 && imp_cmd_modules[i]->by == 0 && imp_cmd_modules[i]->bz == 0) {
            position_control(imp_cmd_modules[i], motor_cmd_modules[i]);
        }
        else {
            if (imp_cmd_modules[i]->theta < 17/180.0*M_PI) { imp_cmd_modules[i]->theta = 17/180.0*M_PI; }

            if (i == 1 || i == 2) {
                force_control(imp_cmd_modules[i], phi_vel_prev_modules_[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], -pitch);
            }
            else {
                force_control(imp_cmd_modules[i], phi_vel_prev_modules_[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], pitch);
            }
            
        }
        phi_vel_prev_modules_[i] << motor_state_modules[i]->velocity_r, motor_state_modules[i]->velocity_l, motor_state_modules[i]->velocity_h;
    }

    // dynamic friction compensation
    if (!kinematics_.is_sim()){
        for (int i=0; i<4; i++) {
            double phi_r = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
            double phi_l = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;
            double gamma_fb = motor_state_modules[i]->gamma;

            if (phi_r > phi_prev_modules_[i](0, 0)){
                motor_cmd_modules[i]->torque_r -= friction_[2*i];
            }
            else {
                motor_cmd_modules[i]->torque_r += friction_[2*i];
            }

            if (phi_l > phi_prev_modules_[i](1, 0)){
                motor_cmd_modules[i]->torque_l -= friction_[2*i+1];
            }
            else {
                motor_cmd_modules[i]->torque_l += friction_[2*i+1];
            }
            
            // NOTE: assuming zero constant friction for gamma for now since it wasn't specified.
            // if needed, similar compensation can be added.

            phi_prev_modules_[i] << phi_r, phi_l, gamma_fb;
        }
    }

    motor_cmd_.header.stamp = this->now();
    
    motor_cmd_pub_->publish(motor_cmd_);

    loop_count_++;
}

void ForceControl3DNode::run() {
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


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceControl3DNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}