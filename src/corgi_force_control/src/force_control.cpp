#include "corgi_force_control/force_control.hpp"

ForceControlNode::ForceControlNode()
    : Node("force_control"),
      kinematics_(sim_),
      phi_vel_prev_modules_(4, Eigen::MatrixXd::Zero(2, 1)),
      phi_prev_modules_(4, Eigen::MatrixXd::Zero(2, 1))
{
    RCLCPP_INFO(this->get_logger(), "Force Control Starts");
    
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
        "impedance/command", 1000, 
        std::bind(&ForceControlNode::imp_cmd_cb, this, std::placeholders::_1));
    
    force_state_sub_ = this->create_subscription<corgi_msgs::msg::ForceStateStamped>(
        "force/state", 1000, 
        std::bind(&ForceControlNode::force_state_cb, this, std::placeholders::_1));
    
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 1000, 
        std::bind(&ForceControlNode::motor_state_cb, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 1000, 
        std::bind(&ForceControlNode::imu_cb, this, std::placeholders::_1));
    
    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
        "motor/command", 1000);
}

void ForceControlNode::imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg) {
    imp_cmd_ = *msg;
}

void ForceControlNode::force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
    force_state_ = *msg;
}

void ForceControlNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

void ForceControlNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void ForceControlNode::quaternion_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void ForceControlNode::force_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                     Eigen::MatrixXd phi_vel_prev_, 
                                     corgi_msgs::msg::MotorState* motor_state_, 
                                     corgi_msgs::msg::ForceState* force_state_, 
                                     corgi_msgs::msg::MotorCmd* motor_cmd_, 
                                     double pitch) {
    // force command
    Eigen::MatrixXd force_des(2, 1);
    force_des << imp_cmd_->fx, imp_cmd_->fy;

    Eigen::MatrixXd force_err(2, 1);
    force_err << force_des(0, 0)-force_state_->fx, force_des(1, 0)-force_state_->fy;

    // position command and state
    Eigen::MatrixXd pos_des(2, 1);

    LegModel& legmodel = kinematics_.get_leg_model();
    legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);

    int target_rim = legmodel.rim;
    int target_alpha = legmodel.alpha;

    pos_des << legmodel.contact_p[0], legmodel.contact_p[1];

    Eigen::MatrixXd pos_fb(2, 1);

    legmodel.forward(motor_state_->theta, motor_state_->beta + pitch);
    if      (target_rim == 1 && (target_alpha > -M_PI*1/2.0)) { pos_fb << legmodel.U_l[0], legmodel.U_l[1] - legmodel.radius; }
    else if (target_rim == 2) { pos_fb << legmodel.L_l[0], legmodel.L_l[1] - legmodel.radius; }
    else if (target_rim == 3) { pos_fb << legmodel.G[0]  , legmodel.G[1]   - legmodel.r;      }
    else if (target_rim == 4) { pos_fb << legmodel.L_r[0], legmodel.L_r[1] - legmodel.radius; }
    else if (target_rim == 5 && (target_alpha < M_PI*1/2.0)) { pos_fb << legmodel.U_r[0], legmodel.U_r[1] - legmodel.radius; }
    else {
        motor_cmd_->theta = imp_cmd_->theta;
        motor_cmd_->beta = imp_cmd_->beta;
        motor_cmd_->kp_r = 50;
        motor_cmd_->kp_l = 50;
        motor_cmd_->kd_r = 1;
        motor_cmd_->kd_l = 1;
        motor_cmd_->torque_r = 0;
        motor_cmd_->torque_l = 0;
        return;
    }

    Eigen::MatrixXd pos_err(2, 1);
    pos_err = pos_des - pos_fb;
    
    // std::cout << "force_des: " << force_des(0, 0) << ", " << force_des(1, 0) << std::endl;
    // std::cout << "force_fb : " << force_state_->Fx << ", " << force_state_->Fy << std::endl;
    // std::cout << "force_err: " << force_err(0, 0) << ", " << force_err(1, 0) << std::endl << std::endl;
    // std::cout << "pos_des: " << pos_des(0, 0) << ", " << pos_des(1, 0) << std::endl;
    // std::cout << "pos_fb : " << pos_fb(0, 0)  << ", " << pos_fb(1, 0)  << std::endl;
    // std::cout << "pos_err: " << pos_err(0, 0) << ", " << pos_err(1, 0) << std::endl << std::endl;

    // calculate jacobian
    Eigen::MatrixXd P_poly = Eigen::MatrixXd::Zero(2, 8);
    Eigen::MatrixXd P_poly_deriv = Eigen::MatrixXd::Zero(2, 7);
    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    legmodel.contact_map(motor_state_->theta, motor_state_->beta + pitch);
    P_poly = kinematics_.calculate_P_poly(legmodel.rim, legmodel.alpha);
    
    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(motor_state_->theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(motor_state_->theta, i); 
    
    Eigen::MatrixXd J_fb(2, 2);
    J_fb = kinematics_.calculate_jacobian(P_theta, P_theta_deriv, motor_state_->beta + pitch);

    // std::cout << "J_fb: " << std::endl
    //           << J_fb(0, 0) << ", " << J_fb(0, 1) << std::endl
    //           << J_fb(1, 0) << ", " << J_fb(1, 1) << std::endl << std::endl;

    Eigen::MatrixXd phi_vel(2, 1);
    phi_vel << motor_state_->velocity_r, motor_state_->velocity_l;

    Eigen::MatrixXd vel_fb = J_fb.transpose() * phi_vel;
    Eigen::MatrixXd acc_fb = (vel_fb - J_fb.transpose() * phi_vel_prev_) * 1000;

    // std::cout << "vel_fb : " << vel_fb(0, 0)  << ", " << vel_fb(1, 0)  << std::endl;
    // std::cout << "acc_fb : " << acc_fb(0, 0)  << ", " << acc_fb(1, 0)  << std::endl << std::endl;

    // impedance control
    Eigen::MatrixXd M(2, 2);
    Eigen::MatrixXd B(2, 2);
    Eigen::MatrixXd K(2, 2);
    Eigen::MatrixXd eta_cmd(2, 1);
    Eigen::MatrixXd trq_cmd(2, 1);
    Eigen::MatrixXd kp_cmd(2, 1);
    Eigen::MatrixXd kd_cmd(2, 1);

    M << imp_cmd_->mx, 0, 0, imp_cmd_->my;
    B << imp_cmd_->bx, 0, 0, imp_cmd_->by;
    K << imp_cmd_->kx, 0, 0, imp_cmd_->ky;

    eta_cmd << imp_cmd_->theta, imp_cmd_->beta;
    trq_cmd = J_fb.transpose() * (force_des + M*(-acc_fb));
    // std::cout << "trq_cmd : " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl << std::endl;

    // calculate phi command
    Eigen::MatrixXd phi_des(2, 1);
    phi_des << eta_cmd(1, 0) - eta_cmd(0, 0) + 17/180.0*M_PI,
               eta_cmd(1, 0) + eta_cmd(0, 0) - 17/180.0*M_PI;

    Eigen::MatrixXd phi_fb(2, 1);
    phi_fb << motor_state_->beta + pitch - motor_state_->theta + 17/180.0*M_PI,
              motor_state_->beta + pitch + motor_state_->theta - 17/180.0*M_PI;

    Eigen::MatrixXd phi_err = phi_des-phi_fb;

    // kp compensate
    kp_cmd << J_fb(0, 0) * J_fb(0, 0) * K(0, 0) + J_fb(1, 0) * J_fb(1, 0) * K(1, 1),
              J_fb(0, 1) * J_fb(0, 1) * K(0, 0) + J_fb(1, 1) * J_fb(1, 1) * K(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(1, 0);
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(0, 0);

    // kd compensate
    kd_cmd << J_fb(0, 0) * J_fb(0, 0) * B(0, 0) + J_fb(1, 0) * J_fb(1, 0) * B(1, 1),
              J_fb(0, 1) * J_fb(0, 1) * B(0, 0) + J_fb(1, 1) * J_fb(1, 1) * B(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(0, 1) * B(0, 0) + J_fb(1, 0) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(1, 0));
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * B(0, 0) + J_fb(1, 0) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(0, 0));
    
    // torque version impedance control
    // trq_cmd << J_fb.transpose() * (force_des + M*(-acc_fb) + B*(-vel_fb) + K*pos_err);
    // kp_cmd << 0, 0;
    // kd_cmd << 0, 0;

    // std::cout << (force_des)(0, 0) << ", " << force_des(1, 0) << std::endl;
    // std::cout << (K*pos_err)(0, 0) << ", " << (K*pos_err)(1, 0) << std::endl;
    // std::cout << (B*(-vel_fb))(0, 0) << ", " << (B*(-vel_fb))(1, 0) << std::endl;
    // std::cout << (M*(-acc_fb))(0, 0) << ", " << (M*(-acc_fb))(1, 0) << std::endl;


    // std::cout << "kp_cmd  : " << kp_cmd(0, 0) << ", " << kp_cmd(1, 0) << std::endl;
    // std::cout << "phi_err : " << phi_err(0, 0) << ", " << phi_err(1, 0) << std::endl << std::endl;

    // std::cout << "kd_cmd  : " << kd_cmd(0, 0) << ", " << kd_cmd(1, 0) << std::endl;
    // std::cout << "phi_vel : " << phi_vel(0, 0) << ", " << phi_vel(1, 0) << std::endl << std::endl;

    // std::cout << "eta_cmd: " << eta_cmd(0, 0) << ", " << eta_cmd(1, 0) << std::endl;
    // std::cout << "trq_kp  : " << kp_cmd(0, 0)*phi_err(0, 0) << ", " << kp_cmd(1, 0)*phi_err(1, 0) << std::endl;
    // std::cout << "trq_kd  : " << -kd_cmd(0, 0)*phi_vel(0, 0) << ", " << -kd_cmd(1, 0)*phi_vel(1, 0) << std::endl;
    // std::cout << "trq_cmd: " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl;

    // send to motor command
    motor_cmd_->theta = eta_cmd(0, 0);
    motor_cmd_->beta = eta_cmd(1, 0);
    motor_cmd_->kp_r = kp_cmd(0, 0); //std::min(200.0, std::max(10.0, kp_cmd(0, 0)));
    motor_cmd_->kp_l = kp_cmd(1, 0); //std::min(200.0, std::max(10.0, kp_cmd(1, 0)));
    motor_cmd_->kd_r = kd_cmd(0, 0); //std::min(5.0,   std::max(0.05, kd_cmd(0, 0)));
    motor_cmd_->kd_l = kd_cmd(1, 0); //std::min(5.0,   std::max(0.05, kd_cmd(1, 0)));
    motor_cmd_->torque_r = trq_cmd(0, 0);
    motor_cmd_->torque_l = trq_cmd(1, 0);
}

void ForceControlNode::position_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                        corgi_msgs::msg::MotorCmd* motor_cmd_) {
    motor_cmd_->theta = imp_cmd_->theta;
    motor_cmd_->beta = imp_cmd_->beta;
    motor_cmd_->kp_r = 50;
    motor_cmd_->kp_l = 50;
    motor_cmd_->kd_r = 1;
    motor_cmd_->kd_l = 1;
    motor_cmd_->torque_r = 0;
    motor_cmd_->torque_l = 0;
}

void ForceControlNode::timer_cb() {
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
        if (imp_cmd_modules[i]->kx == 0 && imp_cmd_modules[i]->ky == 0 && imp_cmd_modules[i]->bx == 0 && imp_cmd_modules[i]->by == 0) {
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
        phi_vel_prev_modules_[i] << motor_state_modules[i]->velocity_r, motor_state_modules[i]->velocity_l;
    }

    // dynamic friction compensation
    if (!kinematics_.is_sim()){
        for (int i=0; i<4; i++) {
            double phi_r = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
            double phi_l = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;

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

            phi_prev_modules_[i] << phi_r, phi_l;
        }
    }

    // std::cout << "= = = = = = = = = = =" << std::endl << std::endl;

    motor_cmd_.header.stamp = this->now();
    
    motor_cmd_pub_->publish(motor_cmd_);

    loop_count_++;
}

void ForceControlNode::run() {
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
    auto node = std::make_shared<ForceControlNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}