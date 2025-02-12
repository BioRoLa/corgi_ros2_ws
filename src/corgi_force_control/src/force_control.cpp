#include "force_estimation.hpp"

corgi_msgs::ImpedanceCmdStamped imp_cmd;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::MotorCmdStamped motor_cmd;


void imp_cmd_cb(const corgi_msgs::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_control(corgi_msgs::ImpedanceCmd* imp_cmd_, Eigen::MatrixXd eta_hist_, corgi_msgs::ForceState* force_state_, corgi_msgs::MotorCmd* motor_cmd_){
    // force command
    Eigen::MatrixXd force_des(2, 1);
    force_des << imp_cmd_->Fx, imp_cmd_->Fy;

    Eigen::MatrixXd force_err(2, 1);
    force_err << force_des(0, 0)-force_state_->Fx, force_des(1, 0)-force_state_->Fy;

    // position command and state
    Eigen::MatrixXd pos_des(2, 1);

    legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);

    int target_rim = legmodel.rim;
    int target_alpha = legmodel.alpha;

    pos_des << legmodel.contact_p[0], legmodel.contact_p[1];

    Eigen::MatrixXd pos_hist_ = Eigen::MatrixXd::Zero(2, 3);
    
    for (int i=0; i<3; i++){
        legmodel.forward(eta_hist_.col(i)(0, 0), eta_hist_.col(i)(1, 0));
        if      (target_rim == 1) { pos_hist_.col(i) << legmodel.U_l[0], legmodel.U_l[1] - legmodel.radius; }
        else if (target_rim == 2) { pos_hist_.col(i) << legmodel.L_l[0], legmodel.L_l[1] - legmodel.radius; }
        else if (target_rim == 3) { pos_hist_.col(i) << legmodel.G[0]  , legmodel.G[1]   - legmodel.r;      }
        else if (target_rim == 4) { pos_hist_.col(i) << legmodel.L_r[0], legmodel.L_r[1] - legmodel.radius; }
        else if (target_rim == 5) { pos_hist_.col(i) << legmodel.U_r[0], legmodel.U_r[1] - legmodel.radius; }
        else { pos_hist_.col(i) << 0, 0; }
    }

    Eigen::MatrixXd pos_fb = pos_hist_.col(0);
    Eigen::MatrixXd vel_fb = (pos_hist_.col(0)-pos_hist_.col(1))*1000;
    Eigen::MatrixXd acc_fb = (pos_hist_.col(0)-2*pos_hist_.col(1)+pos_hist_.col(2))*1000000;

    Eigen::MatrixXd pos_err(2, 1);
    pos_err = pos_des - pos_fb;
    
    std::cout << "force_des: " << force_des(0, 0) << ", " << force_des(1, 0) << std::endl;
    std::cout << "force_fb : " << force_state_->Fx << ", " << force_state_->Fy << std::endl;
    std::cout << "force_err: " << force_err(0, 0) << ", " << force_err(1, 0) << std::endl << std::endl;
    std::cout << "pos_des: " << pos_des(0, 0) << ", " << pos_des(1, 0) << std::endl;
    std::cout << "pos_fb : " << pos_fb(0, 0)  << ", " << pos_fb(1, 0)  << std::endl;
    std::cout << "pos_err: " << pos_err(0, 0) << ", " << pos_err(1, 0) << std::endl << std::endl;
    std::cout << "vel_fb : " << vel_fb(0, 0)  << ", " << vel_fb(1, 0)  << std::endl;
    std::cout << "acc_fb : " << acc_fb(0, 0)  << ", " << acc_fb(1, 0)  << std::endl << std::endl;

    // calculate jacobian
    Eigen::MatrixXd P_poly = Eigen::MatrixXd::Zero(2, 8);
    Eigen::MatrixXd P_poly_deriv = Eigen::MatrixXd::Zero(2, 7);
    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    legmodel.contact_map(eta_hist_.col(0)(0, 0), eta_hist_.col(0)(1, 0));
    P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
    
    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(eta_hist_.col(0)(0, 0), i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(eta_hist_.col(0)(0, 0), i); 
    
    Eigen::MatrixXd J_fb(2, 2);
    J_fb = calculate_jacobian(P_theta, P_theta_deriv, eta_hist_.col(0)(1, 0));

    // std::cout << "J_fb: " << std::endl
    //           << J_fb(0, 0) << ", " << J_fb(0, 1) << std::endl
    //           << J_fb(1, 0) << ", " << J_fb(1, 1) << std::endl << std::endl;

    // impedance control
    Eigen::MatrixXd M(2, 2);
    Eigen::MatrixXd B(2, 2);
    Eigen::MatrixXd K(2, 2);
    Eigen::MatrixXd eta_cmd(2, 1);
    Eigen::MatrixXd trq_cmd(2, 1);
    Eigen::MatrixXd kp_cmd(2, 1);
    Eigen::MatrixXd kd_cmd(2, 1);

    M << imp_cmd_->Mx, 0, 0, imp_cmd_->My;
    B << imp_cmd_->Bx, 0, 0, imp_cmd_->By;
    K << imp_cmd_->Kx, 0, 0, imp_cmd_->Ky;

    eta_cmd << imp_cmd_->theta, imp_cmd_->beta;
    trq_cmd = J_fb.transpose() * (force_des + M*(-acc_fb));
    std::cout << "trq_cmd : " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl << std::endl;

    // calculate phi command
    Eigen::MatrixXd phi_des(2, 1);
    phi_des << eta_cmd(1, 0) - eta_cmd(0, 0) + 17/180.0*M_PI,
               eta_cmd(1, 0) + eta_cmd(0, 0) - 17/180.0*M_PI;

    Eigen::MatrixXd phi_fb(2, 2);
    phi_fb.col(0) << eta_hist_.col(0)(1, 0) - eta_hist_.col(0)(0, 0) + 17/180.0*M_PI,
                     eta_hist_.col(0)(1, 0) + eta_hist_.col(0)(0, 0) - 17/180.0*M_PI;
    phi_fb.col(1) << eta_hist_.col(1)(1, 0) - eta_hist_.col(1)(0, 0) + 17/180.0*M_PI,
                     eta_hist_.col(1)(1, 0) + eta_hist_.col(1)(0, 0) - 17/180.0*M_PI;

    Eigen::MatrixXd phi_err = phi_des-phi_fb.col(0);
    Eigen::MatrixXd phi_vel = (phi_fb.col(0)-phi_fb.col(1))*1000;

    // kp compensate
    kp_cmd << J_fb(0, 0) * J_fb(0, 0) * K(0, 0) + J_fb(1, 0) * J_fb(1, 0) * K(1, 1),
              J_fb(0, 1) * J_fb(0, 1) * K(0, 0) + J_fb(1, 1) * J_fb(1, 1) * K(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(1, 0);
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(0, 0);

    // kd compensate
    kd_cmd << J_fb(0, 0) * J_fb(0, 0) * B(0, 0) + J_fb(0, 1) * J_fb(1, 0) * B(1, 1),
              J_fb(0, 1) * J_fb(1, 0) * B(0, 0) + J_fb(1, 1) * J_fb(1, 1) * B(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(1, 0) * B(0, 0) + J_fb(1, 0) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(1, 0));
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * B(0, 0) + J_fb(0, 1) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(0, 0));
    
    // torque version impedance control
    // trq_cmd << J_fb.transpose() * (force_err + M*(-acc_fb) + B*(-vel_fb));
    // kp_cmd << 0, 0;
    // kd_cmd << 0, 0;

    std::cout << "kp_cmd  : " << kp_cmd(0, 0) << ", " << kp_cmd(1, 0) << std::endl;
    std::cout << "phi_err : " << phi_err(0, 0) << ", " << phi_err(1, 0) << std::endl << std::endl;

    std::cout << "kd_cmd  : " << kd_cmd(0, 0) << ", " << kd_cmd(1, 0) << std::endl;
    std::cout << "phi_vel : " << phi_vel(0, 0) << ", " << phi_vel(1, 0) << std::endl << std::endl;

    // std::cout << "eta_cmd: " << eta_cmd(0, 0) << ", " << eta_cmd(1, 0) << std::endl;
    std::cout << "trq_kp  : " << kp_cmd(0, 0)*phi_err(0, 0) << ", " << kp_cmd(1, 0)*phi_err(1, 0) << std::endl;
    std::cout << "trq_kd  : " << -kd_cmd(0, 0)*phi_vel(0, 0) << ", " << -kd_cmd(1, 0)*phi_vel(1, 0) << std::endl;
    std::cout << "trq_cmd: " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl;

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

int main(int argc, char **argv) {

    ROS_INFO("Force Control Starts\n");

    ros::init(argc, argv, "force_control");

    ros::NodeHandle nh;
    ros::Subscriber imp_cmd_sub = nh.subscribe<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Rate rate(1000);


    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<Eigen::MatrixXd> eta_hist_modules = {
        Eigen::MatrixXd::Zero(2, 3),
        Eigen::MatrixXd::Zero(2, 3),
        Eigen::MatrixXd::Zero(2, 3),
        Eigen::MatrixXd::Zero(2, 3)
    };


    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        for (int i=0; i<1; i++){
            eta_hist_modules[i].col(2) = eta_hist_modules[i].col(1);
            eta_hist_modules[i].col(1) = eta_hist_modules[i].col(0);
            eta_hist_modules[i].col(0) << motor_state_modules[i]->theta, motor_state_modules[i]->beta;

            if (imp_cmd_modules[i]->theta < 17/180.0*M_PI) { imp_cmd_modules[i]->theta = 17/180.0*M_PI; }

            force_control(imp_cmd_modules[i], eta_hist_modules[i], force_state_modules[i], motor_cmd_modules[i]);
        }

        std::cout << "= = = = = = = = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();
        
        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}