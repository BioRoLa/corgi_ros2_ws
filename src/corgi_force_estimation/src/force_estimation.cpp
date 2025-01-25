#include "force_estimation.hpp"


corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::ForceStateStamped force_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

Eigen::MatrixXd calculate_P_poly(int rim, double alpha){
    Eigen::Rotation2D<double> rotation(alpha);
    Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();

    for (int i=0; i<8; i++){
        H_l_coef(0, i) = H_x_coef[i];
        H_l_coef(1, i) = H_y_coef[i];
        U_l_coef(0, i) = U_x_coef[i];
        U_l_coef(1, i) = U_y_coef[i];
        F_l_coef(0, i) = F_x_coef[i];
        F_l_coef(1, i) = F_y_coef[i];
        G_coef(0, i) = 0;
        G_coef(1, i) = G_y_coef[i];
    }

    L_r_coef = L_l_coef * (-1, 1);
    F_r_coef = F_l_coef * (-1, 1);
    U_r_coef = U_l_coef * (-1, 1);


    Eigen::MatrixXd P_poly(2, 8);

    double scaled_radius = legmodel.radius / legmodel.R;

    if      (rim == 1) P_poly = rot_alpha * (H_l_coef-U_l_coef) * scaled_radius + U_l_coef;
    else if (rim == 2) P_poly = rot_alpha * (F_l_coef-L_l_coef) * scaled_radius + L_l_coef;
    else if (rim == 3) P_poly = G_coef * scaled_radius;
    else if (rim == 4) P_poly = rot_alpha * (G_coef-L_r_coef) * scaled_radius + L_r_coef;
    else if (rim == 5) P_poly = rot_alpha * (F_r_coef-U_r_coef) * scaled_radius + U_r_coef;
    else P_poly = Eigen::MatrixXd::Zero(2, 8);
    
    return P_poly;
}

Eigen::MatrixXd calculate_jacobian(Eigen::MatrixXd P_theta, Eigen::MatrixXd P_theta_deriv, double beta){
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

    double J11 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J12 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J21 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;
    double J22 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;

    Eigen::MatrixXd jacobian(2, 2);
    jacobian << J11, J12, J21, J22;

    return jacobian;
}

Eigen::MatrixXd estimate_force(double theta, double beta, double torque_r, double torque_l){
    legmodel.contact_map(theta, beta);

    Eigen::MatrixXd P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
    Eigen::MatrixXd P_poly_deriv(2, 7);

    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(theta, i); 
    
    Eigen::MatrixXd jacobian(2, 2);
    jacobian = calculate_jacobian(P_theta, P_theta_deriv, beta);
    
    Eigen::MatrixXd torque(2, 1);
    torque << torque_r, torque_l;

    Eigen::MatrixXd force_est = jacobian.inverse().transpose() * torque;

    return force_est;
}


int main(int argc, char **argv) {

    ROS_INFO("Force Estimation Starts\n");

    ros::init(argc, argv, "force_estimation");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Publisher force_state_pub = nh.advertise<corgi_msgs::ForceStateStamped>("force/state", 1000);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    while (ros::ok()) {
        ros::spinOnce();

        for (int i=0; i<4; i++){
            Eigen::MatrixXd force_est = estimate_force(motor_state_modules[i]->theta, motor_state_modules[i]->beta,
                                                       motor_state_modules[i]->torque_r, motor_state_modules[i]->torque_l);

            force_state_modules[i]->Fx = force_est(0, 0);
            force_state_modules[i]->Fy = force_est(1, 0);
        }

        force_state.header.seq = motor_state.header.seq;
        force_state.header.stamp = ros::Time::now();
        
        force_state_pub.publish(force_state);

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}