#include "corgi_force_estimation/force_estimation.hpp"

// Global variable definitions
bool sim = true;
LegModel legmodel(sim);

Eigen::MatrixXd H_l_coef(2, 8);
Eigen::MatrixXd H_r_coef(2, 8);
Eigen::MatrixXd F_l_coef(2, 8);
Eigen::MatrixXd F_r_coef(2, 8);
Eigen::MatrixXd U_l_coef(2, 8);
Eigen::MatrixXd U_r_coef(2, 8);
Eigen::MatrixXd L_l_coef(2, 8);
Eigen::MatrixXd L_r_coef(2, 8);
Eigen::MatrixXd G_coef(2, 8);

Eigen::MatrixXd calculate_P_poly(int rim, double alpha){
    for (int i=0; i<8; i++){
        H_l_coef(0, i) = H_x_coef[i];
        H_l_coef(1, i) = H_y_coef[i];

        H_r_coef(0, i) = -H_x_coef[i];
        H_r_coef(1, i) = H_y_coef[i];

        U_l_coef(0, i) = U_x_coef[i];
        U_l_coef(1, i) = U_y_coef[i];

        U_r_coef(0, i) = -U_x_coef[i];
        U_r_coef(1, i) = U_y_coef[i];
        
        L_l_coef(0, i) = L_x_coef[i];
        L_l_coef(1, i) = L_y_coef[i];
        
        L_r_coef(0, i) = -L_x_coef[i];
        L_r_coef(1, i) = L_y_coef[i];
        
        F_l_coef(0, i) = F_x_coef[i];
        F_l_coef(1, i) = F_y_coef[i];
        
        F_r_coef(0, i) = -F_x_coef[i];
        F_r_coef(1, i) = F_y_coef[i];
        
        G_coef(0, i) = 0;
        G_coef(1, i) = G_y_coef[i];
    }

    Eigen::MatrixXd P_poly(2, 8);

    double scaled_radius = legmodel.radius / legmodel.R;

    if (rim == 1 && alpha > -M_PI*2.0/3.0) {
    // if (rim == 1) {
        Eigen::Rotation2D<double> rotation(alpha+M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_l_coef-U_l_coef) * scaled_radius + U_l_coef;
    }
    else if (rim == 2) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_l_coef) * scaled_radius + L_l_coef;
    }
    else if (rim == 3) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_l_coef) * legmodel.r / legmodel.R + G_coef;
    }
    else if (rim == 4) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_r_coef) * scaled_radius + L_r_coef;
    }
    else if (rim == 5 && alpha < M_PI*2.0/3.0) {
    // else if (rim == 5) {
        Eigen::Rotation2D<double> rotation(alpha-M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_r_coef-U_r_coef) * scaled_radius + U_r_coef;
    }
    else {
        P_poly = Eigen::MatrixXd::Zero(2, 8);
    }
    
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
    
    Eigen::MatrixXd force_est(2, 1);

    if (jacobian.isZero(1e-6)) {
        force_est.setZero();
    } else {
        Eigen::MatrixXd torque(2, 1);
        torque << torque_r, torque_l;
        force_est = jacobian.inverse().transpose() * torque;
    }

    return force_est;
}

void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

