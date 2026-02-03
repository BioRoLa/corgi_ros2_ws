#include "general_momentum_observer/DataProcessor.hpp"
#include "Config.hpp"
#include <cmath>

DataProcessor::DataProcessor(double dt) : dt_(dt) {
    // No need for state tracking since velocities come from motor_state
}

DataProcessor::ProcessedData DataProcessor::process_realtime_data(
    const geometry_msgs::msg::Vector3& position,
    const geometry_msgs::msg::Vector3& velocity,
    const corgi_msgs::msg::ImuStamped& imu,
    const corgi_msgs::msg::MotorStateStamped& motor_state
) {
    ProcessedData result;
    
    // Initialize vectors
    result.q = Eigen::VectorXd::Zero(12);
    result.q_dot = Eigen::VectorXd::Zero(12);
    result.tau = Eigen::VectorXd(8);
    result.I_c = Eigen::VectorXd(4);
    
    // Base position (x, z)
    result.q(0) = position.x;
    result.q(1) = position.z;
    
    // Base velocities from subscribed velocity topic
    result.q_dot(0) = velocity.x;
    result.q_dot(1) = velocity.z;
    
    // IMU to Euler angles (Quaternion -> Euler)
    double roll, pitch;
    cal_quaternion_to_euler(
        imu.orientation.x, imu.orientation.y,
        imu.orientation.z, imu.orientation.w,
        roll, pitch
    );
    result.q(2) = roll;
    result.q(3) = pitch;
    
    // Base angular velocity (from IMU)
    result.q_dot(2) = imu.angular_velocity.x;  // roll rate
    result.q_dot(3) = imu.angular_velocity.y;  // pitch rate
    
    // Extract motor states
    double theta_a = motor_state.module_a.theta;
    double beta_a = motor_state.module_a.beta;
    double theta_b = motor_state.module_b.theta;
    double beta_b = motor_state.module_b.beta;
    double theta_c = motor_state.module_c.theta;
    double beta_c = motor_state.module_c.beta;
    double theta_d = motor_state.module_d.theta;
    double beta_d = motor_state.module_d.beta;
    
    // Leg joint angles - Convert theta to Rm
    result.q(4) = beta_a;
    result.q(5) = cal_theta_to_Rm(theta_a);
    result.q(6) = -beta_b;  // RF needs negation
    result.q(7) = cal_theta_to_Rm(theta_b);
    result.q(8) = -beta_c;  // RH needs negation
    result.q(9) = cal_theta_to_Rm(theta_c);
    result.q(10) = beta_d;
    result.q(11) = cal_theta_to_Rm(theta_d);
    
    // Leg joint velocities from motor_state (velocity_r, velocity_l)
    // Convert motor velocities to joint velocities: phi_r, phi_l -> theta, beta
    // theta_dot = (- velocity_r + velocity_l)/2, beta_dot = (velocity_r + velocity_l)/2
    double theta_a_dot = (- motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
    double beta_a_dot = (motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
    
    double theta_b_dot = (- motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
    double beta_b_dot = (motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
    beta_b_dot = -beta_b_dot;  // RF beta velocity needs negation
    
    double theta_c_dot = (- motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
    double beta_c_dot = (motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
    beta_c_dot = -beta_c_dot;  // RH beta velocity needs negation
    
    double theta_d_dot = (- motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
    double beta_d_dot = (motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
    
    // Convert theta_dot to Rm_dot
    result.q_dot(4) = beta_a_dot;
    result.q_dot(5) = cal_theta_dot_to_Rm_dot(theta_a, theta_a_dot);
    result.q_dot(6) = beta_b_dot;
    result.q_dot(7) = cal_theta_dot_to_Rm_dot(theta_b, theta_b_dot);
    result.q_dot(8) = beta_c_dot;
    result.q_dot(9) = cal_theta_dot_to_Rm_dot(theta_c, theta_c_dot);
    result.q_dot(10) = beta_d_dot;
    result.q_dot(11) = cal_theta_dot_to_Rm_dot(theta_d, theta_d_dot);
    
    // Convert motor torques to joint space torques
    double torque_beta_a, F_Rm_a;
    cal_motor_to_joint_torque(theta_a, 
        motor_state.module_a.torque_r, motor_state.module_a.torque_l, 
        torque_beta_a, F_Rm_a);
    
    double torque_beta_b, F_Rm_b;
    cal_motor_to_joint_torque(theta_b,
        motor_state.module_b.torque_r, motor_state.module_b.torque_l,
        torque_beta_b, F_Rm_b);
    
    double torque_beta_c, F_Rm_c;
    cal_motor_to_joint_torque(theta_c,
        motor_state.module_c.torque_r, motor_state.module_c.torque_l,
        torque_beta_c, F_Rm_c);
    
    double torque_beta_d, F_Rm_d;
    cal_motor_to_joint_torque(theta_d,
        motor_state.module_d.torque_r, motor_state.module_d.torque_l,
        torque_beta_d, F_Rm_d);
    
    // Joint torques in generalized coordinates
    result.tau(0) = torque_beta_a;
    result.tau(1) = F_Rm_a;
    result.tau(2) = -torque_beta_b;  // RF beta torque needs negation
    result.tau(3) = F_Rm_b;
    result.tau(4) = -torque_beta_c;  // RH beta torque needs negation
    result.tau(5) = F_Rm_c;
    result.tau(6) = torque_beta_d;
    result.tau(7) = F_Rm_d;
    
    // Compute leg inertia
    result.I_c(0) = cal_theta_to_Ic(theta_a);
    result.I_c(1) = cal_theta_to_Ic(theta_b);
    result.I_c(2) = cal_theta_to_Ic(theta_c);
    result.I_c(3) = cal_theta_to_Ic(theta_d);
    
    return result;
}

void DataProcessor::cal_quaternion_to_euler(double x, double y, double z, double w, double& roll, double& pitch) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);
}

double DataProcessor::cal_theta_to_Rm(double theta) {
    const auto& A = quadruped::Config::RM_COEFF;
    double Rm = 0.0;
    double theta_power = 1.0;
    for (int i = A.size() - 1; i >= 0; --i) {
        Rm += A[i] * theta_power;
        theta_power *= theta;
    }
    return Rm;
}

double DataProcessor::cal_theta_dot_to_Rm_dot(double theta, double theta_dot) {
    const auto& A = quadruped::Config::RM_COEFF;
    double dRm_dtheta = 0.0;
    double theta_power = 1.0;
    
    for (size_t i = 1; i < A.size(); ++i) {
        dRm_dtheta += A[i] * (A.size() - 1 - i) * theta_power;
        theta_power *= theta;
    }
    
    return dRm_dtheta * theta_dot;
}

double DataProcessor::cal_theta_to_Ic(double theta) {
    const auto& B = quadruped::Config::IC_COEFF;
    double Ic = 0.0;
    double theta_power = 1.0;
    for (int i = B.size() - 1; i >= 0; --i) {
        Ic += B[i] * theta_power;
        theta_power *= theta;
    }
    return Ic;
}

void DataProcessor::cal_motor_to_joint_torque(double theta, double torque_right, double torque_left,
                                    double& torque_beta, double& force_Rm) {
    const auto& A = quadruped::Config::RM_COEFF;
    double dRm_dtheta = 0.0;
    double theta_power = 1.0;
    
    for (size_t i = 1; i < A.size(); ++i) {
        dRm_dtheta += A[i] * (A.size() - 1 - i) * theta_power;
        theta_power *= theta;
    }
    
    // Virtual work transformation
    double temp1 = torque_left - torque_right;
    double temp2 = torque_left + torque_right;
    
    if (std::abs(dRm_dtheta) < 1e-9) {
        force_Rm = 0.0;
        torque_beta = 0.0;
        return;
    }
    
    force_Rm = temp1 / dRm_dtheta;
    torque_beta = temp2;
}
