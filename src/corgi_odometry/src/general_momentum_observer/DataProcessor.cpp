#include "general_momentum_observer/DataProcessor.hpp"
#include "common/Config.hpp"
#include <cmath>

DataProcessor::DataProcessor(double dt, double encoder_cutoff_freq)
    : dt_(dt), low_pass_alpha_(1.0 - std::exp(-2.0 * M_PI * encoder_cutoff_freq * dt_))
{
}

// ─────────────────────────────────────────────────────────────────
// Offline: process a single CSV record (stateful velocity LPF)
// ─────────────────────────────────────────────────────────────────
DataProcessor::ProcessedData DataProcessor::process_record(const RawRecord &data)
{
    ProcessedData result;
    result.q = Eigen::VectorXd::Zero(16);
    result.q_dot = Eigen::VectorXd::Zero(16);
    result.tau = Eigen::VectorXd::Zero(12);
    result.I_c = Eigen::VectorXd(4);

    // Base position
    result.q(0) = data.sim_pos_x;
    result.q(1) = data.sim_pos_z;

    // Base velocities with causal IIR low-pass filter
    double x_dot = (data.sim_pos_x - last_sim_pos_x_) / dt_;
    double z_dot = (data.sim_pos_z - last_sim_pos_z_) / dt_;
    if (!first_call_)
    {
        x_dot = (1.0 - low_pass_alpha_) * last_x_dot_ + low_pass_alpha_ * x_dot;
        z_dot = (1.0 - low_pass_alpha_) * last_z_dot_ + low_pass_alpha_ * z_dot;
    }
    result.q_dot(0) = x_dot;
    result.q_dot(1) = z_dot;
    last_sim_pos_x_ = data.sim_pos_x;
    last_sim_pos_z_ = data.sim_pos_z;
    last_x_dot_ = x_dot;
    last_z_dot_ = z_dot;

    // Quaternion → Euler
    double roll, pitch;
    cal_quaternion_to_euler(data.imu_orien_x, data.imu_orien_y,
                            data.imu_orien_z, data.imu_orien_w, roll, pitch);
    result.q(2) = roll;
    result.q(3) = pitch;

    // Leg joint angles (theta→Rm, right-side beta negated)
    result.q(4) = data.state_beta_a;
    result.q(5) = cal_theta_to_Rm(data.state_theta_a);
    result.q(6) = -data.state_beta_b;
    result.q(7) = cal_theta_to_Rm(data.state_theta_b);
    result.q(8) = -data.state_beta_c;
    result.q(9) = cal_theta_to_Rm(data.state_theta_c);
    result.q(10) = data.state_beta_d;
    result.q(11) = cal_theta_to_Rm(data.state_theta_d);
    result.q(12) = data.state_gamma_a;
    result.q(13) = data.state_gamma_b;
    result.q(14) = data.state_gamma_c;
    result.q(15) = data.state_gamma_d;

    // Angular velocity
    result.q_dot(2) = data.imu_ang_vel_x;
    result.q_dot(3) = data.imu_ang_vel_y;

    // Motor velocities → joint velocities
    double theta_a_dot = (-data.state_vel_r_a + data.state_vel_l_a) / 2.0;
    double beta_a_dot = (data.state_vel_r_a + data.state_vel_l_a) / 2.0;
    double gamma_a_dot = data.state_vel_h_a;
    double theta_b_dot = (-data.state_vel_r_b + data.state_vel_l_b) / 2.0;
    double beta_b_dot = (data.state_vel_r_b + data.state_vel_l_b) / 2.0;
    double gamma_b_dot = data.state_vel_h_b;
    beta_b_dot = -beta_b_dot;
    double theta_c_dot = (-data.state_vel_r_c + data.state_vel_l_c) / 2.0;
    double beta_c_dot = (data.state_vel_r_c + data.state_vel_l_c) / 2.0;
    double gamma_c_dot = data.state_vel_h_c;
    beta_c_dot = -beta_c_dot;
    double theta_d_dot = (-data.state_vel_r_d + data.state_vel_l_d) / 2.0;
    double beta_d_dot = (data.state_vel_r_d + data.state_vel_l_d) / 2.0;
    double gamma_d_dot = data.state_vel_h_d;

    result.q_dot(4) = beta_a_dot;
    result.q_dot(5) = cal_theta_dot_to_Rm_dot(data.state_theta_a, theta_a_dot);
    result.q_dot(6) = beta_b_dot;
    result.q_dot(7) = cal_theta_dot_to_Rm_dot(data.state_theta_b, theta_b_dot);
    result.q_dot(8) = beta_c_dot;
    result.q_dot(9) = cal_theta_dot_to_Rm_dot(data.state_theta_c, theta_c_dot);
    result.q_dot(10) = beta_d_dot;
    result.q_dot(11) = cal_theta_dot_to_Rm_dot(data.state_theta_d, theta_d_dot);
    result.q_dot(12) = gamma_a_dot;
    result.q_dot(13) = gamma_b_dot;
    result.q_dot(14) = gamma_c_dot;
    result.q_dot(15) = gamma_d_dot;

    // Motor torques → joint torques
    double torque_beta_a, F_Rm_a;
    cal_motor_to_joint_torque(data.state_theta_a, data.state_trq_r_a, data.state_trq_l_a, torque_beta_a, F_Rm_a);
    double torque_beta_b, F_Rm_b;
    cal_motor_to_joint_torque(data.state_theta_b, data.state_trq_r_b, data.state_trq_l_b, torque_beta_b, F_Rm_b);
    double torque_beta_c, F_Rm_c;
    cal_motor_to_joint_torque(data.state_theta_c, data.state_trq_r_c, data.state_trq_l_c, torque_beta_c, F_Rm_c);
    double torque_beta_d, F_Rm_d;
    cal_motor_to_joint_torque(data.state_theta_d, data.state_trq_r_d, data.state_trq_l_d, torque_beta_d, F_Rm_d);

    result.tau(0) = torque_beta_a;
    result.tau(1) = F_Rm_a;
    result.tau(2) = data.state_trq_h_a;
    result.tau(3) = -torque_beta_b;
    result.tau(4) = F_Rm_b;
    result.tau(5) = data.state_trq_h_b;
    result.tau(6) = -torque_beta_c;
    result.tau(7) = F_Rm_c;
    result.tau(8) = data.state_trq_h_c;
    result.tau(9) = torque_beta_d;
    result.tau(10) = F_Rm_d;
    result.tau(11) = data.state_trq_h_d;

    // Leg inertias
    result.I_c(0) = cal_theta_to_Ic(data.state_theta_a);
    result.I_c(1) = cal_theta_to_Ic(data.state_theta_b);
    result.I_c(2) = cal_theta_to_Ic(data.state_theta_c);
    result.I_c(3) = cal_theta_to_Ic(data.state_theta_d);

    first_call_ = false;
    return result;
}

DataProcessor::ProcessedData DataProcessor::process_realtime_data(
    const geometry_msgs::msg::Vector3 &position,
    const geometry_msgs::msg::Vector3 &velocity,
    const corgi_msgs::msg::ImuStamped &imu,
    const corgi_msgs::msg::MotorStateStamped &motor_state)
{
    ProcessedData result;

    // Initialize vectors
    result.q = Eigen::VectorXd::Zero(16);
    result.q_dot = Eigen::VectorXd::Zero(16);
    result.tau = Eigen::VectorXd::Zero(12);
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
        roll, pitch);
    result.q(2) = roll;
    result.q(3) = pitch;

    // Base angular velocity (from IMU)
    result.q_dot(2) = imu.angular_velocity.x; // roll rate
    result.q_dot(3) = imu.angular_velocity.y; // pitch rate

    // Extract motor states
    double theta_a = motor_state.module_a.theta;
    double beta_a = motor_state.module_a.beta;
    double gamma_a = motor_state.module_a.gamma;
    double theta_b = motor_state.module_b.theta;
    double beta_b = motor_state.module_b.beta;
    double gamma_b = motor_state.module_b.gamma;
    double theta_c = motor_state.module_c.theta;
    double beta_c = motor_state.module_c.beta;
    double gamma_c = motor_state.module_c.gamma;
    double theta_d = motor_state.module_d.theta;
    double beta_d = motor_state.module_d.beta;
    double gamma_d = motor_state.module_d.gamma;

    // Leg joint angles - Convert theta to Rm
    result.q(4) = beta_a;
    result.q(5) = cal_theta_to_Rm(theta_a);
    result.q(6) = -beta_b; // RF needs negation
    result.q(7) = cal_theta_to_Rm(theta_b);
    result.q(8) = -beta_c; // RH needs negation
    result.q(9) = cal_theta_to_Rm(theta_c);
    result.q(10) = beta_d;
    result.q(11) = cal_theta_to_Rm(theta_d);
    result.q(12) = gamma_a;
    result.q(13) = gamma_b;
    result.q(14) = gamma_c;
    result.q(15) = gamma_d;

    // Leg joint velocities from motor_state (velocity_r, velocity_l)
    // Convert motor velocities to joint velocities: phi_r, phi_l -> theta, beta
    // theta_dot = (- velocity_r + velocity_l)/2, beta_dot = (velocity_r + velocity_l)/2
    double theta_a_dot = (-motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
    double beta_a_dot = (motor_state.module_a.velocity_r + motor_state.module_a.velocity_l) / 2.0;
    double gamma_a_dot = motor_state.module_a.velocity_h;

    double theta_b_dot = (-motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
    double beta_b_dot = (motor_state.module_b.velocity_r + motor_state.module_b.velocity_l) / 2.0;
    double gamma_b_dot = motor_state.module_b.velocity_h;
    beta_b_dot = -beta_b_dot; // RF beta velocity needs negation

    double theta_c_dot = (-motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
    double beta_c_dot = (motor_state.module_c.velocity_r + motor_state.module_c.velocity_l) / 2.0;
    double gamma_c_dot = motor_state.module_c.velocity_h;
    beta_c_dot = -beta_c_dot; // RH beta velocity needs negation

    double theta_d_dot = (-motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
    double beta_d_dot = (motor_state.module_d.velocity_r + motor_state.module_d.velocity_l) / 2.0;
    double gamma_d_dot = motor_state.module_d.velocity_h;

    // Convert theta_dot to Rm_dot
    result.q_dot(4) = beta_a_dot;
    result.q_dot(5) = cal_theta_dot_to_Rm_dot(theta_a, theta_a_dot);
    result.q_dot(6) = beta_b_dot;
    result.q_dot(7) = cal_theta_dot_to_Rm_dot(theta_b, theta_b_dot);
    result.q_dot(8) = beta_c_dot;
    result.q_dot(9) = cal_theta_dot_to_Rm_dot(theta_c, theta_c_dot);
    result.q_dot(10) = beta_d_dot;
    result.q_dot(11) = cal_theta_dot_to_Rm_dot(theta_d, theta_d_dot);
    result.q_dot(12) = gamma_a_dot;
    result.q_dot(13) = gamma_b_dot;
    result.q_dot(14) = gamma_c_dot;
    result.q_dot(15) = gamma_d_dot;

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
    result.tau(2) = motor_state.module_a.torque_h;
    result.tau(3) = -torque_beta_b; // RF beta torque needs negation
    result.tau(4) = F_Rm_b;
    result.tau(5) = motor_state.module_b.torque_h;
    result.tau(6) = -torque_beta_c; // RH beta torque needs negation
    result.tau(7) = F_Rm_c;
    result.tau(8) = motor_state.module_c.torque_h;
    result.tau(9) = torque_beta_d;
    result.tau(10) = F_Rm_d;
    result.tau(11) = motor_state.module_d.torque_h;

    // Compute leg inertia
    result.I_c(0) = cal_theta_to_Ic(theta_a);
    result.I_c(1) = cal_theta_to_Ic(theta_b);
    result.I_c(2) = cal_theta_to_Ic(theta_c);
    result.I_c(3) = cal_theta_to_Ic(theta_d);

    return result;
}

void DataProcessor::cal_quaternion_to_euler(double x, double y, double z, double w, double &roll, double &pitch)
{
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

double DataProcessor::cal_theta_to_Rm(double theta)
{
    // Horner's method: A[0]*x^4 + A[1]*x^3 + ... + A[4]
    const auto &A = corgi::Config::RM_COEFF;
    double result = A[0];
    for (size_t i = 1; i < A.size(); ++i)
        result = result * theta + A[i];
    return result;
}

double DataProcessor::cal_theta_dot_to_Rm_dot(double theta, double theta_dot)
{
    // Horner's method on derivative: d/dx[A0*x^4+...+A4] = 4*A0*x^3+3*A1*x^2+2*A2*x+A3
    const auto &A = corgi::Config::RM_COEFF;
    const int n = static_cast<int>(A.size()) - 1; // degree = 4
    double deriv = n * A[0];
    for (int i = 1; i < n; ++i)
        deriv = deriv * theta + (n - i) * A[i];
    return deriv * theta_dot;
}

double DataProcessor::cal_theta_to_Ic(double theta)
{
    // Horner's method: B[0]*x^6 + B[1]*x^5 + ... + B[6]
    const auto &B = corgi::Config::IC_COEFF;
    double result = B[0];
    for (size_t i = 1; i < B.size(); ++i)
        result = result * theta + B[i];
    return result;
}

void DataProcessor::cal_motor_to_joint_torque(double theta, double torque_right, double torque_left,
                                              double &torque_beta, double &force_Rm)
{
    const auto &A = corgi::Config::RM_COEFF;
    // Horner's method on derivative
    const int n = static_cast<int>(A.size()) - 1;
    double dRm_dtheta = n * A[0];
    for (int i = 1; i < n; ++i)
        dRm_dtheta = dRm_dtheta * theta + (n - i) * A[i];

    // Virtual work transformation
    double temp1 = torque_left - torque_right;
    double temp2 = torque_left + torque_right;

    if (std::abs(dRm_dtheta) < 1e-9)
    {
        force_Rm = 0.0;
        torque_beta = 0.0;
        return;
    }

    force_Rm = temp1 / dRm_dtheta;
    torque_beta = temp2;
}
