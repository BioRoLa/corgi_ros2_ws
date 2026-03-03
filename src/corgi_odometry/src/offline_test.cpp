#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include <map>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "es_ekf/ESEKF.hpp"
#include "kinematic/ContactMap.hpp"
#include "kinematic/Leg.hpp"
#include "Config.hpp"

/**
 * @brief CSV Reader with column name support
 */
class CSVReader {
public:
    struct RobotData {
        // Base position
        double sim_pos_x, sim_pos_y, sim_pos_z;
        
        // IMU data
        double imu_orien_x, imu_orien_y, imu_orien_z, imu_orien_w;
        double imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z;
        double imu_lin_acc_x, imu_lin_acc_y, imu_lin_acc_z;
        
        // Leg data (LF, RF, RH, LH)
        double state_theta_a, state_beta_a, state_vel_r_a, state_vel_l_a, state_trq_r_a, state_trq_l_a;
        double state_theta_b, state_beta_b, state_vel_r_b, state_vel_l_b, state_trq_r_b, state_trq_l_b;
        double state_theta_c, state_beta_c, state_vel_r_c, state_vel_l_c, state_trq_r_c, state_trq_l_c;
        double state_theta_d, state_beta_d, state_vel_r_d, state_vel_l_d, state_trq_r_d, state_trq_l_d;
    };
    
    std::vector<RobotData> read_csv(const std::string& filename) {
        std::vector<RobotData> data;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        // Read header line and parse column names
        std::string header_line;
        std::getline(file, header_line);
        auto column_indices = parse_header(header_line);
        
        // Read data lines
        std::string line;
        while (std::getline(file, line)) {
            auto values = parse_line(line);
            
            if (values.size() <= column_indices.at("sim_pos_z")) {
                std::cerr << "Warning: Skipping malformed line\n";
                continue;
            }
            
            RobotData row;
            
            // Extract values by column index
            row.sim_pos_x = values[column_indices.at("sim_pos_x")];
            row.sim_pos_y = values[column_indices.at("sim_pos_y")];
            row.sim_pos_z = values[column_indices.at("sim_pos_z")];
            
            row.imu_orien_x = values[column_indices.at("imu_orien_x")];
            row.imu_orien_y = values[column_indices.at("imu_orien_y")];
            row.imu_orien_z = values[column_indices.at("imu_orien_z")];
            row.imu_orien_w = values[column_indices.at("imu_orien_w")];
            row.imu_ang_vel_x = values[column_indices.at("imu_ang_vel_x")];
            row.imu_ang_vel_y = values[column_indices.at("imu_ang_vel_y")];
            row.imu_ang_vel_z = values[column_indices.at("imu_ang_vel_z")];
            row.imu_lin_acc_x = values[column_indices.at("imu_lin_acc_x")];
            row.imu_lin_acc_y = values[column_indices.at("imu_lin_acc_y")];
            row.imu_lin_acc_z = values[column_indices.at("imu_lin_acc_z")];
            
            row.state_theta_a = values[column_indices.at("state_theta_a")];
            row.state_beta_a = values[column_indices.at("state_beta_a")];
            row.state_vel_r_a = values[column_indices.at("state_vel_r_a")];
            row.state_vel_l_a = values[column_indices.at("state_vel_l_a")];
            row.state_trq_r_a = values[column_indices.at("state_trq_r_a")];
            row.state_trq_l_a = values[column_indices.at("state_trq_l_a")];
            
            row.state_theta_b = values[column_indices.at("state_theta_b")];
            row.state_beta_b = values[column_indices.at("state_beta_b")];
            row.state_vel_r_b = values[column_indices.at("state_vel_r_b")];
            row.state_vel_l_b = values[column_indices.at("state_vel_l_b")];
            row.state_trq_r_b = values[column_indices.at("state_trq_r_b")];
            row.state_trq_l_b = values[column_indices.at("state_trq_l_b")];
            
            row.state_theta_c = values[column_indices.at("state_theta_c")];
            row.state_beta_c = values[column_indices.at("state_beta_c")];
            row.state_vel_r_c = values[column_indices.at("state_vel_r_c")];
            row.state_vel_l_c = values[column_indices.at("state_vel_l_c")];
            row.state_trq_r_c = values[column_indices.at("state_trq_r_c")];
            row.state_trq_l_c = values[column_indices.at("state_trq_l_c")];
            
            row.state_theta_d = values[column_indices.at("state_theta_d")];
            row.state_beta_d = values[column_indices.at("state_beta_d")];
            row.state_vel_r_d = values[column_indices.at("state_vel_r_d")];
            row.state_vel_l_d = values[column_indices.at("state_vel_l_d")];
            row.state_trq_r_d = values[column_indices.at("state_trq_r_d")];
            row.state_trq_l_d = values[column_indices.at("state_trq_l_d")];
            
            data.push_back(row);
        }
        
        return data;
    }
    
private:
    std::map<std::string, size_t> parse_header(const std::string& header) {
        std::map<std::string, size_t> indices;
        std::stringstream ss(header);
        std::string column_name;
        size_t index = 0;
        
        while (std::getline(ss, column_name, ',')) {
            indices[column_name] = index++;
        }
        
        return indices;
    }
    
    std::vector<double> parse_line(const std::string& line) {
        std::vector<double> values;
        std::stringstream ss(line);
        std::string value_str;
        
        while (std::getline(ss, value_str, ',')) {
            try {
                values.push_back(std::stod(value_str));
            } catch (...) {
                values.push_back(0.0);  // Default value for parsing errors
            }
        }
        
        return values;
    }
};

/**
 * @brief Data Processor (corresponds to Python's DataProcessor)
 */
class DataProcessor {
public:
    DataProcessor(double dt) : dt_(dt), first_call_(true) {
        // Calculate low pass filter alpha from Config
        low_pass_alpha_ = 1.0 - std::exp(-2.0 * M_PI * corgi::Config::ENCODER_CUTOFF_FREQ * dt_);
        
        // Initialize last state variables
        last_sim_pos_x_ = 0.0;
        last_sim_pos_z_ = 0.0;
        
        // Initialize last velocity variables
        last_x_dot_ = 0.0;
        last_z_dot_ = 0.0;
    }
    
    struct ProcessedData {
        Eigen::VectorXd q;      // (12,)
        Eigen::VectorXd q_dot;  // (12,)
        Eigen::VectorXd tau;    // (8,)
        Eigen::VectorXd I_c;    // (4,)
    };
    
    ProcessedData process_record_data(const CSVReader::RobotData& data) {
        ProcessedData result;
        
        // Initialize vectors
        result.q = Eigen::VectorXd::Zero(12);
        result.q_dot = Eigen::VectorXd::Zero(12);
        result.tau = Eigen::VectorXd(8);
        result.I_c = Eigen::VectorXd(4);
        
        // Base position
        result.q(0) = data.sim_pos_x;
        result.q(1) = data.sim_pos_z;
        
        // Calculate base velocities with low-pass filtering
        double x_dot = calculate_velocity(data.sim_pos_x, last_sim_pos_x_);
        double z_dot = calculate_velocity(data.sim_pos_z, last_sim_pos_z_);
        
        if (!first_call_) {
            x_dot = (1.0 - low_pass_alpha_) * last_x_dot_ + low_pass_alpha_ * x_dot;
            z_dot = (1.0 - low_pass_alpha_) * last_z_dot_ + low_pass_alpha_ * z_dot;
        }
        
        result.q_dot(0) = x_dot;
        result.q_dot(1) = z_dot;
        
        last_sim_pos_x_ = data.sim_pos_x;
        last_sim_pos_z_ = data.sim_pos_z;
        last_x_dot_ = x_dot;
        last_z_dot_ = z_dot;
        
        // IMU to Euler angles (Quaternion -> Euler)
        double roll, pitch;
        cal_quaternion_to_euler(
            data.imu_orien_x, data.imu_orien_y, 
            data.imu_orien_z, data.imu_orien_w,
            roll, pitch
        );
        result.q(2) = roll;
        result.q(3) = pitch;
        
        // Leg joint angles - CRITICAL: Use Rm not theta!
        result.q(4) = data.state_beta_a;
        result.q(5) = cal_theta_to_Rm(data.state_theta_a);  // Convert theta to Rm
        result.q(6) = -data.state_beta_b;  // RF needs negation
        result.q(7) = cal_theta_to_Rm(data.state_theta_b);
        result.q(8) = -data.state_beta_c;  // RH needs negation
        result.q(9) = cal_theta_to_Rm(data.state_theta_c);
        result.q(10) = data.state_beta_d;
        result.q(11) = cal_theta_to_Rm(data.state_theta_d);
        
        // Base angular velocity (from IMU)
        result.q_dot(2) = data.imu_ang_vel_x;  // roll rate
        result.q_dot(3) = data.imu_ang_vel_y;  // pitch rate
        
        // Leg joint velocities from motor velocities (matching corgi_leg_odom.cpp)
        // theta_d = (-velocity_r + velocity_l) / 2
        // beta_d  = -(velocity_r + velocity_l) / 2   (negated for right-side legs)
        double theta_a_dot = (-data.state_vel_r_a + data.state_vel_l_a) / 2.0;
        double beta_a_dot  = -(data.state_vel_r_a + data.state_vel_l_a) / 2.0;
        double theta_b_dot = (-data.state_vel_r_b + data.state_vel_l_b) / 2.0;
        double beta_b_dot  = -(data.state_vel_r_b + data.state_vel_l_b) / 2.0;
        beta_b_dot = -beta_b_dot;  // RF is right-side, negate beta_d
        double theta_c_dot = (-data.state_vel_r_c + data.state_vel_l_c) / 2.0;
        double beta_c_dot  = -(data.state_vel_r_c + data.state_vel_l_c) / 2.0;
        beta_c_dot = -beta_c_dot;  // RH is right-side, negate beta_d
        double theta_d_dot = (-data.state_vel_r_d + data.state_vel_l_d) / 2.0;
        double beta_d_dot  = -(data.state_vel_r_d + data.state_vel_l_d) / 2.0;
        
        // Convert theta_dot to Rm_dot
        result.q_dot(4) = beta_a_dot;
        result.q_dot(5) = cal_theta_dot_to_Rm_dot(data.state_theta_a, theta_a_dot);
        result.q_dot(6) = beta_b_dot;
        result.q_dot(7) = cal_theta_dot_to_Rm_dot(data.state_theta_b, theta_b_dot);
        result.q_dot(8) = beta_c_dot;
        result.q_dot(9) = cal_theta_dot_to_Rm_dot(data.state_theta_c, theta_c_dot);
        result.q_dot(10) = beta_d_dot;
        result.q_dot(11) = cal_theta_dot_to_Rm_dot(data.state_theta_d, theta_d_dot);
        
        // No need to update last theta/beta values since we now use motor velocities directly
        
        // CRITICAL: Convert motor torques to joint space torques
        // Motor torques (trq_r, trq_l) need to be converted to (torque_beta, force_Rm)
        double torque_beta_a, F_Rm_a;
        cal_motor_to_joint_torque(data.state_theta_a, data.state_trq_r_a, data.state_trq_l_a, torque_beta_a, F_Rm_a);
        
        double torque_beta_b, F_Rm_b;
        cal_motor_to_joint_torque(data.state_theta_b, data.state_trq_r_b, data.state_trq_l_b, torque_beta_b, F_Rm_b);
        
        double torque_beta_c, F_Rm_c;
        cal_motor_to_joint_torque(data.state_theta_c, data.state_trq_r_c, data.state_trq_l_c, torque_beta_c, F_Rm_c);
        
        double torque_beta_d, F_Rm_d;
        cal_motor_to_joint_torque(data.state_theta_d, data.state_trq_r_d, data.state_trq_l_d, torque_beta_d, F_Rm_d);
        
        // Joint torques in generalized coordinates
        result.tau(0) = torque_beta_a;
        result.tau(1) = F_Rm_a;
        result.tau(2) = -torque_beta_b;  // RF beta torque needs negation
        result.tau(3) = F_Rm_b;
        result.tau(4) = -torque_beta_c;  // RH beta torque needs negation
        result.tau(5) = F_Rm_c;
        result.tau(6) = torque_beta_d;
        result.tau(7) = F_Rm_d;
        
        // Compute leg inertia (fitting formula)
        result.I_c(0) = cal_theta_to_Ic(data.state_theta_a);
        result.I_c(1) = cal_theta_to_Ic(data.state_theta_b);
        result.I_c(2) = cal_theta_to_Ic(data.state_theta_c);
        result.I_c(3) = cal_theta_to_Ic(data.state_theta_d);
        
        first_call_ = false;
        
        return result;
    }
    
private:
    double dt_;
    double low_pass_alpha_;
    bool first_call_;
    
    // Last state variables for velocity calculation
    double last_sim_pos_x_, last_sim_pos_z_;
    
    // Last velocity variables for low-pass filtering
    double last_x_dot_, last_z_dot_;
    
    double calculate_velocity(double current, double last) {
        return (current - last) / dt_;
    }
    
    void cal_quaternion_to_euler(
        double x, double y, double z, double w,
        double& roll, double& pitch
    ) {
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
    
    double cal_theta_to_Rm(double theta) {
        // Polynomial evaluation: Rm = A[0]*theta^4 + A[1]*theta^3 + ... + A[4]
        const auto& A = corgi::Config::RM_COEFF;
        double result = 0.0;
        double theta_power = 1.0;
        
        // Evaluate from highest to lowest degree (Horner's method would be more efficient)
        for (int i = A.size() - 1; i >= 0; --i) {
            result += A[i] * theta_power;
            theta_power *= theta;
        }
        return result;
    }
    
    double cal_theta_dot_to_Rm_dot(double theta, double theta_dot) {
        // Derivative of polynomial: dRm/dt = (dRm/dtheta) * (dtheta/dt)
        const auto& A = corgi::Config::RM_COEFF;
        double derivative = 0.0;
        double theta_power = 1.0;
        
        // Derivative: d/dtheta[A[0]*theta^4 + ... + A[4]] = 4*A[0]*theta^3 + 3*A[1]*theta^2 + ...
        for (int i = A.size() - 2; i >= 0; --i) {
            derivative += A[i] * (A.size() - 1 - i) * theta_power;
            theta_power *= theta;
        }
        return derivative * theta_dot;
    }
    
    double cal_theta_to_Ic(double theta) {
        // Polynomial evaluation: Ic = B[0]*theta^6 + B[1]*theta^5 + ... + B[6]
        const auto& B = corgi::Config::IC_COEFF;
        double result = 0.0;
        double theta_power = 1.0;
        
        for (int i = B.size() - 1; i >= 0; --i) {
            result += B[i] * theta_power;
            theta_power *= theta;
        }
        return result;
    }
    
    void cal_motor_to_joint_torque(
        double theta, 
        double torque_right,
        double torque_left, 
        double& torque_beta,
        double& force_Rm
    ) {
        // Convert motor torques to joint space using virtual work method
        
        // J_theta = [[dRm/dtheta, 0], [0, 1]]
        const auto& A = corgi::Config::RM_COEFF;
        
        // Compute dRm/dtheta using polynomial derivative
        double dRm_dtheta = 0.0;
        double theta_power = 1.0;
        for (int i = A.size() - 2; i >= 0; --i) {
            dRm_dtheta += A[i] * (A.size() - 1 - i) * theta_power;
            theta_power *= theta;
        }
        
        // J_eta = [[1/2, -1/2], 
        //          [1/2, 1/2]]
        // J_theta_inv = [[1/dRm_dtheta, 0], 
        //                [           0, 1]]
        // J_eta_inv = [[1,  1], 
        //              [-1, 1]]
        
        // Virtual work: force = (J_theta_inv)^T @ (J_eta_inv)^T @ [tau_R, tau_L]^T
        
        // (J_eta_inv)^T @ [tau_R, tau_L]^T = [[1, -1], [1, 1]] @ [tau_R, tau_L]^T
        double temp1 = torque_left - torque_right;  // tau_L - tau_R
        double temp2 = torque_left + torque_right;  // tau_L + tau_R
        
        // (J_theta_inv)^T @ [temp1, temp2]^T = [[1/dRm_dtheta, 0], [0, 1]] @ [temp1, temp2]^T
        if (std::abs(dRm_dtheta) < 1e-9) {
            // Singular configuration
            force_Rm = 0.0;
            torque_beta = 0.0;
            return;
        }
        
        force_Rm = temp1 / dRm_dtheta;
        torque_beta = temp2;
    }
};

int main(int argc, char** argv) {
    try {
        (void)argc;
        (void)argv;
        // Configuration from Config.hpp
        // Build CSV path relative to this source file: package_root/data/<filename>.csv
        const auto csv_file_path = (std::filesystem::path(__FILE__).parent_path().parent_path() / "data" /
                       (std::string(corgi::Config::CSV_FILENAME) + ".csv")).string();
        const double dt = corgi::Config::DT;
        const double cutoff_freq = corgi::Config::OBSERVER_CUTOFF_FREQ;
        const int start_index = corgi::Config::START_INDEX;
        
        std::cout << "Loading data...\n";
        CSVReader reader;
        auto data = reader.read_csv(csv_file_path);
        std::cout << "✓ Loaded " << data.size() << " records\n";
        
        // Initialize observer
        std::cout << "\nInitializing observer...\n";
        corgi::DisturbanceObserver observer(
            dt, 
            cutoff_freq,
            corgi::Config::DOF,
            corgi::Config::ENABLE_LOGGING,
            corgi::Config::CSV_FILENAME,
            corgi::Config::LOG_DETAILS
        );
        
        // Initialize data processor
        DataProcessor processor(dt);
        
        // ============================================================
        // ES-EKF setup
        // ============================================================
        auto createLeg = [](double x_sign, double y_sign) -> Leg {
            return Leg{
                Eigen::Vector3f(x_sign * corgi::Config::LEG_X_OFFSET,
                                y_sign * corgi::Config::LEG_Y_OFFSET,
                                corgi::Config::LEG_Z_OFFSET),
                static_cast<float>(corgi::Config::WHEEL_RADIUS),
                static_cast<float>(corgi::Config::TIRE_SKIN_RADIUS)
            };
        };

        Leg lf_leg = createLeg( 1,  1);
        Leg rf_leg = createLeg( 1, -1);
        Leg rh_leg = createLeg(-1, -1);
        Leg lh_leg = createLeg(-1,  1);
        Leg* legs[4] = {&lf_leg, &rf_leg, &rh_leg, &lh_leg};

        estimation_model::ContactMap contact_map;
        estimation_model::ESEKF esekf(static_cast<float>(dt));

        // Override noise parameters for offline testing
        {
            estimation_model::NoiseParams np;
            // Noise parameters tuned for offline testing
            np.sigma_a  = {0.1f, 0.1f, 0.1f};        // moderate: balance IMU trust vs walking dynamics
            np.sigma_w  = {0.001f, 0.01f, 0.001f};   // tight x/z: roll/yaw unobservable from legs
            np.sigma_ba = {3.924e-4f, 3.924e-4f, 3.924e-4f};
            np.sigma_bw = {1e-8f, 1e-5f, 1e-8f};     // freeze bw_x/bw_z: unobservable
            np.sigma_bv = {1e-6f, 1e-6f, 1e-6f};
            np.sigma_leg = 1e-3f;
            esekf.set_noise_params(np);
            std::cout << "Noise params: sigma_leg=" << np.sigma_leg << "\n";
        }

        std::array<bool, 4> leg_contact_state = {false, false, false, false};
        std::array<float, 4> contact_beta = {0.f, 0.f, 0.f, 0.f};
        bool esekf_initialized = false;

        // Output CSV for ESEKF results
        const auto output_dir = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "output_data";
        std::filesystem::create_directories(output_dir);
        const auto esekf_out_path = output_dir / (std::string(corgi::Config::CSV_FILENAME) + "_esekf.csv");
        std::ofstream esekf_out(esekf_out_path);
        esekf_out << "Index,sim_pos_x,sim_pos_y,sim_pos_z,"
                  << "est_pos_x,est_pos_y,est_pos_z,"
                  << "est_vel_x,est_vel_y,est_vel_z,"
                  << "contact_a,contact_b,contact_c,contact_d,"
                  << "z_avg_x,z_avg_y,z_avg_z,"
                  << "ba_x,ba_y,ba_z,bw_x,bw_y,bw_z,bv_x,bv_y,bv_z,"
                  << "est_qw,est_qx,est_qy,est_qz,"
                  << "gt_qw,gt_qx,gt_qy,gt_qz\n";
        esekf_out << std::fixed << std::setprecision(8);

        // Process data
        std::cout << "\nStarting data processing...\n";
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Velocity RMSE accumulators
        double vel_sq_err_sum_x = 0.0, vel_sq_err_sum_y = 0.0, vel_sq_err_sum_z = 0.0;
        double pos_sq_err_sum_x = 0.0, pos_sq_err_sum_y = 0.0, pos_sq_err_sum_z = 0.0;
        double last_gt_px = data[start_index].sim_pos_x;
        double last_gt_py = data[start_index].sim_pos_y;
        double last_gt_pz = data[start_index].sim_pos_z;
        // GT initial position offset (ESEKF starts at origin)
        const double gt_offset_x = data[start_index].sim_pos_x;
        const double gt_offset_y = data[start_index].sim_pos_y;
        const double gt_offset_z = data[start_index].sim_pos_z;
        // Low-pass filtered GT velocity (10 Hz cutoff)
        const double gt_vel_alpha = 1.0 - std::exp(-2.0 * M_PI * 10.0 * dt);
        double gt_vx_filt = 0.0, gt_vy_filt = 0.0, gt_vz_filt = 0.0;
        const size_t rmse_skip = 2000;  // skip 2s warmup for GT filter + ESEKF settling
        size_t vel_count = 0;

        size_t processed_count = 0;
        for (size_t i = start_index; i < data.size(); ++i) {
            if (i % 100 == 0) {
                std::cout << "Processing index: " << i << " / " << data.size() << "\r" << std::flush;
            }
            
            // Process raw data
            auto processed = processor.process_record_data(data[i]);
            
            // Estimate disturbance
            auto disturbance = observer.estimate_disturbance(
                processed.q,
                processed.q_dot,
                processed.tau,
                processed.I_c,
                i,
                false  // Don't print detailed info
            );
            
            // ============================================================
            // Schmitt trigger contact detection (same as online)
            // ============================================================
            {
                constexpr int rm_idx[4]   = {5, 7, 9, 11};
                constexpr int beta_idx[4] = {4, 6, 8, 10};
                for (int j = 0; j < 4; ++j) {
                    double rm        = disturbance(rm_idx[j]);
                    double beta_dist = disturbance(beta_idx[j]);
                    if (!leg_contact_state[j]) {
                        if (std::abs(rm)   > corgi::Config::CONTACT_RM_THRESHOLD_HIGH ||
                            std::abs(beta_dist) > corgi::Config::CONTACT_BETA_THRESHOLD_HIGH)
                            leg_contact_state[j] = true;
                    } else {
                        if (std::abs(rm)   < corgi::Config::CONTACT_RM_THRESHOLD_LOW &&
                            std::abs(beta_dist) < corgi::Config::CONTACT_BETA_THRESHOLD_LOW)
                            leg_contact_state[j] = false;
                    }
                }
            }

            // ============================================================
            // ES-EKF pipeline
            // ============================================================

            // --- Initialize on first iteration ---
            if (!esekf_initialized) {
                estimation_model::NominalState x0;
                x0.q = Eigen::Quaternionf(
                    static_cast<float>(data[i].imu_orien_w),
                    static_cast<float>(data[i].imu_orien_x),
                    static_cast<float>(data[i].imu_orien_y),
                    static_cast<float>(data[i].imu_orien_z)).normalized();
                esekf.init(x0);
                contact_beta.fill(0.f);
                esekf_initialized = true;
                std::cout << "\nES-EKF initialized at index " << i << "\n";
            }

            // --- IMU measurements ---
            Eigen::Vector3f a_m(
                static_cast<float>(data[i].imu_lin_acc_x),
                static_cast<float>(data[i].imu_lin_acc_y),
                static_cast<float>(data[i].imu_lin_acc_z));
            Eigen::Vector3f w_m(
                static_cast<float>(data[i].imu_ang_vel_x),
                static_cast<float>(data[i].imu_ang_vel_y),
                static_cast<float>(data[i].imu_ang_vel_z));

            // --- Predict ---
            esekf.predict(a_m, w_m);

            // --- Build per-leg observations ---
            const float w_y = static_cast<float>(data[i].imu_ang_vel_y)
                            - esekf.nominal().bw.y();

            double thetas[4] = {data[i].state_theta_a, data[i].state_theta_b,
                                data[i].state_theta_c, data[i].state_theta_d};
            double betas[4]  = {data[i].state_beta_a,  data[i].state_beta_b,
                                data[i].state_beta_c,  data[i].state_beta_d};
            double vel_r[4]  = {data[i].state_vel_r_a, data[i].state_vel_r_b,
                                data[i].state_vel_r_c, data[i].state_vel_r_d};
            double vel_l[4]  = {data[i].state_vel_l_a, data[i].state_vel_l_b,
                                data[i].state_vel_l_c, data[i].state_vel_l_d};

            std::vector<estimation_model::LegObservation> observations;
            observations.reserve(4);
            std::array<bool, 4> exclude_flags{};

            for (int j = 0; j < 4; ++j) {
                const float fdt = static_cast<float>(dt);
                bool is_right_side = (j == 1 || j == 2);

                float theta   = static_cast<float>(thetas[j]);
                float beta    = is_right_side
                    ? -static_cast<float>(betas[j])
                    :  static_cast<float>(betas[j]);
                float theta_d = static_cast<float>((-vel_r[j] + vel_l[j]) / 2.0);
                float beta_d  = -static_cast<float>(( vel_r[j] + vel_l[j]) / 2.0);
                if (is_right_side) beta_d = -beta_d;

                // --- Use beta directly for rim lookup (matching Python approach) ---
                // The Python tune_schmitt_trigger.py uses contact_beta = beta, alpha = 0
                // and achieves RMSE ~4.5mm/s. The accumulated contact_beta diverges
                // because it starts from 0 while beta is already non-zero at start_index.
                RIM rim = contact_map.lookup(theta, beta);
                float alpha = 0.0f;

                // Contact flag: Schmitt trigger AND rim in contact
                bool in_contact = leg_contact_state[j] && (rim != NO_CONTACT);

                observations.push_back(
                    {legs[j], theta, theta_d, beta, beta_d, rim, alpha, in_contact});
                exclude_flags[j] = !in_contact;
            }

            // --- Compute average z_leg for diagnostics (RAW, no ESEKF bias) ---
            Eigen::Vector3f z_avg = Eigen::Vector3f::Zero();
            Eigen::Vector3f z_avg_noW = Eigen::Vector3f::Zero();  // without w×r term
            int n_contact = 0;
            {
                // Use raw w_m with only w_y (matching observation model)
                Eigen::Vector3f w_raw(0.0f,
                    static_cast<float>(data[i].imu_ang_vel_y),
                    0.0f);
                Eigen::Vector3f w_zero = Eigen::Vector3f::Zero();
                for (int j = 0; j < 4; ++j) {
                    if (!exclude_flags[j]) {
                        auto& o = observations[j];
                        o.leg->Calculate(o.theta, o.theta_d, 0, o.beta, o.beta_d, 0);
                        o.leg->PointContact(o.rim, o.alpha);
                        Eigen::Vector3f v_zero = Eigen::Vector3f::Zero();
                        
                        // With angular velocity
                        o.leg->PointVelocity(v_zero, w_raw, o.rim, o.alpha, true);
                        z_avg += -o.leg->contact_velocity;
                        
                        // Without angular velocity (pure kinematic)  
                        o.leg->PointVelocity(v_zero, w_zero, o.rim, o.alpha, true);
                        z_avg_noW += -o.leg->contact_velocity;
                        
                        n_contact++;
                        
                        // Debug first step: per-leg details
                        if (i == static_cast<size_t>(start_index)) {
                            const char* leg_names[] = {"LF", "RF", "RH", "LH"};
                            Eigen::Vector3f cp = o.leg->contact_point;
                            o.leg->PointVelocity(v_zero, w_raw, o.rim, o.alpha, true);
                            Eigen::Vector3f z_w = -o.leg->contact_velocity;
                            o.leg->PointVelocity(v_zero, w_zero, o.rim, o.alpha, true);
                            Eigen::Vector3f z_nw = -o.leg->contact_velocity;
                            std::cout << "[Step " << i << "] Leg " << leg_names[j]
                                      << ": theta=" << o.theta << " beta=" << o.beta
                                      << " theta_d=" << o.theta_d << " beta_d=" << o.beta_d
                                      << "\n    contact_pt=(" << cp.x() << ", " << cp.y() << ", " << cp.z() << ")"
                                      << " rim=" << o.rim << " alpha=" << o.alpha
                                      << "\n    z_leg(w)=(" << z_w.x() << ", " << z_w.y() << ", " << z_w.z() << ")"
                                      << "\n    z_leg(0)=(" << z_nw.x() << ", " << z_nw.y() << ", " << z_nw.z() << ")"
                                      << "\n    w×cp=(" << (z_w.x()-z_nw.x()) << ", " << (z_w.y()-z_nw.y()) << ", " << (z_w.z()-z_nw.z()) << ")"
                                      << "\n";
                        }
                    }
                }
                if (n_contact > 0) {
                    z_avg /= static_cast<float>(n_contact);
                    z_avg_noW /= static_cast<float>(n_contact);
                }
            }

            // --- Update ---
            esekf.update_all_legs(observations, w_m, exclude_flags);

            // --- Inject & reset ---
            esekf.inject_and_reset();

            // --- Accumulate position & velocity RMSE ---
            {
                const auto& st2 = esekf.nominal();
                // Position error (world frame, offset-corrected)
                double ep_x = st2.p.x() - (data[i].sim_pos_x - gt_offset_x);
                double ep_y = st2.p.y() - (data[i].sim_pos_y - gt_offset_y);
                double ep_z = st2.p.z() - (data[i].sim_pos_z - gt_offset_z);

                // GT velocity (world frame, finite difference + LPF)
                double gt_vx_raw = (data[i].sim_pos_x - last_gt_px) / dt;
                double gt_vy_raw = (data[i].sim_pos_y - last_gt_py) / dt;
                double gt_vz_raw = (data[i].sim_pos_z - last_gt_pz) / dt;
                last_gt_px = data[i].sim_pos_x;
                last_gt_py = data[i].sim_pos_y;
                last_gt_pz = data[i].sim_pos_z;

                // 10 Hz low-pass filter on GT velocity (raw diff at 1kHz is too noisy)
                gt_vx_filt = (1.0 - gt_vel_alpha) * gt_vx_filt + gt_vel_alpha * gt_vx_raw;
                gt_vy_filt = (1.0 - gt_vel_alpha) * gt_vy_filt + gt_vel_alpha * gt_vy_raw;
                gt_vz_filt = (1.0 - gt_vel_alpha) * gt_vz_filt + gt_vel_alpha * gt_vz_raw;

                // Estimated velocity in world frame: v_world = R * v_body
                Eigen::Matrix3f R_est = st2.q.toRotationMatrix();
                Eigen::Vector3f v_world = R_est * st2.v;

                // Accumulate error only after warmup
                if (processed_count >= rmse_skip) {
                    double ev_x = v_world.x() - gt_vx_filt;
                    double ev_y = v_world.y() - gt_vy_filt;
                    double ev_z = v_world.z() - gt_vz_filt;
                    vel_sq_err_sum_x += ev_x * ev_x;
                    vel_sq_err_sum_y += ev_y * ev_y;
                    vel_sq_err_sum_z += ev_z * ev_z;

                    pos_sq_err_sum_x += ep_x * ep_x;
                    pos_sq_err_sum_y += ep_y * ep_y;
                    pos_sq_err_sum_z += ep_z * ep_z;

                    vel_count++;
                }
            }

            // --- Log ---
            const auto& st = esekf.nominal();
            esekf_out << i << ","
                      << data[i].sim_pos_x << "," << data[i].sim_pos_y << "," << data[i].sim_pos_z << ","
                      << st.p.x() << "," << st.p.y() << "," << st.p.z() << ","
                      << st.v.x() << "," << st.v.y() << "," << st.v.z() << ","
                      << leg_contact_state[0] << "," << leg_contact_state[1] << ","
                      << leg_contact_state[2] << "," << leg_contact_state[3] << ","
                      << z_avg.x() << "," << z_avg.y() << "," << z_avg.z() << ","
                      << st.ba.x() << "," << st.ba.y() << "," << st.ba.z() << ","
                      << st.bw.x() << "," << st.bw.y() << "," << st.bw.z() << ","
                      << st.bv.x() << "," << st.bv.y() << "," << st.bv.z() << ","
                      << st.q.w() << "," << st.q.x() << "," << st.q.y() << "," << st.q.z() << ","
                      << data[i].imu_orien_w << "," << data[i].imu_orien_x << ","
                      << data[i].imu_orien_y << "," << data[i].imu_orien_z << "\n";

            processed_count++;
        }
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        
        esekf_out.close();

        std::cout << "\nProcessing complete\n";
        std::cout << "Disturbance results: output_data/" << corgi::Config::CSV_FILENAME << "_result.csv\n";
        std::cout << "ESEKF results:       " << esekf_out_path.string() << "\n";
        
        // Display performance statistics
        std::cout << "\n========== Performance Statistics ==========\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Processed records: " << processed_count << "\n";
        std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
        std::cout << "Average time per record: " << (elapsed.count() / processed_count) << " seconds\n";
        std::cout << "Processing speed: " << (processed_count / elapsed.count()) << " records/second\n";
        std::cout << "==========================================\n";

        // Final ESEKF state summary
        const auto& final_st = esekf.nominal();
        std::cout << "\n========== ESEKF Final State ==========\n";
        std::cout << "Position: [" << final_st.p.x() << ", " << final_st.p.y() << ", " << final_st.p.z() << "]\n";
        std::cout << "Velocity: [" << final_st.v.x() << ", " << final_st.v.y() << ", " << final_st.v.z() << "]\n";
        std::cout << "Bias_a:   [" << final_st.ba.x() << ", " << final_st.ba.y() << ", " << final_st.ba.z() << "]\n";
        std::cout << "Bias_w:   [" << final_st.bw.x() << ", " << final_st.bw.y() << ", " << final_st.bw.z() << "]\n";
        std::cout << "Bias_v:   [" << final_st.bv.x() << ", " << final_st.bv.y() << ", " << final_st.bv.z() << "]\n";
        std::cout << "GT (off): [" << data.back().sim_pos_x - gt_offset_x << ", " << data.back().sim_pos_y - gt_offset_y << ", " << data.back().sim_pos_z - gt_offset_z << "]\n";
        std::cout << "==========================================\n";

        // Position & Velocity RMSE
        if (vel_count > 0) {
            double n = static_cast<double>(vel_count);
            std::cout << "\n========== RMSE Statistics ==========\n";
            std::cout << "Position RMSE (m):  ["
                      << std::sqrt(pos_sq_err_sum_x / n) << ", "
                      << std::sqrt(pos_sq_err_sum_y / n) << ", "
                      << std::sqrt(pos_sq_err_sum_z / n) << "]\n";
            std::cout << "Velocity RMSE (m/s):["
                      << std::sqrt(vel_sq_err_sum_x / n) << ", "
                      << std::sqrt(vel_sq_err_sum_y / n) << ", "
                      << std::sqrt(vel_sq_err_sum_z / n) << "]\n";
            double pos_rmse_total = std::sqrt((pos_sq_err_sum_x + pos_sq_err_sum_y + pos_sq_err_sum_z) / n);
            double vel_rmse_total = std::sqrt((vel_sq_err_sum_x + vel_sq_err_sum_y + vel_sq_err_sum_z) / n);
            std::cout << "Position RMSE total: " << pos_rmse_total << " m\n";
            std::cout << "Velocity RMSE total: " << vel_rmse_total << " m/s\n";
            std::cout << "====================================\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}