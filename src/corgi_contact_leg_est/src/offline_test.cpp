#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include <map>
#include <chrono>
#include <iomanip>
#include "DisturbanceObserver.hpp"
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
        
        // Leg data (LF, RF, RH, LH)
        double state_theta_a, state_beta_a, state_trq_r_a, state_trq_l_a;
        double state_theta_b, state_beta_b, state_trq_r_b, state_trq_l_b;
        double state_theta_c, state_beta_c, state_trq_r_c, state_trq_l_c;
        double state_theta_d, state_beta_d, state_trq_r_d, state_trq_l_d;
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
            
            row.state_theta_a = values[column_indices.at("state_theta_a")];
            row.state_beta_a = values[column_indices.at("state_beta_a")];
            row.state_trq_r_a = values[column_indices.at("state_trq_r_a")];
            row.state_trq_l_a = values[column_indices.at("state_trq_l_a")];
            
            row.state_theta_b = values[column_indices.at("state_theta_b")];
            row.state_beta_b = values[column_indices.at("state_beta_b")];
            row.state_trq_r_b = values[column_indices.at("state_trq_r_b")];
            row.state_trq_l_b = values[column_indices.at("state_trq_l_b")];
            
            row.state_theta_c = values[column_indices.at("state_theta_c")];
            row.state_beta_c = values[column_indices.at("state_beta_c")];
            row.state_trq_r_c = values[column_indices.at("state_trq_r_c")];
            row.state_trq_l_c = values[column_indices.at("state_trq_l_c")];
            
            row.state_theta_d = values[column_indices.at("state_theta_d")];
            row.state_beta_d = values[column_indices.at("state_beta_d")];
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
        low_pass_alpha_ = 1.0 - std::exp(-2.0 * M_PI * quadruped::Config::ENCODER_CUTOFF_FREQ * dt_);
        
        // Initialize last state variables
        last_sim_pos_x_ = 0.0;
        last_sim_pos_z_ = 0.0;
        last_theta_a_ = 0.0;
        last_theta_b_ = 0.0;
        last_theta_c_ = 0.0;
        last_theta_d_ = 0.0;
        last_beta_a_ = 0.0;
        last_beta_b_ = 0.0;
        last_beta_c_ = 0.0;
        last_beta_d_ = 0.0;
        
        // Initialize last velocity variables
        last_x_dot_ = 0.0;
        last_z_dot_ = 0.0;
        last_theta_a_dot_ = 0.0;
        last_theta_b_dot_ = 0.0;
        last_theta_c_dot_ = 0.0;
        last_theta_d_dot_ = 0.0;
        last_beta_a_dot_ = 0.0;
        last_beta_b_dot_ = 0.0;
        last_beta_c_dot_ = 0.0;
        last_beta_d_dot_ = 0.0;
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
        quaternion_to_euler(
            data.imu_orien_x, data.imu_orien_y, 
            data.imu_orien_z, data.imu_orien_w,
            roll, pitch
        );
        result.q(2) = roll;
        result.q(3) = pitch;
        
        // Leg joint angles - CRITICAL: Use Rm not theta!
        result.q(4) = data.state_beta_a;
        result.q(5) = theta_to_Rm(data.state_theta_a);  // Convert theta to Rm
        result.q(6) = -data.state_beta_b;  // RF needs negation
        result.q(7) = theta_to_Rm(data.state_theta_b);
        result.q(8) = -data.state_beta_c;  // RH needs negation
        result.q(9) = theta_to_Rm(data.state_theta_c);
        result.q(10) = data.state_beta_d;
        result.q(11) = theta_to_Rm(data.state_theta_d);
        
        // Base angular velocity (from IMU)
        result.q_dot(2) = data.imu_ang_vel_x;  // roll rate
        result.q_dot(3) = data.imu_ang_vel_y;  // pitch rate
        
        // Leg joint velocities with low-pass filtering
        double theta_a_dot = calculate_velocity(data.state_theta_a, last_theta_a_);
        double beta_a_dot = calculate_velocity(data.state_beta_a, last_beta_a_);
        double theta_b_dot = calculate_velocity(data.state_theta_b, last_theta_b_);
        double beta_b_dot = calculate_velocity(-data.state_beta_b, last_beta_b_);  // Note: negated
        double theta_c_dot = calculate_velocity(data.state_theta_c, last_theta_c_);
        double beta_c_dot = calculate_velocity(-data.state_beta_c, last_beta_c_);  // Note: negated
        double theta_d_dot = calculate_velocity(data.state_theta_d, last_theta_d_);
        double beta_d_dot = calculate_velocity(data.state_beta_d, last_beta_d_);
        
        if (!first_call_) {
            theta_a_dot = (1.0 - low_pass_alpha_) * last_theta_a_dot_ + low_pass_alpha_ * theta_a_dot;
            beta_a_dot = (1.0 - low_pass_alpha_) * last_beta_a_dot_ + low_pass_alpha_ * beta_a_dot;
            theta_b_dot = (1.0 - low_pass_alpha_) * last_theta_b_dot_ + low_pass_alpha_ * theta_b_dot;
            beta_b_dot = (1.0 - low_pass_alpha_) * last_beta_b_dot_ + low_pass_alpha_ * beta_b_dot;
            theta_c_dot = (1.0 - low_pass_alpha_) * last_theta_c_dot_ + low_pass_alpha_ * theta_c_dot;
            beta_c_dot = (1.0 - low_pass_alpha_) * last_beta_c_dot_ + low_pass_alpha_ * beta_c_dot;
            theta_d_dot = (1.0 - low_pass_alpha_) * last_theta_d_dot_ + low_pass_alpha_ * theta_d_dot;
            beta_d_dot = (1.0 - low_pass_alpha_) * last_beta_d_dot_ + low_pass_alpha_ * beta_d_dot;
        }
        
        // Convert theta_dot to Rm_dot
        result.q_dot(4) = beta_a_dot;
        result.q_dot(5) = theta_dot_to_Rm_dot(data.state_theta_a, theta_a_dot);
        result.q_dot(6) = beta_b_dot;
        result.q_dot(7) = theta_dot_to_Rm_dot(data.state_theta_b, theta_b_dot);
        result.q_dot(8) = beta_c_dot;
        result.q_dot(9) = theta_dot_to_Rm_dot(data.state_theta_c, theta_c_dot);
        result.q_dot(10) = beta_d_dot;
        result.q_dot(11) = theta_dot_to_Rm_dot(data.state_theta_d, theta_d_dot);
        
        // Update last values
        last_theta_a_ = data.state_theta_a;
        last_beta_a_ = data.state_beta_a;
        last_theta_b_ = data.state_theta_b;
        last_beta_b_ = -data.state_beta_b;
        last_theta_c_ = data.state_theta_c;
        last_beta_c_ = -data.state_beta_c;
        last_theta_d_ = data.state_theta_d;
        last_beta_d_ = data.state_beta_d;
        
        last_theta_a_dot_ = theta_a_dot;
        last_beta_a_dot_ = beta_a_dot;
        last_theta_b_dot_ = theta_b_dot;
        last_beta_b_dot_ = beta_b_dot;
        last_theta_c_dot_ = theta_c_dot;
        last_beta_c_dot_ = beta_c_dot;
        last_theta_d_dot_ = theta_d_dot;
        last_beta_d_dot_ = beta_d_dot;
        
        // CRITICAL: Convert motor torques to joint space torques
        // Motor torques (trq_r, trq_l) need to be converted to (torque_beta, force_Rm)
        double torque_beta_a, F_Rm_a;
        calculate_motor_to_joint_torque(data.state_theta_a, data.state_trq_r_a, data.state_trq_l_a, torque_beta_a, F_Rm_a);
        
        double torque_beta_b, F_Rm_b;
        calculate_motor_to_joint_torque(data.state_theta_b, data.state_trq_r_b, data.state_trq_l_b, torque_beta_b, F_Rm_b);
        
        double torque_beta_c, F_Rm_c;
        calculate_motor_to_joint_torque(data.state_theta_c, data.state_trq_r_c, data.state_trq_l_c, torque_beta_c, F_Rm_c);
        
        double torque_beta_d, F_Rm_d;
        calculate_motor_to_joint_torque(data.state_theta_d, data.state_trq_r_d, data.state_trq_l_d, torque_beta_d, F_Rm_d);
        
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
        result.I_c(0) = theta_to_Ic(data.state_theta_a);
        result.I_c(1) = theta_to_Ic(data.state_theta_b);
        result.I_c(2) = theta_to_Ic(data.state_theta_c);
        result.I_c(3) = theta_to_Ic(data.state_theta_d);
        
        first_call_ = false;
        
        return result;
    }
    
private:
    double dt_;
    double low_pass_alpha_;
    bool first_call_;
    
    // Last state variables for velocity calculation
    double last_sim_pos_x_, last_sim_pos_z_;
    double last_theta_a_, last_theta_b_, last_theta_c_, last_theta_d_;
    double last_beta_a_, last_beta_b_, last_beta_c_, last_beta_d_;
    
    // Last velocity variables for low-pass filtering
    double last_x_dot_, last_z_dot_;
    double last_theta_a_dot_, last_theta_b_dot_, last_theta_c_dot_, last_theta_d_dot_;
    double last_beta_a_dot_, last_beta_b_dot_, last_beta_c_dot_, last_beta_d_dot_;
    
    double calculate_velocity(double current, double last) {
        return (current - last) / dt_;
    }
    
    void quaternion_to_euler(
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
    
    double theta_to_Rm(double theta) {
        // Polynomial evaluation: Rm = A[0]*theta^4 + A[1]*theta^3 + ... + A[4]
        const auto& A = quadruped::Config::RM_COEFF;
        double result = 0.0;
        double theta_power = 1.0;
        
        // Evaluate from highest to lowest degree (Horner's method would be more efficient)
        for (int i = A.size() - 1; i >= 0; --i) {
            result += A[i] * theta_power;
            theta_power *= theta;
        }
        return result;
    }
    
    double theta_dot_to_Rm_dot(double theta, double theta_dot) {
        // Derivative of polynomial: dRm/dt = (dRm/dtheta) * (dtheta/dt)
        const auto& A = quadruped::Config::RM_COEFF;
        double derivative = 0.0;
        double theta_power = 1.0;
        
        // Derivative: d/dtheta[A[0]*theta^4 + ... + A[4]] = 4*A[0]*theta^3 + 3*A[1]*theta^2 + ...
        for (int i = A.size() - 2; i >= 0; --i) {
            derivative += A[i] * (A.size() - 1 - i) * theta_power;
            theta_power *= theta;
        }
        return derivative * theta_dot;
    }
    
    double theta_to_Ic(double theta) {
        // Polynomial evaluation: Ic = B[0]*theta^6 + B[1]*theta^5 + ... + B[6]
        const auto& B = quadruped::Config::IC_COEFF;
        double result = 0.0;
        double theta_power = 1.0;
        
        for (int i = B.size() - 1; i >= 0; --i) {
            result += B[i] * theta_power;
            theta_power *= theta;
        }
        return result;
    }
    
    void calculate_motor_to_joint_torque(
        double theta, 
        double torque_right,
        double torque_left, 
        double& torque_beta,
        double& force_Rm
    ) {
        // Convert motor torques to joint space using virtual work method
        
        // J_theta = [[dRm/dtheta, 0], [0, 1]]
        const auto& A = quadruped::Config::RM_COEFF;
        
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
        // Configuration from Config.hpp
        const std::string csv_file_path = std::string("data/") + quadruped::Config::CSV_FILENAME + ".csv";
        const double dt = quadruped::Config::DT;
        const double cutoff_freq = quadruped::Config::OBSERVER_CUTOFF_FREQ;
        const int start_index = quadruped::Config::START_INDEX;
        
        std::cout << "Loading data...\n";
        CSVReader reader;
        auto data = reader.read_csv(csv_file_path);
        std::cout << "✓ Loaded " << data.size() << " records\n";
        
        // Initialize observer
        std::cout << "\nInitializing observer...\n";
        quadruped::DisturbanceObserver observer(
            dt, 
            cutoff_freq,
            quadruped::Config::DOF,
            quadruped::Config::ENABLE_LOGGING,
            quadruped::Config::CSV_FILENAME
        );
        
        // Initialize data processor
        DataProcessor processor(dt);
        
        // Process data
        std::cout << "\nStarting data processing...\n";
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
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
            
            // Can use disturbance for further processing here
            processed_count++;
        }
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        
        std::cout << "\nProcessing complete\n";
        std::cout << "Results saved to output_data/" << quadruped::Config::CSV_FILENAME << "_result.csv\n";
        
        // Display performance statistics
        std::cout << "\n========== Performance Statistics ==========\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Processed records: " << processed_count << "\n";
        std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
        std::cout << "Average time per record: " << (elapsed.count() / processed_count) << " seconds\n";
        std::cout << "Processing speed: " << (processed_count / elapsed.count()) << " records/second\n";
        std::cout << "==========================================\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}