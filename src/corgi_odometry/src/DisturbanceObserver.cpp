#include "DisturbanceObserver.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace quadruped {

DisturbanceObserver::DisturbanceObserver(
    double dt,
    double cutoff_freq,
    int dof,
    bool logging,
    const std::string& output_filename
) : dt_(dt), 
    cutoff_freq_(cutoff_freq), 
    dof_(dof),
    logging_(logging),
    output_filename_(output_filename),
    initialized_(false) 
{
    validate_params(dt, cutoff_freq);
    compute_filter_params();
    initialize_observer_states();
    initialize_selection_matrix();
    
    if (logging_) {
        initialize_logging();
    }
    
    print_init_info();
}

void DisturbanceObserver::validate_params(double dt, double cutoff_freq) {
    if (dt <= 0) {
        throw std::invalid_argument("採樣時間 dt 必須大於 0");
    }
    if (cutoff_freq <= 0) {
        throw std::invalid_argument("截止頻率必須大於 0");
    }
    
    // Check Nyquist condition
    double nyquist_freq = 1.0 / (2.0 * dt);
    if (cutoff_freq >= nyquist_freq) {
        std::cerr << "Warning: 截止頻率 (" << cutoff_freq << " Hz) 接近 nyquist_freq (" 
                  << nyquist_freq << " Hz)" << std::endl;
    }
}

void DisturbanceObserver::compute_filter_params() {
    // Convert cutoff frequency to rad/s
    double omega_c = 2.0 * M_PI * cutoff_freq_;
    
    // Z-domain pole (γ in paper)
    gamma_ = std::exp(-omega_c * dt_);
    
    // Feedforward gain (β in paper)
    beta_ = (1.0 - gamma_) / (gamma_ * dt_);
}

void DisturbanceObserver::print_init_info() {
    std::cout << "============================================================\n";
    std::cout << "Discrete-Time Disturbance Observer\n";
    std::cout << "============================================================\n";
    std::cout << "Sampling Time: " << (dt_ * 1000.0) << " ms\n";
    std::cout << "Cutoff frequency: " << cutoff_freq_ << " Hz\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "γ (Z-domain Pole): " << gamma_ << "\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "β: " << beta_ << "\n";
    std::cout << "============================================================\n";
}

void DisturbanceObserver::initialize_observer_states() {
    estimated_disturbance_ = Eigen::VectorXd::Zero(dof_);
    Y_filtered_prev_ = Eigen::VectorXd::Zero(dof_);
    
    // Initialize temporary matrices
    M_ = Eigen::MatrixXd::Zero(dof_, dof_);
    C_ = Eigen::MatrixXd::Zero(dof_, dof_);
    G_ = Eigen::VectorXd::Zero(dof_);
    D_ = Eigen::MatrixXd::Zero(dof_, dof_);
    
    initialized_ = true;
}

void DisturbanceObserver::initialize_selection_matrix() {
    // S^T is (dof x actuated_dof) = (12 x 8)
    // Maps 8 actuated joint torques to 12-DOF generalized forces
    
    S_T_ = Eigen::MatrixXd::Zero(dof_, 8);
    
    // Base DOFs (0-3: x, z, roll, pitch) are unactuated -> zeros
    // Leg DOFs (4-11: beta_lf, Rm_lf, beta_rf, Rm_rf, beta_rh, Rm_rh, beta_lh, Rm_lh)
    // are actuated -> identity mapping
    
    for (int i = 0; i < 8; ++i) {
        S_T_(i + 4, i) = 1.0;  // Start from index 4 (skip base DOFs)
    }
}

void DisturbanceObserver::initialize_logging() {
    // Define DOF suffixes
    std::vector<std::string> dof_suffixes = {
        "x", "z", "roll", "pitch",
        "beta_a", "rm_a", "beta_b", "rm_b",
        "beta_c", "rm_c", "beta_d", "rm_d"
    };
    
    column_names_.push_back("Index");
    
    // Add columns for each state variable with all DOF suffixes
    std::vector<std::string> state_names = {
        "p_k", "Y_k", "Y_filtered_k", "S_T_tau", 
        "C_q_dot", "g", "estimated_disturbance"
    };
    
    for (const auto& state_name : state_names) {
        for (const auto& suffix : dof_suffixes) {
            column_names_.push_back(state_name + "_" + suffix);
        }
    }
    
    std::cout << "✓ Logging enabled\n";
}

Eigen::VectorXd DisturbanceObserver::estimate_disturbance(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& q_dot,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    int index,
    bool details
) {
    // Validate input dimensions
    if (q.size() != dof_ || q_dot.size() != dof_) {
        throw std::invalid_argument("輸入維度不一致");
    }
    if (tau.size() != 8) {
        throw std::invalid_argument("tau 維度必須為 8 (actuated DOFs)");
    }
    if (I_c.size() != 4) {
        throw std::invalid_argument("I_c 維度必須為 4 (4 legs)");
    }
    
    // Compute robot dynamics
    quadruped_dynamics::compute_dynamics(q, q_dot, I_c, M_, C_, G_, D_);
    
    // === Discrete-time Disturbance Observer Core Algorithm ===
    
    // Step 1: Compute generalized momentum p_k = M(q_k) * q̇_k
    Eigen::VectorXd p_k = M_ * q_dot;
    
    // Step 2: Compute filter input term (term inside brackets in paper formula)
    // Y_k = β*p + S^T*τ + C^T*q̇ - g
    Eigen::VectorXd S_T_tau = S_T_ * tau;
    Eigen::VectorXd C_q_dot = C_.transpose() * q_dot;
    Eigen::VectorXd Y_k = beta_ * p_k + S_T_tau + C_q_dot - G_;
    
    // Step 3: Apply discrete-time low-pass filter
    // Y_filtered_k = (1-γ)*Y_k + γ*Y_filtered_{k-1}
    Eigen::VectorXd Y_filtered_k = (1.0 - gamma_) * Y_k + gamma_ * Y_filtered_prev_;
    
    // Step 4: Compute estimated disturbance torque (paper core formula)
    // τ̂_d_k = β*p_k - Y_filtered_k
    estimated_disturbance_ = beta_ * p_k - Y_filtered_k;
    
    if (logging_) {
        log_details_to_file(index, p_k, Y_k, Y_filtered_k, S_T_tau, C_q_dot, G_, estimated_disturbance_);
    }
    
    // Update filter state for next iteration
    Y_filtered_prev_ = Y_filtered_k;
    
    return estimated_disturbance_;
}

void DisturbanceObserver::log_details_to_file(
    int index,
    const Eigen::VectorXd& p_k,
    const Eigen::VectorXd& Y_k,
    const Eigen::VectorXd& Y_filtered_k,
    const Eigen::VectorXd& S_T_tau,
    const Eigen::VectorXd& C_q_dot,
    const Eigen::VectorXd& g,
    const Eigen::VectorXd& estimated_disturbance
) {
    // Open file if not already open
    if (!log_file_.is_open()) {
        std::string filename = "output_data/" + output_filename_ + "_result.csv";
        log_file_.open(filename);
        
        // Write header
        for (size_t i = 0; i < column_names_.size(); ++i) {
            log_file_ << column_names_[i];
            if (i < column_names_.size() - 1) log_file_ << ",";
        }
        log_file_ << "\n";
    }
    
    // Write data row
    log_file_ << index;
    
    std::vector<const Eigen::VectorXd*> arrays = {
        &p_k, &Y_k, &Y_filtered_k, &S_T_tau, &C_q_dot, &g, &estimated_disturbance
    };
    
    for (const auto* arr : arrays) {
        for (int i = 0; i < arr->size(); ++i) {
            log_file_ << "," << std::setprecision(10) << (*arr)(i);
        }
    }
    log_file_ << "\n";
    log_file_.flush();
}

} // namespace quadruped
