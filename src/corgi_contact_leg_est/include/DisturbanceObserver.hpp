#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include "quadruped_dynamics.hpp"

namespace quadruped {

/**
 * @brief Disturbance Observer for quadruped robot
 * 
 * Implements discrete-time disturbance observer:
 * τ̂_d = β*p_k - LPF(β*p + S^T*τ + C^T*q̇ - g)
 * 
 * where LPF is a discrete-time low-pass filter.
 */
class DisturbanceObserver {
public:
    /**
     * @brief Constructor
     * 
     * @param dt Sampling time (s)
     * @param cutoff_freq Low-pass filter cutoff frequency (Hz)
     * @param dof Degrees of freedom (default: 12)
     * @param logging Enable logging to CSV file
     * @param output_filename Base filename for output CSV (without path and extension)
     */
    DisturbanceObserver(
        double dt,
        double cutoff_freq,
        int dof = 12,
        bool logging = false,
        const std::string& output_filename = "cpp_test"
    );
    
    /**
     * @brief Estimate external disturbance torques
     * 
     * @param q Generalized coordinates (dof,)
     * @param q_dot Generalized velocities (dof,)
     * @param tau Control torque commands (8,) - actuated joints only
     * @param I_c Leg inertias [I_lf, I_rf, I_rh, I_lh] (4,)
     * @param index Current iteration index (for logging)
     * @param details Print detailed information
     * @return Estimated external disturbance torques (dof,)
     */
    Eigen::VectorXd estimate_disturbance(
        const Eigen::Ref<const Eigen::VectorXd>& q,
        const Eigen::Ref<const Eigen::VectorXd>& q_dot,
        const Eigen::Ref<const Eigen::VectorXd>& tau,
        const Eigen::Ref<const Eigen::VectorXd>& I_c,
        int index = 0,
        bool details = false
    );
    
    /**
     * @brief Get current disturbance estimate
     * 
     * @return Current estimated disturbance (dof,)
     */
    Eigen::VectorXd get_disturbance() const {
        return estimated_disturbance_;
    }
    
private:
    // Observer parameters
    double dt_;                          // Sampling time
    double cutoff_freq_;                 // Cutoff frequency
    int dof_;                            // Degrees of freedom
    bool logging_;                       // Enable logging
    std::string output_filename_;        // Output CSV filename base
    
    // Filter parameters
    double gamma_;                       // Z-domain pole
    double beta_;                        // Feedforward gain
    
    // Observer state
    bool initialized_;                   // Initialization flag
    Eigen::VectorXd estimated_disturbance_;  // Estimated disturbance
    Eigen::VectorXd Y_filtered_prev_;    // Previous filtered value
    
    // Selection matrix S^T (dof x actuated_dof)
    // Maps actuated joint torques to generalized force space
    Eigen::MatrixXd S_T_;
    
    // Temporary matrices for dynamics computation
    Eigen::MatrixXd M_;                  // Mass matrix
    Eigen::MatrixXd C_;                  // Coriolis matrix
    Eigen::VectorXd G_;                  // Gravity vector
    Eigen::MatrixXd D_;                  // Inertia rate matrix (not used in observer)
    
    // Logging
    std::ofstream log_file_;
    std::vector<std::string> column_names_;
    
    /**
     * @brief Validate initialization parameters
     */
    void validate_params(double dt, double cutoff_freq);
    
    /**
     * @brief Compute discrete-time filter parameters
     * 
     * Based on paper formulas:
     * - γ = e^(-λΔt)  (Z-domain pole)
     * - β = (1-γ)/(γ*Δt)  (feedforward gain)
     */
    void compute_filter_params();
    
    /**
     * @brief Print initialization information
     */
    void print_init_info();
    
    /**
     * @brief Initialize observer internal states
     */
    void initialize_observer_states();
    
    /**
     * @brief Initialize selection matrix S^T
     * 
     * S^T maps 8 actuated joint torques to 12-DOF generalized forces
     * Structure:
     * - Base (x, z, roll, pitch): unactuated -> zeros
     * - Legs (beta, Rm) x 4: actuated -> identity
     */
    void initialize_selection_matrix();
    
    /**
     * @brief Log details to CSV file
     */
    void log_details_to_file(
        int index,
        const Eigen::VectorXd& p_k,
        const Eigen::VectorXd& Y_k,
        const Eigen::VectorXd& Y_filtered_k,
        const Eigen::VectorXd& S_T_tau,
        const Eigen::VectorXd& C_q_dot,
        const Eigen::VectorXd& g,
        const Eigen::VectorXd& estimated_disturbance
    );
    
    /**
     * @brief Initialize logging system
     */
    void initialize_logging();
};

} // namespace quadruped
