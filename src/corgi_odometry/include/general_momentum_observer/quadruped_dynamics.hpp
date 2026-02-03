#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace quadruped_dynamics {

/**
 * @brief Compute dynamics matrices for 12-DOF quadruped robot
 * 
 * State vector definition (12-dimensional):
 * q = [x, z, phi, psi, beta_lf, Rm_lf, beta_rf, Rm_rf, 
 *      beta_rh, Rm_rh, beta_lh, Rm_lh]
 * 
 * @param q Generalized coordinates (12,)
 * @param q_dot Generalized velocities (12,)
 * @param I_c Leg inertias [I_lf, I_rf, I_rh, I_lh] (4,)
 * @param M Output: Mass matrix (12x12)
 * @param C Output: Coriolis matrix (12x12)
 * @param G Output: Gravity vector (12,)
 * @param D Output: Inertia rate matrix (12x12)
 */
void compute_dynamics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& q_dot,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    Eigen::Ref<Eigen::MatrixXd> M,
    Eigen::Ref<Eigen::MatrixXd> C,
    Eigen::Ref<Eigen::VectorXd> G,
    Eigen::Ref<Eigen::MatrixXd> D
);

/**
 * @brief Compute mass matrix only (if only M is needed)
 */
void compute_mass_matrix(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    Eigen::Ref<Eigen::MatrixXd> M
);

} // namespace quadruped_dynamics
