#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace simplify_dynamics
{

    /**
     * @brief Compute dynamics matrices for corgi robot generalized state.
     *
     * State vector definition (16-dimensional in ABAD mode):
     * q = [x, z, phi, psi, beta_lf, Rm_lf, beta_rf, Rm_rf,
     *      beta_rh, Rm_rh, beta_lh, Rm_lh, gamma_lf, gamma_rf, gamma_rh, gamma_lh]
     *
     * The symbolic dynamics core currently models the first 12 states.
     * Additional gamma states are treated as decoupled channels.
     *
     * @param q Generalized coordinates (dof,)
     * @param q_dot Generalized velocities (dof,)
     * @param I_c Leg inertias [I_lf, I_rf, I_rh, I_lh] (4,)
     * @param M Output: Mass matrix (dof x dof)
     * @param C Output: Coriolis matrix (dof x dof)
     * @param G Output: Gravity vector (dof,)
     * @param D Output: Inertia rate matrix (dof x dof)
     */
    void compute_dynamics(
        const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::VectorXd> &q_dot,
        const Eigen::Ref<const Eigen::VectorXd> &I_c,
        Eigen::Ref<Eigen::MatrixXd> M,
        Eigen::Ref<Eigen::MatrixXd> C,
        Eigen::Ref<Eigen::VectorXd> G,
        Eigen::Ref<Eigen::MatrixXd> D);

    /**
     * @brief Compute mass matrix only (if only M is needed)
     */
    void compute_mass_matrix(
        const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::VectorXd> &I_c,
        Eigen::Ref<Eigen::MatrixXd> M);

} // namespace simplify_dynamics
