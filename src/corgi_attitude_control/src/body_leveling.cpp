#include "corgi_attitude_control/body_leveling.hpp"

#include <cmath>
#include <algorithm>

BodyLevelingController::BodyLevelingController(
    bool   sim,
    double BL,
    double BW,
    double stand_height,
    double roll_thresh,
    double pitch_thresh,
    double omega_thresh)
: leg_model_(sim),
  BL_(BL), BW_(BW),
  stand_height_(stand_height),
  roll_thresh_(roll_thresh),
  pitch_thresh_(pitch_thresh),
  omega_thresh_(omega_thresh)
{
    // Hip positions in body frame
    // leg 0 = FL (+BL/2, +BW/2)
    // leg 1 = FR (+BL/2, -BW/2)
    // leg 2 = RR (-BL/2, -BW/2)
    // leg 3 = RL (-BL/2, +BW/2)
    hip_x_[0] =  BL_ / 2.0;  hip_y_[0] =  BW_ / 2.0;
    hip_x_[1] =  BL_ / 2.0;  hip_y_[1] = -BW_ / 2.0;
    hip_x_[2] = -BL_ / 2.0;  hip_y_[2] = -BW_ / 2.0;
    hip_x_[3] = -BL_ / 2.0;  hip_y_[3] =  BW_ / 2.0;
}

std::array<std::array<double, 4>, 2>
BodyLevelingController::compute(
    double roll, double pitch,
    const std::array<int, 4>    & swing_mask,
    const std::array<double, 4> & current_theta,
    const std::array<double, 4> & current_beta)
{
    std::array<double, 4> theta_out = current_theta;
    std::array<double, 4> beta_out  = current_beta;

    for (int i = 0; i < 4; ++i)
    {
        if (swing_mask[i] == 1) {
            // Leg is swinging — pass through unchanged
            continue;
        }

        // Δh: vertical hip displacement due to body tilt.  Positive Δh means
        // that hip is higher than nominal, so the required leg depth is shorter.
        //
        // For roll (rotation about X, +roll = right-side-down):
        //   hip_y > 0 (left side) → goes UP → leg needs to be SHORTER   (Δh > 0)
        //   hip_y < 0 (right)    → goes DOWN → leg needs to be LONGER   (Δh < 0)
        //
        // For pitch (rotation about Y, +pitch = nose-up):
        //   hip_x > 0 (front)    → goes UP → leg SHORTER  (Δh > 0)
        //   hip_x < 0 (rear)     → goes DOWN → leg LONGER (Δh < 0)
        //
        // Required foot-to-hip distance: d = stand_height - Δh
        double delta_h = hip_y_[i] * std::sin(roll) * std::cos(pitch) - hip_x_[i] * std::sin(pitch);
        double target_depth = stand_height_ - delta_h;

        // Clamp to safe range (same limits as set_stand_height in walk_gait)
        target_depth = std::max(0.12, std::min(0.34, target_depth));

        // Linearised IK via numerical Jacobian of G w.r.t. (theta, beta).
        // Solves:  J * [d_theta; d_beta] = [0; dG_y_desired]
        //   row 0: keep G_x (foot horizontal position) unchanged
        //   row 1: achieve the required change in G_y (depth)
        // This avoids absolute IK branch-jumps for small depth corrections.
        leg_model_.forward(current_theta[i], current_beta[i]);
        const double G0_x = leg_model_.G[0];
        const double G0_y = leg_model_.G[1];

        // G[1] < 0 (foot below hip), so current_depth = -G0_y
        const double dG_y_desired = -target_depth - G0_y;  // target G_y − current G_y

        // Numerical partial derivatives
        const double eps = 1e-5;
        leg_model_.forward(current_theta[i] + eps, current_beta[i]);
        const double dGx_dth = (leg_model_.G[0] - G0_x) / eps;
        const double dGy_dth = (leg_model_.G[1] - G0_y) / eps;

        leg_model_.forward(current_theta[i], current_beta[i] + eps);
        const double dGx_dbe = (leg_model_.G[0] - G0_x) / eps;
        const double dGy_dbe = (leg_model_.G[1] - G0_y) / eps;

        // Cramer's rule for 2×2 system
        const double det = dGx_dth * dGy_dbe - dGx_dbe * dGy_dth;
        double d_theta = 0.0, d_beta = 0.0;
        if (std::abs(det) > 1e-10) {
            d_theta = (-dGx_dbe * dG_y_desired) / det;
            d_beta  = ( dGx_dth * dG_y_desired) / det;
        } else {
            // Near-singular: adjust only beta (it dominates depth change)
            if (std::abs(dGy_dbe) > 1e-6)
                d_beta = dG_y_desired / dGy_dbe;
        }

        theta_out[i] = current_theta[i] + d_theta;
        beta_out[i]  = current_beta[i]  + d_beta;
    }

    return {theta_out, beta_out};
}

bool BodyLevelingController::is_stable(
    double roll, double pitch,
    double omega_x, double omega_y) const
{
    return (std::abs(roll)    < roll_thresh_)  &&
           (std::abs(pitch)   < pitch_thresh_) &&
           (std::abs(omega_x) < omega_thresh_) &&
           (std::abs(omega_y) < omega_thresh_);
}
