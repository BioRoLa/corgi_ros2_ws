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

        // Δh: how much the hip of leg i drops toward the ground due to tilt.
        // A hip on the high side of a roll/pitch must extend its leg further.
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
        double delta_h = hip_y_[i] * std::sin(roll) + hip_x_[i] * std::sin(pitch);
        double target_depth = stand_height_ - delta_h;

        // Clamp to safe range (same limits as set_stand_height in walk_gait)
        target_depth = std::max(0.12, std::min(0.34, target_depth));

        // Preserve the current horizontal foot position (foot is NOT directly below hip
        // during walking — it is offset by ±step/2 in the hip frame).
        // FK gives us the current G position, then IK adjusts only the vertical depth.
        leg_model_.forward(current_theta[i], current_beta[i]);
        double foot_x = leg_model_.G[0];  // hip-frame horizontal offset

        auto eta = leg_model_.inverse({foot_x, -target_depth}, "G");

        theta_out[i] = eta[0];
        beta_out[i]  = eta[1];
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
