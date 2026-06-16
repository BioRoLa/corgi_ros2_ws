#ifndef BODY_LEVELING_HPP
#define BODY_LEVELING_HPP

#include <array>
#include "corgi_utils/leg_model.hpp"

/**
 * BodyLevelingController
 *
 * Given IMU roll/pitch angles and the current swing-phase mask, computes
 * corrected theta/beta for every STANCE leg so that the robot body remains
 * level with the ground.
 *
 * Hip-position convention (matches corgi_gait_selector / walk_gait):
 *   leg 0 = FL  (+BL/2, +BW/2)
 *   leg 1 = FR  (+BL/2, -BW/2)
 *   leg 2 = RR  (-BL/2, -BW/2)
 *   leg 3 = RL  (-BL/2, +BW/2)
 * (body frame: +x = forward, +y = left, +z = up)
 */
class BodyLevelingController
{
public:
    /**
     * @param sim          Passed to LegModel constructor
     * @param BL           Body length  (m), default 0.444
     * @param BW           Body width   (m), default 0.4
     * @param stand_height Nominal stand height (m), default 0.25
     * @param roll_thresh  Stability roll  threshold (rad), default 3°
     * @param pitch_thresh Stability pitch threshold (rad), default 3°
     * @param omega_thresh Stability angular-rate threshold (rad/s), default 0.05
     */
    BodyLevelingController(bool   sim          = true,
                           double BL           = 0.444,
                           double BW           = 0.4,
                           double stand_height = 0.20,
                           double roll_thresh  = 0.0087,
                           double pitch_thresh = 0.0087,
                           double omega_thresh = 0.02);

    /**
     * Compute corrected joint angles for all four legs.
     *
     * @param roll         Body roll  (rad) derived from IMU quaternion (+= right-side-down)
     * @param pitch        Body pitch (rad) derived from IMU quaternion (+= nose-up)
     * @param swing_mask   swing_phase[4]: 1 = leg is swinging (skip correction), 0 = stance
     * @param current_theta Current theta for each leg (used for swing legs passthrough)
     * @param current_beta  Current beta  for each leg (used for swing legs passthrough)
     * @return             {theta[4], beta[4]} — corrected for stance legs, passthrough for swing
     */
    std::array<std::array<double, 4>, 2> compute(
        double roll, double pitch,
        const std::array<int, 4>    & swing_mask,
        const std::array<double, 4> & current_theta,
        const std::array<double, 4> & current_beta);

    /**
     * Stability check — returns true when body attitude is within thresholds.
     *
     * @param roll    Body roll  (rad)
     * @param pitch   Body pitch (rad)
     * @param omega_x Angular velocity around X (rad/s)
     * @param omega_y Angular velocity around Y (rad/s)
     */
    bool is_stable(double roll, double pitch, double omega_x, double omega_y) const;

    void set_stand_height(double h) { stand_height_ = h; }
    void set_thresholds(double roll_t, double pitch_t, double omega_t)
    {
        roll_thresh_  = roll_t;
        pitch_thresh_ = pitch_t;
        omega_thresh_ = omega_t;
    }

private:
    LegModel leg_model_;
    double BL_, BW_;
    double stand_height_;
    double roll_thresh_, pitch_thresh_, omega_thresh_;

    // Hip positions in body frame [i][0]=x, [i][1]=y
    double hip_x_[4];
    double hip_y_[4];
};

#endif // BODY_LEVELING_HPP
