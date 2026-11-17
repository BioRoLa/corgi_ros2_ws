#ifndef CORGI_UTILS_SWING_PLANNER_HPP
#define CORGI_UTILS_SWING_PLANNER_HPP

// Swing-leg planning ported from LegWheel (BioRoLa/LegWheel):
//   legwheel/bezier/swing.py                  SwingProfile, SwingLegPlanner
//   legwheel/planners/trajectory_planning_3d.py  generate_trajectory (swing part)
// restricted to the sagittal plane with the ABAD joint at gamma = 0.
//
// Frame: the leg's hip frame, x forward (direction of travel for +velocity),
// z up. The swing is planned relative to the hip, as LegWheel plans it in the
// body frame, and it tracks the rim point that will touch the ground at
// touchdown (LegWheel "material point tracking").
//
// LegWheel solves the swing shape with NLopt COBYLA. This port uses its own
// bounded Nelder-Mead on the same objectives, so the shape parameters can
// differ slightly where the objective is flat (a few millimetres of foot path).

#include <array>
#include <vector>

#include "corgi_utils/bezier.hpp"
#include "corgi_utils/leg_model.hpp"

namespace legwheel_swing {

// 12-point Bezier swing curve, swing.py SwingProfile restricted to
// [forward, height]. Local x runs from 0 to L (always >= 0); a backward step
// is mirrored about the lift-off point.
class Profile {
public:
    Profile() = default;
    Profile(double L, double h, double dh, double dL1, double dL2, double dL3, double dL4,
            double dH1, double dH2, double offset_y, double diff_h,
            double step_sign = 1.0, double offset_x_true = 0.0);

    // Point on the curve at normalised time t in [0, 1], in the planning frame.
    std::array<double, 2> point(double t) const;
    // Same point in the local frame (x from 0 to L), without the mirroring.
    std::array<double, 2> local_point(double t) const;

private:
    std::array<std::array<double, 2>, 12> cp_{};
    double offset_y_ = 0.0;
    double step_sign_ = 1.0;
    double offset_x_true_ = 0.0;
};

// swing.py SwingLegPlanner: fits the curve shape (dL1, dL2, dH1 at lift-off and
// dL3, dL4, dH2 at touchdown) to the desired boundary velocities.
class Planner {
public:
    // T_sw: swing duration (s).
    explicit Planner(double T_sw) : T_sw_(T_sw) {}

    // p_lo, p_td: [forward, height] (m); v_lo, v_td: [forward, height] (m/s).
    Profile solve(const std::array<double, 2>& p_lo, const std::array<double, 2>& p_td,
                  double step_h, const std::array<double, 2>& v_lo,
                  const std::array<double, 2>& v_td);

    // Shape parameters of the last solve: {dL1, dL2, dH1, dL3, dL4, dH2}.
    std::array<double, 6> last_params() const { return last_params_; }

private:
    double T_sw_;
    double step_L_ = 0.0, step_h_ = 0.0, diff_h_ = 0.0, step_sign_ = 1.0;
    std::array<double, 2> p_lo_{}, p_td_{}, v_lo_{}, v_td_{};
    std::array<double, 6> last_params_{};

    double objective_lo(const std::array<double, 3>& x) const;
    double objective_td(const std::array<double, 3>& x) const;
    double backtracking_penalty(const Profile& profile) const;
};

// LegWheel swing velocity scale (gait_generator_3d.py "Swing guard"): the
// lift-off and touchdown velocities shrink as the stance sweep uses more of the
// +-40 deg touchdown beta range. beta_td is the stance half sweep (rad).
double swing_velocity_scale(double beta_td);

// One leg's swing in the hip frame, tracking the rim point that touches the
// ground at touchdown.
class RimSwing {
public:
    RimSwing() = default;

    // q_lo: last stance pose {theta, beta}; q_prev: the pose one control period
    // earlier. td_x: touchdown contact point ahead of the hip (m), at height
    // -stand_height. v_body: hip velocity (m/s, signed). T_sw: swing duration (s).
    void plan(LegModel& leg, const std::array<double, 2>& q_lo, const std::array<double, 2>& q_prev,
              double td_x, double stand_height, double v_body, double velocity_scale,
              double step_height, double T_sw, double dt, int direction);

    // Pose {theta, beta} at swing fraction ratio in [0, 1]; guess is the
    // previous pose.
    std::array<double, 2> eta(LegModel& leg, double ratio, const std::array<double, 2>& guess) const;

    int getDirection() const { return direction_; }

    // Tracked rim point for a pose (hip frame).
    std::array<double, 2> rim_point(LegModel& leg, double theta, double beta) const;
    // Inverse of rim_point.
    std::array<double, 2> solve_pose(LegModel& leg, const std::array<double, 2>& target,
                                     std::array<double, 2> guess) const;

private:
    Profile profile_;
    double psi_ = 0.0;  // angle from (G - O_r) to the tracked rim point (rad)
    int direction_ = 1;
};

}  // namespace legwheel_swing

#endif  // CORGI_UTILS_SWING_PLANNER_HPP
