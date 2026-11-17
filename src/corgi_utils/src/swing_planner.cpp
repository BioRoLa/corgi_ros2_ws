#include "corgi_utils/swing_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace legwheel_swing {

namespace {

// Binomial coefficients C(11, i) for the 12-point curve.
constexpr std::array<double, 12> kBinom = {1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1};

// Values below are LegWheel's (swing.py, config RobotParams).
constexpr double kShapeDh = 0.01;            // mid-swing height adjustment
constexpr double kPresetDL1 = 0.05;          // lift-off shape preset
constexpr double kPresetDL2 = 0.0;
constexpr double kPresetDL3 = 0.05;          // touchdown shape preset
constexpr double kPresetDL4 = 0.0;
// Boundary velocities are finite differences over 1 ms of swing, whatever the
// control rate (LegWheel plans at dt = 1 ms).
constexpr double kVelocitySampleT = 0.001;   // s
constexpr double kBacktrackAllowance = 0.015;  // m
constexpr double kBacktrackWeight = 10.0;
constexpr double kTouchdownVelMax = 0.3;     // m/s, TOUCHDOWN_VEL_H_MAX
constexpr double kBetaMax = 40.0 * M_PI / 180.0;  // BETA_MAX_DEG
constexpr double kUsageThreshold = 0.15;     // STEP_USAGE_THRESHOLD
constexpr double kDecay = 0.3;               // STEP_DECAY_COEFF
constexpr double kScaleFloor = 0.2;          // STEP_FLOOR

using Vec3 = std::array<double, 3>;

// Bounded Nelder-Mead. Bounds are enforced by clipping before each objective
// evaluation and a steep penalty outside them.
Vec3 minimize(const std::function<double(const Vec3&)>& f, Vec3 x0, const Vec3& lb, const Vec3& ub)
{
    auto clip = [&](Vec3 x) {
        for (int i = 0; i < 3; ++i) x[i] = std::clamp(x[i], lb[i], ub[i]);
        return x;
    };
    auto cost = [&](const Vec3& x) {
        double out = 0.0;
        for (int i = 0; i < 3; ++i) out += std::max(0.0, lb[i] - x[i]) + std::max(0.0, x[i] - ub[i]);
        return f(clip(x)) + 1e3 * out;
    };
    Vec3 best = clip(x0);
    for (int restart = 0; restart < 3; ++restart) {
        std::array<Vec3, 4> s;
        std::array<double, 4> fs;
        s[0] = best;
        for (int i = 0; i < 3; ++i) {
            s[i + 1] = best;
            double step = 0.01;
            if (s[i + 1][i] + step > ub[i]) step = -step;
            s[i + 1][i] += step;
        }
        for (int i = 0; i < 4; ++i) fs[i] = cost(s[i]);
        for (int iter = 0; iter < 600; ++iter) {
            std::array<int, 4> idx = {0, 1, 2, 3};
            std::sort(idx.begin(), idx.end(), [&](int a, int b) { return fs[a] < fs[b]; });
            std::array<Vec3, 4> s2;
            std::array<double, 4> f2;
            for (int i = 0; i < 4; ++i) { s2[i] = s[idx[i]]; f2[i] = fs[idx[i]]; }
            s = s2; fs = f2;
            double size = 0.0;
            for (int i = 1; i < 4; ++i)
                for (int k = 0; k < 3; ++k) size = std::max(size, std::abs(s[i][k] - s[0][k]));
            if (size < 1e-9) break;
            Vec3 c = {0, 0, 0};
            for (int i = 0; i < 3; ++i)
                for (int k = 0; k < 3; ++k) c[k] += s[i][k] / 3.0;
            auto along = [&](double t) {
                Vec3 p;
                for (int k = 0; k < 3; ++k) p[k] = c[k] + t * (s[3][k] - c[k]);
                return p;
            };
            Vec3 xr = along(-1.0);
            double fr = cost(xr);
            if (fr < fs[0]) {
                Vec3 xe = along(-2.0);
                double fe = cost(xe);
                if (fe < fr) { s[3] = xe; fs[3] = fe; } else { s[3] = xr; fs[3] = fr; }
            } else if (fr < fs[2]) {
                s[3] = xr; fs[3] = fr;
            } else {
                Vec3 xc = fr < fs[3] ? along(-0.5) : along(0.5);
                double fc = cost(xc);
                if (fc < std::min(fr, fs[3])) {
                    s[3] = xc; fs[3] = fc;
                } else {
                    for (int i = 1; i < 4; ++i) {
                        for (int k = 0; k < 3; ++k) s[i][k] = s[0][k] + 0.5 * (s[i][k] - s[0][k]);
                        fs[i] = cost(s[i]);
                    }
                }
            }
        }
        int ib = static_cast<int>(std::min_element(fs.begin(), fs.end()) - fs.begin());
        best = clip(s[ib]);
    }
    return best;
}

}  // namespace

// ---------------------------------------------------------------- Profile

Profile::Profile(double L, double h, double dh, double dL1, double dL2, double dL3, double dL4,
                 double dH1, double dH2, double offset_y, double diff_h,
                 double step_sign, double offset_x_true)
    : offset_y_(offset_y), step_sign_(step_sign), offset_x_true_(offset_x_true)
{
    const std::array<double, 2> c0 = {0.0, 0.0};
    const std::array<double, 2> c1 = {c0[0] - dL1, c0[1] + dH1};
    const std::array<double, 2> c2 = {c1[0] - dL2, c1[1] + h - dH1};
    const std::array<double, 2> c5 = {c2[0] + 0.5 * L + dL1 + dL2, c2[1]};
    const std::array<double, 2> c7 = {c5[0] + 0.5 * L + dL3 + dL4, c5[1] + dh};
    const std::array<double, 2> c10 = {c7[0] - dL4, c7[1] - (h + dh) + diff_h + dH2};
    const std::array<double, 2> c11 = {c10[0] - dL3, c10[1] - dH2};
    cp_ = {c0, c1, c2, c2, c2, c5, c5, c7, c7, c7, c10, c11};
}

std::array<double, 2> Profile::local_point(double t) const
{
    // Bernstein weights C(11,i) (1-t)^(11-i) t^i.
    std::array<double, 12> pt, pu;
    pt[0] = 1.0; pu[0] = 1.0;
    for (int i = 1; i < 12; ++i) { pt[i] = pt[i - 1] * t; pu[i] = pu[i - 1] * (1.0 - t); }
    double x = 0.0, y = 0.0;
    for (int i = 0; i < 12; ++i) {
        const double w = kBinom[i] * pu[11 - i] * pt[i];
        x += w * cp_[i][0];
        y += w * cp_[i][1];
    }
    return {x, y + offset_y_};
}

std::array<double, 2> Profile::point(double t) const
{
    auto p = local_point(t);
    p[0] = offset_x_true_ + step_sign_ * p[0];
    return p;
}

// ---------------------------------------------------------------- Planner

double Planner::backtracking_penalty(const Profile& profile) const
{
    const double sx = p_td_[0] - p_lo_[0], sy = p_td_[1] - p_lo_[1];
    const double len = std::hypot(sx, sy);
    if (len < 1e-9) return 0.0;
    double min_progress = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 80; ++k) {
        const auto p = profile.point(0.35 * k / 79.0);
        min_progress = std::min(min_progress, ((p[0] - p_lo_[0]) * sx + (p[1] - p_lo_[1]) * sy) / len);
    }
    const double excess = std::max(0.0, std::max(0.0, -min_progress) - kBacktrackAllowance);
    return kBacktrackWeight * (excess / len) * (excess / len);
}

double Planner::objective_lo(const std::array<double, 3>& x) const
{
    const Profile sp(step_L_, step_h_, kShapeDh, x[0], x[1], kPresetDL3, kPresetDL4, x[2], 0.0,
                     p_lo_[1], diff_h_, step_sign_, p_lo_[0]);
    // Velocity in the local (|L|) frame against the mirrored target.
    const auto p0 = sp.local_point(0.0);
    const auto p1 = sp.local_point(kVelocitySampleT / T_sw_);
    const double ex = v_lo_[0] - (p1[0] - p0[0]) / kVelocitySampleT;
    const double ey = v_lo_[1] - (p1[1] - p0[1]) / kVelocitySampleT;
    return std::hypot(ex, ey) + backtracking_penalty(sp);
}

double Planner::objective_td(const std::array<double, 3>& x) const
{
    // As in LegWheel, evaluated on a curve without offsets or height change.
    const Profile sp(step_L_, step_h_, kShapeDh, kPresetDL1, kPresetDL2, x[0], x[1], 0.0, x[2], 0.0, 0.0);
    const auto p2 = sp.local_point(1.0 - kVelocitySampleT / T_sw_);
    const auto p3 = sp.local_point(1.0);
    return std::hypot(v_td_[0] - (p3[0] - p2[0]) / kVelocitySampleT, v_td_[1] - (p3[1] - p2[1]) / kVelocitySampleT);
}

Profile Planner::solve(const std::array<double, 2>& p_lo, const std::array<double, 2>& p_td,
                       double step_h, const std::array<double, 2>& v_lo,
                       const std::array<double, 2>& v_td)
{
    p_lo_ = p_lo;
    p_td_ = p_td;
    const double dx = p_td[0] - p_lo[0];
    step_sign_ = std::abs(dx) > 1e-6 ? (dx > 0 ? 1.0 : -1.0) : 1.0;
    step_L_ = std::abs(dx);
    diff_h_ = p_td[1] - p_lo[1];
    step_h_ = step_h;
    v_lo_ = {step_sign_ * v_lo[0], v_lo[1]};
    v_td_ = {step_sign_ * v_td[0], v_td[1]};

    const double dH1 = std::max(0.0, std::min(std::abs(v_lo_[1]) * T_sw_ / 11.0, 0.8 * step_h));
    const double dH2 = std::max(0.0, std::min(std::abs(v_td_[1]) * T_sw_ / 11.0, 0.8 * step_h));
    const Vec3 lb = {0.0, 0.0, 0.0};
    const Vec3 ub = {0.5, 0.5, std::min(0.9 * step_h, 0.5)};

    // LegWheel constraints (always met for steps longer than 2 cm), kept as penalties.
    auto f_lo = [&](const Vec3& x) {
        return objective_lo(x) + 1e3 * std::max(0.0, -x[0] - x[1] - (step_L_ / 2.0 - 0.01));
    };
    auto f_td = [&](const Vec3& x) {
        return objective_td(x) + 1e3 * std::max(0.0, (step_L_ / 2.0 + 0.01) - (step_L_ + x[0] + x[1]));
    };
    const Vec3 lo = minimize(f_lo, {kPresetDL1, kPresetDL2, dH1}, lb, ub);
    const Vec3 td = minimize(f_td, {kPresetDL3, kPresetDL4, dH2}, lb, ub);
    last_params_ = {lo[0], lo[1], lo[2], td[0], td[1], td[2]};
    return Profile(step_L_, step_h_, kShapeDh, lo[0], lo[1], td[0], td[1], lo[2], td[2],
                   p_lo[1], diff_h_, step_sign_, p_lo[0]);
}

double swing_velocity_scale(double beta_td)
{
    const double usage = std::abs(beta_td) / kBetaMax;
    if (usage <= kUsageThreshold) return 1.0;
    return std::max(1.0 - kDecay * (usage - kUsageThreshold) / (1.0 - kUsageThreshold), kScaleFloor);
}

// ---------------------------------------------------------------- RimSwing

std::array<double, 2> RimSwing::rim_point(LegModel& leg, double theta, double beta) const
{
    leg.forward(std::clamp(theta, leg.min_theta, leg.max_theta), beta);
    const double ux = leg.G[0] - leg.O_r[0], uy = leg.G[1] - leg.O_r[1];
    const double n = std::hypot(ux, uy);
    const double c = std::cos(psi_), s = std::sin(psi_);
    return {leg.O_r[0] + leg.foot_radius * (c * ux - s * uy) / n,
            leg.O_r[1] + leg.foot_radius * (s * ux + c * uy) / n};
}

std::array<double, 2> RimSwing::solve_pose(LegModel& leg, const std::array<double, 2>& target,
                                           std::array<double, 2> q) const
{
    const double eps = 1e-7;
    for (int iter = 0; iter < 30; ++iter) {
        q[0] = std::clamp(q[0], leg.min_theta, leg.max_theta);
        const auto p = rim_point(leg, q[0], q[1]);
        const double ex = p[0] - target[0], ey = p[1] - target[1];
        if (std::hypot(ex, ey) < 1e-11) break;
        const auto pt = rim_point(leg, q[0] + eps, q[1]);
        const auto pb = rim_point(leg, q[0], q[1] + eps);
        const double j11 = (pt[0] - p[0]) / eps, j12 = (pb[0] - p[0]) / eps;
        const double j21 = (pt[1] - p[1]) / eps, j22 = (pb[1] - p[1]) / eps;
        const double det = j11 * j22 - j12 * j21;
        if (std::abs(det) < 1e-12) break;
        q[0] -= std::clamp((j22 * ex - j12 * ey) / det, -0.2, 0.2);
        q[1] -= std::clamp((-j21 * ex + j11 * ey) / det, -0.2, 0.2);
    }
    q[0] = std::clamp(q[0], leg.min_theta, leg.max_theta);
    return q;
}

void RimSwing::plan(LegModel& leg, const std::array<double, 2>& q_lo, const std::array<double, 2>& q_prev,
                    double td_x, double stand_height, double v_body, double velocity_scale,
                    double step_height, double T_sw, double dt, int direction)
{
    direction_ = direction;
    // Touchdown pose: the foot arc touches the ground at td_x, directly below O_r.
    const auto q_td = leg.inverse(std::array<double, 2>{td_x, -stand_height + leg.foot_radius}, "O_r");
    leg.forward(q_td[0], q_td[1]);
    const double ux = leg.G[0] - leg.O_r[0], uy = leg.G[1] - leg.O_r[1];
    psi_ = std::atan2(-ux, -uy);  // rotation from (G - O_r) to straight down
    const auto p_td = rim_point(leg, q_td[0], q_td[1]);
    const auto p_lo = rim_point(leg, q_lo[0], q_lo[1]);

    // Lift-off velocity of the tracked point: Jacobian times the last stance
    // joint velocity, with the vertical part zeroed (LegWheel).
    const double diff = 1e-5;
    const auto pt = rim_point(leg, q_lo[0] + diff, q_lo[1]);
    const auto pb = rim_point(leg, q_lo[0], q_lo[1] + diff);
    const double qd0 = (q_lo[0] - q_prev[0]) / dt, qd1 = (q_lo[1] - q_prev[1]) / dt;
    std::array<double, 2> v_lo = {((pt[0] - p_lo[0]) * qd0 + (pb[0] - p_lo[0]) * qd1) / diff, 0.0};
    // Touchdown velocity: the hip velocity, capped, with no vertical part.
    std::array<double, 2> v_td = {std::clamp(v_body, -kTouchdownVelMax, kTouchdownVelMax), 0.0};
    v_lo[0] *= velocity_scale;
    v_td[0] *= velocity_scale;

    profile_ = Planner(T_sw).solve(p_lo, p_td, step_height, v_lo, v_td);
}

std::array<double, 2> RimSwing::eta(LegModel& leg, double ratio, const std::array<double, 2>& guess) const
{
    return solve_pose(leg, profile_.point(std::clamp(ratio, 0.0, 1.0)), guess);
}

}  // namespace legwheel_swing
