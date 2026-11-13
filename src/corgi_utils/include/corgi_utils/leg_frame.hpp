// The leg model's Cartesian frame and the body frame (CONVENTIONS.md section 4).
//
// LegModel's contact points and calculate_jacobian_3d work in one frame per
// leg: x BACKWARD (a foot behind the hip has x > 0), y OUTWARD (away from the
// body's centreline: the wheel offset d_wheel is +y), z up. The body frame is
// x forward, y left, z up. So a vector converts with
//
//     v_body = diag(-1, s, +1) v_leg        s = +1 left legs (A, D), -1 right (B, C)
//
// which is its own inverse. s is geometry -- which side of the body the leg
// is on -- not a per-module wiring sign; the wiring is converted at the
// hardware boundary (module_frame.hpp).
//
// Until 2026-11-13 force_control fed body-frame commands straight into the leg
// frame (+fx at the ground came out -fx on every leg, +fy -fy on B and C;
// gslip_pronk's lat_sign undid the y half), and force_estimation wrote this map
// out per leg as "fx = est, fy = +/-est". Both now call this.
#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

namespace corgi {

// +1 for a leg on the left of the body (A = 0, D = 3), -1 on the right (B = 1, C = 2).
inline double hip_side_y(int leg)
{
    switch (leg) {
        case 0: case 3: return +1.0;
        case 1: case 2: return -1.0;
        default: throw std::out_of_range("leg index " + std::to_string(leg) + " (expected 0..3 = A..D)");
    }
}

inline Eigen::Vector3d leg_model_to_body(const Eigen::Vector3d& v, int leg)
{
    return Eigen::Vector3d(-v.x(), hip_side_y(leg) * v.y(), v.z());
}

inline Eigen::Vector3d body_to_leg_model(const Eigen::Vector3d& v, int leg)
{
    return leg_model_to_body(v, leg);  // self-inverse
}

}  // namespace corgi
