#ifndef CORGI_UTILS_ROBOT_PARAMS_HPP
#define CORGI_UTILS_ROBOT_PARAMS_HPP

#include <array>
#include <stdexcept>

// Body dimensions of the ABAD Corgi. Values follow LegWheel RobotParams
// (System_Parameters.pdf) and the joint anchors in CorgiRobotABAD.proto.
//
// Body frame {B}: origin at the chassis centre, +x front, +y left, +z up.
// Modules: 0 = A (left front), 1 = B (right front), 2 = C (right hind),
// 3 = D (left hind).
namespace corgi_robot {

constexpr double chassis_length = 0.694;
constexpr double chassis_width  = 0.352;
constexpr double chassis_height = 0.138;

constexpr double wheel_base       = 0.510;     // front to hind ABAD joint, along x
constexpr double abad_track       = 0.240;     // left to right ABAD axis, along y
constexpr double abad_axis_height = 0.057166;  // ABAD axis above the body origin
constexpr double abad_axis_to_wheel_plane = 0.091675;  // ABAD axis to wheel mid-plane

// ABAD joint position in {B}. LegModel::contact_p_3d is relative to this point.
inline std::array<double, 3> abad_origin(int module) {
    if (module < 0 || module >= 4) throw std::out_of_range("Invalid module index");
    double sx = (module == 0 || module == 1) ? 1.0 : -1.0;
    double sy = (module == 0 || module == 3) ? 1.0 : -1.0;
    return {sx * wheel_base / 2.0, sy * abad_track / 2.0, abad_axis_height};
}

} // namespace corgi_robot

#endif // CORGI_UTILS_ROBOT_PARAMS_HPP
