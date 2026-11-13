// Conversion between the ROS joint convention and a leg module's own frame.
//
// The convention and the derivation are in CONVENTIONS.md §5 at the top of
// the workspace. This is the C++ copy of the algebra the sim driver applies in
// corgi_driver.py (ros_to_module / convert_torque / convert_gains); the two
// must agree, and test/test_module_frame.cpp checks the cases.
//
// Every map here is self-inverse, so the same call converts ROS -> module and
// module -> ROS.
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace corgi {

struct ModuleDirections {
    double theta = 1.0;  // must be +1 (the 17 deg offset in phi breaks the swap)
    double beta = 1.0;   // -1 mirrors beta and swaps the r/l motor pair
    double gamma = 1.0;  // -1 mirrors the ABAD
};

class ModuleFrame {
public:
    ModuleFrame() = default;

    explicit ModuleFrame(const ModuleDirections& d, const std::string& leg = "")
        : d_(d)
    {
        check(d_.theta, "theta", leg);
        check(d_.beta, "beta", leg);
        check(d_.gamma, "gamma", leg);
        if (d_.theta < 0.0) {
            throw std::invalid_argument(
                "leg " + leg + ": joint_dir.theta = -1 is not supported; the 17 deg "
                "offset in phi breaks the r/l swap (CONVENTIONS.md section 5)");
        }
    }

    const ModuleDirections& directions() const { return d_; }
    bool mirrors_beta() const { return d_.beta < 0.0; }

    // positions
    double theta(double x) const { return x; }
    double beta(double x) const { return d_.beta * x; }
    double gamma(double x) const { return d_.gamma * x; }

    // r/l torque or velocity pair: swap and negate when beta is mirrored
    std::pair<double, double> torque_rl(double r, double l) const
    {
        return mirrors_beta() ? std::make_pair(-l, -r) : std::make_pair(r, l);
    }
    std::pair<double, double> velocity_rl(double r, double l) const { return torque_rl(r, l); }

    // r/l gain pair (kp, ki or kd): swap, never negate -- a gain stays positive
    std::pair<double, double> gain_rl(double r, double l) const
    {
        return mirrors_beta() ? std::make_pair(l, r) : std::make_pair(r, l);
    }

    // ABAD torque or velocity; ABAD gains are unchanged
    double torque_h(double x) const { return d_.gamma * x; }
    double velocity_h(double x) const { return d_.gamma * x; }

private:
    static void check(double v, const char* name, const std::string& leg)
    {
        if (!(v == 1.0 || v == -1.0)) {
            throw std::invalid_argument("leg " + leg + ": joint_dir." + name +
                                        " must be +1 or -1, got " + std::to_string(v));
        }
    }

    ModuleDirections d_;
};

}  // namespace corgi
