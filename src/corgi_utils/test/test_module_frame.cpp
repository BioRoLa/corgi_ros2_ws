// Tests for corgi_utils/module_frame.hpp. The cases mirror the sim driver's
// corgi_sim/test/test_frame_transforms.py, so the C++ boundary (the robot
// bridge) and the Python one (the sim driver) are held to the same algebra.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "corgi_utils/module_frame.hpp"

using corgi::ModuleDirections;
using corgi::ModuleFrame;

namespace {

const double TH0 = 17.0 * M_PI / 180.0;

// motor angles in the frame they are given in (CONVENTIONS.md section 3)
double phi_l(double theta, double beta) { return beta + theta - TH0; }
double phi_r(double theta, double beta) { return beta - theta + TH0; }

std::vector<ModuleDirections> all_dirs()
{
    return {{1, 1, 1}, {1, -1, 1}, {1, 1, -1}, {1, -1, -1}};
}

// the configured legs, A..D (CONVENTIONS.md section 5)
std::vector<ModuleDirections> configured_legs()
{
    return {{1, 1, 1}, {1, -1, -1}, {1, -1, 1}, {1, 1, -1}};
}

}  // namespace

TEST(ModuleFrame, PositionsAreSelfInverse)
{
    for (const auto& d : all_dirs()) {
        ModuleFrame f(d);
        EXPECT_DOUBLE_EQ(f.beta(f.beta(0.3)), 0.3);
        EXPECT_DOUBLE_EQ(f.gamma(f.gamma(-0.1)), -0.1);
        EXPECT_DOUBLE_EQ(f.theta(1.6), 1.6);
    }
}

TEST(ModuleFrame, TorquesVelocitiesAndGainsAreSelfInverse)
{
    for (const auto& d : all_dirs()) {
        ModuleFrame f(d);
        auto t = f.torque_rl(4.0, -2.0);
        EXPECT_EQ(f.torque_rl(t.first, t.second), std::make_pair(4.0, -2.0));
        auto v = f.velocity_rl(0.7, 1.3);
        EXPECT_EQ(f.velocity_rl(v.first, v.second), std::make_pair(0.7, 1.3));
        auto g = f.gain_rl(120.0, 45.0);
        EXPECT_EQ(f.gain_rl(g.first, g.second), std::make_pair(120.0, 45.0));
        EXPECT_DOUBLE_EQ(f.torque_h(f.torque_h(2.5)), 2.5);
    }
}

TEST(ModuleFrame, MirroredLegSwapsAndNegatesTorques)
{
    ModuleFrame b({1, -1, 1});
    EXPECT_EQ(b.torque_rl(4.0, -2.0), std::make_pair(2.0, -4.0));
    EXPECT_EQ(b.velocity_rl(4.0, -2.0), std::make_pair(2.0, -4.0));
}

TEST(ModuleFrame, UnmirroredLegIsUntouched)
{
    ModuleFrame a({1, 1, 1});
    EXPECT_EQ(a.torque_rl(4.0, -2.0), std::make_pair(4.0, -2.0));
    EXPECT_EQ(a.gain_rl(120.0, 45.0), std::make_pair(120.0, 45.0));
    EXPECT_DOUBLE_EQ(a.torque_h(3.0), 3.0);
}

TEST(ModuleFrame, GainsSwapButAreNeverNegated)
{
    ModuleFrame b({1, -1, -1});
    EXPECT_EQ(b.gain_rl(120.0, 45.0), std::make_pair(45.0, 120.0));
    for (const auto& d : all_dirs()) {
        auto g = ModuleFrame(d).gain_rl(120.0, 45.0);
        EXPECT_GT(g.first, 0.0);
        EXPECT_GT(g.second, 0.0);
    }
}

TEST(ModuleFrame, AbadTorqueAndVelocityFollowGammaDirection)
{
    ModuleFrame d({1, 1, -1});
    EXPECT_DOUBLE_EQ(d.torque_h(3.0), -3.0);
    EXPECT_DOUBLE_EQ(d.velocity_h(0.5), -0.5);
    EXPECT_DOUBLE_EQ(d.gamma(0.2), -0.2);
}

// The torque map is not a free choice: it is forced by the position map. Map a
// small ROS-frame motion into the module frame through the position transform
// (finite differences of the phi formulas) and check that power, tau . phidot,
// is the same in both frames for every configuration.
TEST(ModuleFrame, PowerIsFrameInvariant)
{
    const double th = 95.0 * M_PI / 180.0, be = 0.4, dt = 1e-6;
    const double dth = 0.3 * dt, dbe = -0.7 * dt;  // an arbitrary motion
    for (const auto& d : all_dirs()) {
        ModuleFrame f(d);
        const double pl = (phi_l(th + dth, be + dbe) - phi_l(th, be)) / dt;
        const double pr = (phi_r(th + dth, be + dbe) - phi_r(th, be)) / dt;
        const double plm = (phi_l(f.theta(th + dth), f.beta(be + dbe)) - phi_l(f.theta(th), f.beta(be))) / dt;
        const double prm = (phi_r(f.theta(th + dth), f.beta(be + dbe)) - phi_r(f.theta(th), f.beta(be))) / dt;
        const double tr = 4.0, tl = -2.0;
        const auto tm = f.torque_rl(tr, tl);
        EXPECT_NEAR(tr * pr + tl * pl, tm.first * prm + tm.second * plm, 1e-9);
    }
}

// A PD law evaluated in the ROS frame and converted gives the same torque as the
// same law evaluated in the module frame on module-frame angles and converted
// gains -- the property the half-conversion broke (torques and gains left in the
// ROS frame while the angles were mirrored).
TEST(ModuleFrame, PdLawIsFrameConsistent)
{
    const double th = 95.0 * M_PI / 180.0, be = 0.40;      // measured
    const double thc = 97.0 * M_PI / 180.0, bec = 0.35;    // commanded
    const double kp_r = 120.0, kp_l = 45.0;
    for (const auto& d : all_dirs()) {
        ModuleFrame f(d);
        // ROS frame, then convert the torque
        const double tr_ros = kp_r * (phi_r(thc, bec) - phi_r(th, be));
        const double tl_ros = kp_l * (phi_l(thc, bec) - phi_l(th, be));
        const auto expect = f.torque_rl(tr_ros, tl_ros);
        // module frame: converted gains on module-frame angles
        const auto km = f.gain_rl(kp_r, kp_l);
        const double tr_m = km.first * (phi_r(f.theta(thc), f.beta(bec)) - phi_r(f.theta(th), f.beta(be)));
        const double tl_m = km.second * (phi_l(f.theta(thc), f.beta(bec)) - phi_l(f.theta(th), f.beta(be)));
        EXPECT_NEAR(tr_m, expect.first, 1e-12);
        EXPECT_NEAR(tl_m, expect.second, 1e-12);
    }
}

TEST(ModuleFrame, EveryConfiguredLegRoundTrips)
{
    for (const auto& d : configured_legs()) {
        ModuleFrame f(d);
        auto t = f.torque_rl(1.5, -0.25);
        EXPECT_EQ(f.torque_rl(t.first, t.second), std::make_pair(1.5, -0.25));
        EXPECT_DOUBLE_EQ(f.gamma(f.gamma(0.05)), 0.05);
    }
}

TEST(ModuleFrame, ThetaMirrorIsRejectedNotGuessed)
{
    EXPECT_THROW(ModuleFrame({-1, 1, 1}, "X"), std::invalid_argument);
}

TEST(ModuleFrame, NonUnitDirectionIsRejected)
{
    EXPECT_THROW(ModuleFrame({1, 0.5, 1}, "X"), std::invalid_argument);
    EXPECT_THROW(ModuleFrame({1, 1, 0.0}, "X"), std::invalid_argument);
}

// The regression this file exists for: on a mirrored leg the torques must be
// transformed, on an unmirrored one they must pass through.
TEST(ModuleFrame, TheRegressionItself)
{
    ModuleFrame a({1, 1, 1}), b({1, -1, -1});
    EXPECT_EQ(a.torque_rl(3.0, 1.0), std::make_pair(3.0, 1.0));
    EXPECT_NE(b.torque_rl(3.0, 1.0), std::make_pair(3.0, 1.0));
}
