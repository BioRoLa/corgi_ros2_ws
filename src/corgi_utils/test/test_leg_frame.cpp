// corgi_utils/leg_frame.hpp: the leg model's frame (x back, y outward, z up)
// against the body frame (x forward, y left, z up).
#include <gtest/gtest.h>

#include "corgi_utils/leg_frame.hpp"

using corgi::body_to_leg_model;
using corgi::hip_side_y;
using corgi::leg_model_to_body;

TEST(LegFrame, SidesFollowTheHipPositions)
{
    // A front-left, B front-right, C rear-right, D rear-left (CONVENTIONS.md section 2)
    EXPECT_EQ(hip_side_y(0), +1.0);
    EXPECT_EQ(hip_side_y(1), -1.0);
    EXPECT_EQ(hip_side_y(2), -1.0);
    EXPECT_EQ(hip_side_y(3), +1.0);
    EXPECT_THROW(hip_side_y(4), std::out_of_range);
    EXPECT_THROW(hip_side_y(-1), std::out_of_range);
}

TEST(LegFrame, IsSelfInverse)
{
    const Eigen::Vector3d v(3.0, -2.0, -80.0);
    for (int leg = 0; leg < 4; ++leg) {
        EXPECT_TRUE(body_to_leg_model(leg_model_to_body(v, leg), leg).isApprox(v));
    }
}

TEST(LegFrame, ForwardIsLegMinusXOnEveryLeg)
{
    for (int leg = 0; leg < 4; ++leg) {
        EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(1, 0, 0), leg).x(), -1.0);
        EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(0, 0, 1), leg).z(), 1.0);
    }
}

TEST(LegFrame, OutwardIsBodyLeftOnTheLeftAndRightOnTheRight)
{
    // a body +y (left) push is outward for A and D, inward for B and C
    EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(0, 1, 0), 0).y(), +1.0);
    EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(0, 1, 0), 1).y(), -1.0);
    EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(0, 1, 0), 2).y(), -1.0);
    EXPECT_DOUBLE_EQ(body_to_leg_model(Eigen::Vector3d(0, 1, 0), 3).y(), +1.0);
}

// The map force_estimation used to write out by hand, with est = -F_leg
// (its -J^-T tau): fx = est_x on every leg, fy = -est_y on A/D and +est_y on
// B/C, fz = -est_z. The helper must reproduce it exactly, or the refactor
// changed the estimate.
TEST(LegFrame, ReproducesTheEstimatorsOldPerLegMapping)
{
    const Eigen::Vector3d est(4.0, 1.5, 90.0);
    for (int leg = 0; leg < 4; ++leg) {
        const Eigen::Vector3d f = -leg_model_to_body(est, leg);
        EXPECT_DOUBLE_EQ(f.x(), est.x());
        EXPECT_DOUBLE_EQ(f.y(), (leg == 1 || leg == 2) ? est.y() : -est.y());
        EXPECT_DOUBLE_EQ(f.z(), -est.z());
    }
}
