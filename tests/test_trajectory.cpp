// SPDX-License-Identifier: MIT
#include <cmath>

#include <gtest/gtest.h>

#include "ethercat_motion/trajectory.hpp"

using namespace ethercat_motion;

namespace {

/// Samples the profile and checks velocity/acceleration bounds and continuity.
void checkProfile(const TrapezoidalProfile& p, double vmax, double amax, double dt = 1e-4) {
    TrajectoryPoint prev = p.sample(0.0);
    const double end = p.duration() + 0.01;
    for (double t = dt; t <= end; t += dt) {
        const TrajectoryPoint cur = p.sample(t);
        EXPECT_LE(std::fabs(cur.velocity), vmax + 1e-6) << "t=" << t;
        EXPECT_LE(std::fabs(cur.acceleration), amax + 1e-6) << "t=" << t;
        // Position continuity: |dp| <= vmax*dt (+ tolerance)
        EXPECT_LE(std::fabs(cur.position - prev.position), vmax * dt + 1e-6) << "t=" << t;
        prev = cur;
    }
}

}  // namespace

TEST(Trajectory, TrapezoidRestToRest) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(0.0, 0.0, 100.0, 10.0, 5.0, 5.0));
    // accel 2 s (10 m), cruise 8 s (80 m), decel 2 s (10 m)
    EXPECT_NEAR(p.duration(), 12.0, 1e-9);
    EXPECT_NEAR(p.sample(1.0).position, 2.5, 1e-9);
    EXPECT_NEAR(p.sample(6.0).velocity, 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.sample(12.5).position, 100.0);
    EXPECT_DOUBLE_EQ(p.sample(12.5).velocity, 0.0);
    checkProfile(p, 10.0, 5.0);
}

TEST(Trajectory, TriangularWhenDistanceShort) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(0.0, 0.0, 1.0, 10.0, 4.0, 4.0));
    // vp = sqrt(a*D) = 2, t = 2 * vp/a = 1 s
    EXPECT_NEAR(p.duration(), 1.0, 1e-9);
    EXPECT_NEAR(p.sample(0.5).velocity, 2.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.finalPosition(), 1.0);
    checkProfile(p, 2.0, 4.0);
}

TEST(Trajectory, AsymmetricAccelDecel) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(10.0, 0.0, -20.0, 5.0, 10.0, 2.5));
    EXPECT_DOUBLE_EQ(p.sample(p.duration()).position, -20.0);
    checkProfile(p, 5.0, 10.0);
}

TEST(Trajectory, ReplanWhileMovingTowardTarget) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(0.0, 5.0, 100.0, 10.0, 5.0, 5.0));
    const TrajectoryPoint start = p.sample(0.0);
    EXPECT_DOUBLE_EQ(start.velocity, 5.0);  // continuous velocity
    EXPECT_DOUBLE_EQ(p.sample(p.duration()).position, 100.0);
    checkProfile(p, 10.0, 5.0);
}

TEST(Trajectory, OvershootReversesAfterStopping) {
    TrapezoidalProfile p;
    // Moving at 10 toward +; target only 1 ahead, stopping needs 10 -> overshoot then return.
    ASSERT_TRUE(p.planPosition(0.0, 10.0, 1.0, 10.0, 5.0, 5.0));
    double max_pos = 0.0;
    for (double t = 0; t <= p.duration(); t += 1e-3) max_pos = std::max(max_pos, p.sample(t).position);
    EXPECT_NEAR(max_pos, 10.0, 1e-3);
    EXPECT_DOUBLE_EQ(p.sample(p.duration() + 1).position, 1.0);
    checkProfile(p, 10.0, 5.0);
}

TEST(Trajectory, MovingAwayFromTarget) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(0.0, -4.0, 10.0, 8.0, 4.0, 4.0));
    EXPECT_DOUBLE_EQ(p.sample(0.0).velocity, -4.0);
    EXPECT_DOUBLE_EQ(p.sample(p.duration()).position, 10.0);
    checkProfile(p, 8.0, 4.0);
}

TEST(Trajectory, FasterThanVmaxDeceleratesFirst) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(0.0, 20.0, 1000.0, 10.0, 5.0, 5.0));
    EXPECT_DOUBLE_EQ(p.sample(0.0).velocity, 20.0);
    EXPECT_NEAR(p.sample(2.0).velocity, 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.sample(p.duration()).position, 1000.0);
}

TEST(Trajectory, ZeroDistance) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planPosition(3.0, 0.0, 3.0, 1.0, 1.0, 1.0));
    EXPECT_DOUBLE_EQ(p.duration(), 0.0);
    EXPECT_TRUE(p.finished(0.0));
    EXPECT_DOUBLE_EQ(p.sample(1.0).position, 3.0);
}

TEST(Trajectory, VelocityProfile) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planVelocity(0.0, 0.0, 10.0, 5.0, 5.0));
    EXPECT_NEAR(p.duration(), 2.0, 1e-9);
    EXPECT_FALSE(p.finished(100.0));  // runs forever at constant velocity
    EXPECT_NEAR(p.sample(3.0).velocity, 10.0, 1e-9);
    EXPECT_NEAR(p.sample(3.0).position, 10.0 + 10.0, 1e-9);
}

TEST(Trajectory, VelocityReversal) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planVelocity(0.0, 10.0, -10.0, 5.0, 10.0));
    EXPECT_NEAR(p.sample(1.0).velocity, 0.0, 1e-9);  // decel 10/10 = 1 s
    EXPECT_NEAR(p.sample(3.0).velocity, -10.0, 1e-9);  // accel 10/5 = 2 s
}

TEST(Trajectory, StopDistance) {
    TrapezoidalProfile p;
    ASSERT_TRUE(p.planStop(0.0, 10.0, 5.0));
    EXPECT_NEAR(p.finalPosition(), TrapezoidalProfile::stoppingDistance(10.0, 5.0), 1e-9);
    EXPECT_NEAR(p.finalPosition(), 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.sample(10.0).velocity, 0.0);
}

TEST(Trajectory, RejectsInvalidLimits) {
    TrapezoidalProfile p;
    EXPECT_FALSE(p.planPosition(0, 0, 1, 0.0, 1, 1));
    EXPECT_FALSE(p.planPosition(0, 0, 1, 1, -1, 1));
    EXPECT_FALSE(p.planPosition(0, 0, std::nan(""), 1, 1, 1));
    EXPECT_FALSE(p.planVelocity(0, 0, 1, 0, 1));
}
