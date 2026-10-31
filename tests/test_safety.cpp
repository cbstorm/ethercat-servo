// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "ethercat_motion/safety_manager.hpp"

using namespace ethercat_motion;

namespace {

struct Harness {
    SafetyConfig config;
    CommStatus comm;
    CycleStats timing;
    std::int64_t now = 1'000'000'000;

    Harness() {
        comm.link_up = true;
        comm.wc_complete = true;
        comm.all_operational = true;
        comm.reached_operational = true;
    }

    SafetyInputs inputs(std::size_t axes = 2) {
        SafetyInputs in;
        in.comm = &comm;
        in.timing = &timing;
        in.axis_count = axes;
        now += 1'000'000;
        in.now_ns = now;
        return in;
    }
};

}  // namespace

TEST(Safety, NoFaultsPermitMotion) {
    Harness h;
    SafetyManager s(h.config);
    s.evaluate(h.inputs());
    EXPECT_TRUE(s.motionPermitted(0));
    EXPECT_EQ(s.axisReaction(0), SafetyReaction::None);
    EXPECT_TRUE(s.status().communication_ready);
}

TEST(Safety, EmergencyStopLatchesUntilReleasedAndReset) {
    Harness h;
    SafetyManager s(h.config);
    s.requestEmergencyStop();
    s.evaluate(h.inputs());
    EXPECT_EQ(s.axisReaction(0), SafetyReaction::QuickStop);
    EXPECT_FALSE(s.motionPermitted(1));
    EXPECT_TRUE(s.status().latched_faults & safety_fault::kEmergencyStop);

    // Reset refused while the E-stop input is still active.
    s.requestReset();
    for (int i = 0; i < 3000; ++i) s.evaluate(h.inputs());
    EXPECT_TRUE(s.status().reset_blocked);
    EXPECT_FALSE(s.motionPermitted(0));

    s.releaseEmergencyStop();
    s.evaluate(h.inputs());
    EXPECT_FALSE(s.motionPermitted(0));  // still latched: release is not a reset
    s.requestReset();
    s.evaluate(h.inputs());
    EXPECT_TRUE(s.motionPermitted(0));
    EXPECT_EQ(s.status().reset_count, 1U);
}

TEST(Safety, CommunicationLossReaction) {
    Harness h;
    h.config.communication_loss_reaction = SafetyReaction::DisableVoltage;
    SafetyManager s(h.config);
    h.comm.consecutive_wc_errors = h.config.working_counter_error_cycles;
    h.comm.wc_complete = false;
    s.evaluate(h.inputs());
    EXPECT_TRUE(s.status().latched_faults & safety_fault::kWorkingCounter);
    EXPECT_EQ(s.axisReaction(0), SafetyReaction::DisableVoltage);
}

TEST(Safety, NoCommFaultsBeforeBusReachedOp) {
    Harness h;
    h.comm.reached_operational = false;
    h.comm.all_operational = false;
    h.comm.consecutive_wc_errors = 100;
    SafetyManager s(h.config);
    s.evaluate(h.inputs());
    EXPECT_EQ(s.status().latched_faults, 0U);
}

TEST(Safety, AxisFaultStopsOtherAxes) {
    Harness h;
    h.config.drive_fault_reaction = SafetyReaction::QuickStop;
    h.config.stop_all_on_axis_fault = true;
    SafetyManager s(h.config);
    SafetyInputs in = h.inputs();
    in.axes[0].drive_fault = true;
    s.evaluate(in);
    EXPECT_EQ(s.axisReaction(0), SafetyReaction::DisableVoltage);
    EXPECT_EQ(s.axisReaction(1), SafetyReaction::QuickStop);
    EXPECT_EQ(s.status().first_fault_axis, 0);
    EXPECT_EQ(s.status().axis_faults[0], safety_fault::kDriveFault);
}

TEST(Safety, AxisFaultIsolatedWhenConfigured) {
    Harness h;
    h.config.stop_all_on_axis_fault = false;
    SafetyManager s(h.config);
    SafetyInputs in = h.inputs();
    in.axes[0].following_error = true;
    s.evaluate(in);
    EXPECT_EQ(s.axisReaction(0), h.config.limit_reaction);
    EXPECT_EQ(s.axisReaction(1), SafetyReaction::None);
    EXPECT_TRUE(s.motionPermitted(1));
    EXPECT_FALSE(s.motionPermitted(0));
}

TEST(Safety, ResetWaitsForConditionToClear) {
    Harness h;
    SafetyManager s(h.config);
    SafetyInputs in = h.inputs();
    in.axes[0].drive_fault = true;
    s.evaluate(in);
    s.requestReset();
    in = h.inputs();
    in.axes[0].drive_fault = true;  // still faulted
    s.evaluate(in);
    EXPECT_FALSE(s.motionPermitted(0));
    // Drive reset completes a few cycles later: the pending reset applies.
    s.evaluate(h.inputs());
    EXPECT_TRUE(s.motionPermitted(0));
    EXPECT_FALSE(s.status().reset_blocked);
}

TEST(Safety, OverrunsAndHeartbeat) {
    Harness h;
    h.config.max_consecutive_overruns = 3;
    h.config.heartbeat_timeout_ms = 10;
    SafetyManager s(h.config);
    h.timing.consecutive_overruns = 3;
    s.evaluate(h.inputs());
    EXPECT_TRUE(s.status().latched_faults & safety_fault::kDeadlineOverrun);

    SafetyManager hb(h.config);
    h.timing.consecutive_overruns = 0;
    hb.heartbeat(h.now);
    for (int i = 0; i < 5; ++i) hb.evaluate(h.inputs());
    EXPECT_FALSE(hb.status().latched_faults & safety_fault::kHeartbeatTimeout);
    for (int i = 0; i < 10; ++i) hb.evaluate(h.inputs());
    EXPECT_TRUE(hb.status().latched_faults & safety_fault::kHeartbeatTimeout);
    EXPECT_EQ(hb.axisReaction(0), SafetyReaction::Stop);
}

TEST(Safety, DescribeFaults) {
    char buf[128];
    describeSafetyFaults(safety_fault::kEmergencyStop | safety_fault::kFollowingError, buf, sizeof(buf));
    EXPECT_STREQ(buf, "emergency_stop,following_error");
}
