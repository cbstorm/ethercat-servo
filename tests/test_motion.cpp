// SPDX-License-Identifier: MIT
//
// End-to-end tests on the simulation backend: configuration -> EtherCAT
// master -> PDO process image -> mock CiA 402 drives -> motion control ->
// safety. Cycles are stepped deterministically with Controller::step().
#include <atomic>
#include <cmath>

#include <gtest/gtest.h>

#include "test_helpers.hpp"

using namespace ethercat_motion;
using namespace ethercat_motion::test;
using namespace std::chrono_literals;

class MotionTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<Controller>(simulationConfig());
        bringUp(*controller_);
    }

    Controller& c() { return *controller_; }
    void cycles(std::uint32_t n) { ASSERT_TRUE(controller_->step(n).isOk()); }
    SimulationControl& sim() { return *controller_->simulation(); }

    /// Steps until the axis' last motion command finished; returns its result.
    ErrorCode runMotion(const std::string& axis, int max_cycles = 20000) {
        const std::uint64_t id = c().axis(axis).lastCommandId();
        EXPECT_TRUE(stepUntil(
            c(),
            [&] {
                const AxisStatus s = c().axis(axis).status();
                return (s.motion_command.id == id && s.motion_command.state != CommandState::Pending) ||
                       s.last_rejected.id == id;
            },
            max_cycles));
        const AxisStatus s = c().axis(axis).status();
        return s.last_rejected.id == id ? s.last_rejected.result : s.motion_command.result;
    }

    std::unique_ptr<Controller> controller_;
};

TEST_F(MotionTest, BringUpConfirmsModesIncludingSdoPath) {
    const AxisStatus x = c().axis("axis_x").status();
    const AxisStatus y = c().axis("axis_y").status();
    EXPECT_EQ(x.mode_display, OperationMode::CSP);
    EXPECT_EQ(y.mode_display, OperationMode::CSV);  // switched by SDO (0x6060 not in ESI default PDO)
    EXPECT_FALSE(x.enabled);
    EXPECT_TRUE(c().ethercatStatus().comm.wc_complete);
    EXPECT_EQ(c().ethercatStatus().comm.working_counter, 6U);
}

TEST_F(MotionTest, EnableUsesCia402Sequence) {
    enableAxis(c(), "axis_x");
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_EQ(s.drive_state, DriveState::OperationEnabled);
    EXPECT_EQ(s.controlword & 0x0F, 0x0F);
    EXPECT_NEAR(s.setpoint_position, s.position, 1e-9);  // no jump on enable
}

TEST_F(MotionTest, MoveAbsoluteRespectsLimits) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(10.0).isOk());
    double max_vel = 0.0;
    const std::uint64_t id = c().axis("axis_x").lastCommandId();
    ASSERT_TRUE(stepUntil(c(), [&] {
        const AxisStatus s = c().axis("axis_x").status();
        max_vel = std::max(max_vel, std::fabs(s.setpoint_velocity));
        return s.motion_command.id == id && s.motion_command.state != CommandState::Pending;
    }));
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_EQ(s.motion_command.state, CommandState::Completed);
    EXPECT_NEAR(s.position, 10.0, c().axis("axis_x").config().motion.in_position_window);
    EXPECT_LE(max_vel, c().axis("axis_x").config().motion.velocity + 1e-9);
    EXPECT_TRUE(s.motion_done);
    EXPECT_EQ(s.safety_faults, 0U);
}

TEST_F(MotionTest, RelativeMovesChainFromTarget) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveRelative(5.0).isOk());
    cycles(5);
    ASSERT_TRUE(c().axis("axis_x").moveRelative(5.0).isOk());  // relative to the active target
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_x").getActualPosition(), 10.0, 0.01);
}

TEST_F(MotionTest, SoftLimitsRejectAndStopJog) {
    enableAxis(c(), "axis_x");
    const Status rejected = c().axis("axis_x").moveAbsolute(150.0);
    EXPECT_EQ(rejected.code(), ErrorCode::LimitViolation);
    EXPECT_EQ(c().axis("axis_x").moveVelocity(500.0).code(), ErrorCode::LimitViolation);

    ASSERT_TRUE(c().axis("axis_x").moveVelocity(40.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);  // velocity reached
    // Keep stepping: the jog must decelerate to rest exactly at the limit.
    ASSERT_TRUE(stepUntil(c(), [&] { return !c().axis("axis_x").status().moving; }, 20000));
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_NEAR(s.position, 100.0, 0.05);
    EXPECT_LE(s.position, 100.0 + c().axis("axis_x").config().limits.position_tolerance);
    EXPECT_TRUE(s.at_positive_limit);
    EXPECT_EQ(s.motion_command.result, ErrorCode::Ok);  // completed when the speed was reached
    EXPECT_EQ(s.safety_faults, 0U);  // stopping at the soft limit is not a fault
    // Moving further out is rejected, moving back in is allowed.
    ASSERT_TRUE(c().axis("axis_x").moveVelocity(5.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::LimitViolation);
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(50.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
}

TEST_F(MotionTest, StopDuringMove) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(80.0).isOk());
    cycles(500);
    const Status stop_status = c().axis("axis_x").stop();
    ASSERT_TRUE(stop_status.isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_LT(s.position, 80.0);
    EXPECT_NEAR(s.setpoint_velocity, 0.0, 1e-9);
    EXPECT_TRUE(s.enabled);
}

TEST_F(MotionTest, MotionRejectedWhenDisabled) {
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(5.0).isOk());  // passes validation
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::InvalidState);   // cyclic task rejects
}

TEST_F(MotionTest, DisableDuringMotionStopsFirst) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveVelocity(30.0).isOk());
    cycles(800);
    ASSERT_TRUE(c().axis("axis_x").disable().isOk());
    cycles(2);
    EXPECT_EQ(c().axis("axis_x").status().motion_state, MotionState::Disabling);
    EXPECT_TRUE(c().axis("axis_x").status().enabled);  // still powered while ramping down
    ASSERT_TRUE(stepUntil(c(), [&] {
        return c().axis("axis_x").status().power_command.state != CommandState::Pending;
    }, 5000));
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_EQ(s.drive_state, DriveState::SwitchOnDisabled);
    EXPECT_EQ(s.power_command.state, CommandState::Completed);
    EXPECT_LT(std::fabs(s.velocity), 0.5);
}

TEST_F(MotionTest, ProfilePositionModeViaSdoParameters) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").setMode(OperationMode::PP).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_EQ(c().axis("axis_x").status().mode_display, OperationMode::PP);
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(-20.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_x").getActualPosition(), -20.0, 0.01);
}

TEST_F(MotionTest, ProfileVelocityMode) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").setMode(OperationMode::PV).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    ASSERT_TRUE(c().axis("axis_x").moveVelocity(-10.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_x").getActualVelocity(), -10.0, 0.6);
    ASSERT_TRUE(c().axis("axis_x").stop().isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_x").getActualVelocity(), 0.0, 0.6);
}

TEST_F(MotionTest, HomingRestoresMode) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(4.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    ASSERT_TRUE(c().axis("axis_x").home().isOk());
    EXPECT_EQ(runMotion("axis_x", 30000), ErrorCode::Ok);
    const AxisStatus s = c().axis("axis_x").status();
    EXPECT_TRUE(s.homed);
    EXPECT_EQ(s.mode_display, OperationMode::CSP);
    EXPECT_NEAR(s.position, 0.0, 0.01);
    // Normal motion continues after homing without a position jump.
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(1.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
}

TEST_F(MotionTest, SoftwarePositionRedefinition) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").setPosition(42.0).isOk());
    cycles(3);
    EXPECT_NEAR(c().axis("axis_x").getActualPosition(), 42.0, 1e-3);
    EXPECT_TRUE(c().axis("axis_x").status().homed);
    ASSERT_TRUE(c().axis("axis_x").moveRelative(-2.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_x").getActualPosition(), 40.0, 0.01);
}

TEST_F(MotionTest, CsvWithExternalPositionLoop) {
    enableAxis(c(), "axis_y");
    ASSERT_TRUE(c().axis("axis_y").moveAbsolute(90.0).isOk());
    EXPECT_EQ(runMotion("axis_y"), ErrorCode::Ok);
    EXPECT_NEAR(c().axis("axis_y").getActualPosition(), 90.0, 0.2);
    // CST/torque commands are not valid in CSV.
    EXPECT_EQ(c().axis("axis_y").setTorque(10.0).code(), ErrorCode::Ok);  // passes validation
    EXPECT_EQ(runMotion("axis_y"), ErrorCode::NotSupported);
}

TEST_F(MotionTest, DriveFaultStopsOtherAxesAndRecovers) {
    enableAxis(c(), "axis_x");
    enableAxis(c(), "axis_y");
    sim().injectDriveFault(0, 0x2310);
    ASSERT_TRUE(stepUntil(c(), [&] {
        return c().axis("axis_x").status().fault && !c().axis("axis_y").status().enabled;
    }, 500));
    EXPECT_EQ(c().axis("axis_x").status().error_code, 0x2310);
    const SafetyStatus safety = c().safetyStatus();
    EXPECT_TRUE(safety.latched_faults & safety_fault::kDriveFault);
    EXPECT_EQ(safety.first_fault_axis, 0);

    // Enabling is refused while faults are latched.
    ASSERT_TRUE(c().axis("axis_y").enable().isOk());
    cycles(3);
    EXPECT_EQ(c().axis("axis_y").status().last_rejected.result, ErrorCode::SafetyFault);

    ASSERT_TRUE(c().resetFaults().isOk());
    ASSERT_TRUE(stepUntil(c(), [&] { return c().safetyStatus().latched_faults == 0; }, 3000));
    EXPECT_FALSE(c().axis("axis_x").status().fault);
    EXPECT_FALSE(c().axis("axis_x").status().enabled);  // never re-enabled automatically
    enableAxis(c(), "axis_x");
    enableAxis(c(), "axis_y");
}

TEST_F(MotionTest, CommunicationLossAndRecovery) {
    enableAxis(c(), "axis_x");
    enableAxis(c(), "axis_y");
    sim().setSlaveOnline(1, false);
    ASSERT_TRUE(stepUntil(c(), [&] {
        return (c().safetyStatus().latched_faults & safety_fault::kWorkingCounter) != 0;
    }, 100));
    EXPECT_FALSE(c().axis("axis_y").status().comm_ok);
    cycles(20);
    EXPECT_EQ(sim().driveSnapshot(0).controlword & 0x0F, 0);  // axis_x got Disable Voltage

    sim().setSlaveOnline(1, true);
    ASSERT_TRUE(stepUntil(c(), [&] { return c().axis("axis_y").status().comm_ok; }, 200));
    ASSERT_TRUE(stepUntil(c(), [&] { return c().axis("axis_y").status().fault; }, 200));
    EXPECT_EQ(c().axis("axis_y").status().error_code, 0x8130);  // SM watchdog fault of the drive
    ASSERT_TRUE(c().resetFaults().isOk());
    ASSERT_TRUE(stepUntil(c(), [&] { return c().safetyStatus().latched_faults == 0; }, 3000));
    enableAxis(c(), "axis_y");
}

TEST_F(MotionTest, FollowingErrorIsDetected) {
    enableAxis(c(), "axis_x");
    sim().setPositionDisturbance(0, 10000.0);  // 5 mm offset between command and actual
    ASSERT_TRUE(stepUntil(c(), [&] {
        return (c().safetyStatus().latched_faults & safety_fault::kFollowingError) != 0;
    }, 200));
    ASSERT_TRUE(stepUntil(c(), [&] { return !c().axis("axis_x").status().enabled; }, 200));
    sim().setPositionDisturbance(0, 0.0);
    ASSERT_TRUE(c().resetFaults().isOk());
    ASSERT_TRUE(stepUntil(c(), [&] { return c().safetyStatus().latched_faults == 0; }, 3000));
}

TEST_F(MotionTest, EmergencyStop) {
    enableAxis(c(), "axis_x");
    enableAxis(c(), "axis_y");
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(80.0).isOk());
    cycles(200);
    c().emergencyStop();
    ASSERT_TRUE(stepUntil(c(), [&] {
        return !c().axis("axis_x").status().enabled && !c().axis("axis_y").status().enabled;
    }, 500));
    EXPECT_EQ(c().axis("axis_x").status().motion_command.result, ErrorCode::SafetyFault);
    ASSERT_TRUE(c().resetFaults().isOk());
    cycles(10);
    EXPECT_NE(c().safetyStatus().latched_faults, 0U);  // input still active
    c().releaseEmergencyStop();
    ASSERT_TRUE(c().resetFaults().isOk());
    ASSERT_TRUE(stepUntil(c(), [&] { return c().safetyStatus().latched_faults == 0; }, 100));
    enableAxis(c(), "axis_x");
}

TEST_F(MotionTest, QuickStopCommand) {
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveVelocity(20.0).isOk());
    cycles(300);
    ASSERT_TRUE(c().axis("axis_x").quickStop().isOk());
    ASSERT_TRUE(stepUntil(c(), [&] { return !c().axis("axis_x").status().enabled; }, 500));
    EXPECT_EQ(c().axis("axis_x").status().power_command.state, CommandState::Completed);
    enableAxis(c(), "axis_x");  // re-enable after quick stop
}

TEST_F(MotionTest, CommandResultsAreReported) {
    std::atomic<int> completed{0};
    c().setCommandResultCallback([&](const CommandResult& r) {
        if (r.code == ErrorCode::Ok) ++completed;
    });
    enableAxis(c(), "axis_x");
    ASSERT_TRUE(c().axis("axis_x").moveAbsolute(1.0).isOk());
    EXPECT_EQ(runMotion("axis_x"), ErrorCode::Ok);
    cycles(1);
    EXPECT_GE(completed.load(), 2);
}

TEST(MotionThreaded, RealCyclicThread) {
    Controller controller(simulationConfig());
    ASSERT_TRUE(controller.initialize().isOk());
    ASSERT_TRUE(controller.start().isOk());
    Axis& x = controller.axis("axis_x");
    // Wait for the drive to finish booting and the mode to be confirmed.
    for (int i = 0; i < 500 && x.status().drive_state != DriveState::SwitchOnDisabled; ++i) {
        std::this_thread::sleep_for(1ms);
    }
    ASSERT_TRUE(x.enable().isOk());
    ASSERT_TRUE(x.wait(2s).isOk());
    ASSERT_TRUE(x.moveAbsolute(5.0).isOk());
    const Status moved = x.wait(5s);
    EXPECT_TRUE(moved.isOk()) << moved.toString();
    EXPECT_NEAR(x.getActualPosition(), 5.0, 0.01);
    const MasterStatus m = controller.ethercatStatus();
    EXPECT_GT(m.timing.cycles, 100U);
    EXPECT_GT(m.timing.period_max_ns, 0);
    ASSERT_TRUE(controller.stop().isOk());
    EXPECT_FALSE(x.status().enabled);
}

TEST(MasterStartup, IdentityMismatchFailsSafely) {
    SimulatedSlaveSpec spec;
    spec.vendor_id = 0x0A79;
    spec.product_code = 0x5000;  // DS2-EC on the bus, ISL-EC expected
    spec.revision = 1;
    EthercatMaster master(MasterConfig{}, std::make_unique<SimulatedBackend>(std::vector<SimulatedSlaveSpec>{spec}));
    ASSERT_TRUE(master.initialize().isOk());
    auto cfg = simulationConfig();
    ASSERT_TRUE(master.addSlave(toSlaveDeviceConfig(cfg.axes[0])).isOk());
    const Status s = master.configure();
    EXPECT_EQ(s.code(), ErrorCode::DeviceMismatch);
    EXPECT_NE(s.message().find("product 0x00005000"), std::string::npos) << s.message();
    EXPECT_EQ(master.activate().code(), ErrorCode::InvalidState);  // never activated
}

TEST(MasterStartup, RevisionMismatchFailsSafely) {
    SimulatedSlaveSpec spec;
    spec.vendor_id = 0x0A79;
    spec.product_code = 0x5800;
    spec.revision = 7;
    EthercatMaster master(MasterConfig{}, std::make_unique<SimulatedBackend>(std::vector<SimulatedSlaveSpec>{spec}));
    ASSERT_TRUE(master.initialize().isOk());
    auto cfg = simulationConfig();
    ASSERT_TRUE(master.addSlave(toSlaveDeviceConfig(cfg.axes[0])).isOk());
    EXPECT_EQ(master.configure().code(), ErrorCode::DeviceMismatch);
}

TEST(MasterStartup, MissingSlaveFails) {
    EthercatMaster master(MasterConfig{}, std::make_unique<SimulatedBackend>(std::vector<SimulatedSlaveSpec>{}));
    ASSERT_TRUE(master.initialize().isOk());
    auto cfg = simulationConfig();
    ASSERT_TRUE(master.addSlave(toSlaveDeviceConfig(cfg.axes[0])).isOk());
    EXPECT_EQ(master.configure().code(), ErrorCode::NotFound);
}

TEST(MasterStartup, SupportedModesReadFromDrive) {
    SimulatedSlaveSpec spec;
    spec.vendor_id = 0x0A79;
    spec.product_code = 0x5800;
    spec.revision = 1;
    spec.supported_modes = 0x1DD;  // like the ISL-EC ESI: no CST
    EthercatMaster master(MasterConfig{}, std::make_unique<SimulatedBackend>(std::vector<SimulatedSlaveSpec>{spec}));
    ASSERT_TRUE(master.initialize().isOk());
    auto cfg = simulationConfig();
    ASSERT_TRUE(master.addSlave(toSlaveDeviceConfig(cfg.axes[0])).isOk());
    ASSERT_TRUE(master.configure().isOk());
    auto modes = master.uploadSdo(0, 0x6502, 0, 4);
    ASSERT_TRUE(modes.isOk()) << modes.status().toString();
    EXPECT_EQ(modes.value(), 0x1DDU);
    EXPECT_FALSE(master.uploadSdo(0, 0x2FFF, 0, 4).isOk());  // object does not exist
}

TEST_F(MotionTest, SynchronizedMultiAxisMove) {
    enableAxis(c(), "axis_x");
    enableAxis(c(), "axis_y");
    MultiAxisCoordinator coordinator(c().motion());
    auto plan = coordinator.moveSynchronized({{"axis_x", 20.0}, {"axis_y", 180.0}});
    ASSERT_TRUE(plan.isOk()) << plan.status().toString();
    ASSERT_EQ(plan->entries.size(), 2U);
    // Respect each axis' own limits.
    EXPECT_LE(plan->entries[0].velocity, c().axis("axis_x").config().motion.velocity + 1e-9);
    EXPECT_LE(plan->entries[1].velocity, c().axis("axis_y").config().motion.velocity + 1e-9);

    // Mid-move, both axes have covered the same fraction of their distance.
    cycles(static_cast<std::uint32_t>(plan->duration_s * 500.0));
    const AxisStatus x = c().axis("axis_x").status();
    const AxisStatus y = c().axis("axis_y").status();
    const double px = (x.setpoint_position - plan->entries[0].start) / (20.0 - plan->entries[0].start);
    const double py = (y.setpoint_position - plan->entries[1].start) / (180.0 - plan->entries[1].start);
    EXPECT_GT(px, 0.2);
    EXPECT_LT(px, 0.8);
    EXPECT_NEAR(px, py, 1e-9);

    ASSERT_TRUE(stepUntil(c(), [&] {
        return c().axis("axis_x").status().motion_done && c().axis("axis_y").status().motion_done;
    }));
    EXPECT_NEAR(c().axis("axis_x").getActualPosition(), 20.0, 0.01);
    EXPECT_NEAR(c().axis("axis_y").getActualPosition(), 180.0, 0.2);
    EXPECT_EQ(c().axis("axis_x").status().motion_command.state, CommandState::Completed);
    EXPECT_EQ(c().axis("axis_y").status().motion_command.state, CommandState::Completed);
}

TEST_F(MotionTest, SynchronizedMoveValidation) {
    MultiAxisCoordinator coordinator(c().motion());
    EXPECT_EQ(coordinator.plan({{"nope", 1.0}}).status().code(), ErrorCode::NotFound);
    EXPECT_EQ(coordinator.plan({{"axis_x", 500.0}}).status().code(), ErrorCode::LimitViolation);
    EXPECT_EQ(coordinator.plan({{"axis_x", 1.0}, {"axis_x", 2.0}}).status().code(), ErrorCode::InvalidArgument);
    EXPECT_EQ(coordinator.plan({{"axis_x", 1.0}}, 1.5).status().code(), ErrorCode::InvalidArgument);
}
