// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "ethercat_motion/cia402_drive.hpp"

using namespace ethercat_motion;

namespace {

// Minimal drive-side model: applies the controlword to a power state machine.
struct FakeDrive {
    DriveState state = DriveState::SwitchOnDisabled;
    std::uint16_t last_cw = 0;
    bool ignore_commands = false;

    std::uint16_t statusword() const {
        switch (state) {
            case DriveState::NotReadyToSwitchOn: return 0x0000;
            case DriveState::SwitchOnDisabled: return 0x0240;
            case DriveState::ReadyToSwitchOn: return 0x0231;
            case DriveState::SwitchedOn: return 0x0233;
            case DriveState::OperationEnabled: return 0x0237;
            case DriveState::QuickStopActive: return 0x0217;
            case DriveState::FaultReactionActive: return 0x020F;
            case DriveState::Fault: return 0x0208;
            default: return 0;
        }
    }

    void apply(std::uint16_t cw) {
        const bool reset_edge = (cw & 0x80) && !(last_cw & 0x80);
        last_cw = cw;
        if (ignore_commands) return;
        const bool shutdown = (cw & 0x87) == 0x06;
        const bool switch_on = (cw & 0x8F) == 0x07;
        const bool enable = (cw & 0x8F) == 0x0F;
        const bool dv = (cw & 0x82) == 0x00;
        const bool qs = (cw & 0x86) == 0x02;
        switch (state) {
            case DriveState::SwitchOnDisabled:
                if (shutdown) state = DriveState::ReadyToSwitchOn;
                break;
            case DriveState::ReadyToSwitchOn:
                if (dv || qs) state = DriveState::SwitchOnDisabled;
                else if (switch_on) state = DriveState::SwitchedOn;
                break;
            case DriveState::SwitchedOn:
                if (dv || qs) state = DriveState::SwitchOnDisabled;
                else if (shutdown) state = DriveState::ReadyToSwitchOn;
                else if (enable) state = DriveState::OperationEnabled;
                break;
            case DriveState::OperationEnabled:
                if (dv) state = DriveState::SwitchOnDisabled;
                else if (qs) state = DriveState::QuickStopActive;
                else if (shutdown) state = DriveState::ReadyToSwitchOn;
                else if (switch_on) state = DriveState::SwitchedOn;
                break;
            case DriveState::QuickStopActive:
                if (dv) state = DriveState::SwitchOnDisabled;
                break;
            case DriveState::Fault:
                if (reset_edge) state = DriveState::SwitchOnDisabled;
                break;
            default:
                break;
        }
    }
};

/// Runs drive + controller for `cycles`, recording controlwords.
std::vector<std::uint16_t> run(Cia402Drive& ctrl, FakeDrive& drive, int cycles) {
    std::vector<std::uint16_t> cws;
    for (int i = 0; i < cycles; ++i) {
        ctrl.update(drive.statusword());
        cws.push_back(ctrl.controlword());
        drive.apply(ctrl.controlword());
    }
    return cws;
}

}  // namespace

TEST(Cia402, DecodeAllStates) {
    using cia402::decodeState;
    EXPECT_EQ(decodeState(0x0000), DriveState::NotReadyToSwitchOn);
    EXPECT_EQ(decodeState(0x0040), DriveState::SwitchOnDisabled);
    EXPECT_EQ(decodeState(0x0021), DriveState::ReadyToSwitchOn);
    EXPECT_EQ(decodeState(0x0023), DriveState::SwitchedOn);
    EXPECT_EQ(decodeState(0x0027), DriveState::OperationEnabled);
    EXPECT_EQ(decodeState(0x0007), DriveState::QuickStopActive);
    EXPECT_EQ(decodeState(0x000F), DriveState::FaultReactionActive);
    EXPECT_EQ(decodeState(0x0008), DriveState::Fault);
    // Mode specific / voltage / remote bits must not change the decoding.
    EXPECT_EQ(decodeState(0x1637), DriveState::OperationEnabled);
    EXPECT_EQ(decodeState(0x0250), DriveState::SwitchOnDisabled);
    EXPECT_EQ(decodeState(0x0218), DriveState::Fault);
}

TEST(Cia402, EnableSequenceIsStateAware) {
    Cia402Drive ctrl;
    FakeDrive drive;
    ctrl.requestEnable();
    const auto cws = run(ctrl, drive, 6);
    ASSERT_GE(cws.size(), 3U);
    EXPECT_EQ(cws[0], cia402::kCwShutdown);
    EXPECT_EQ(cws[1], cia402::kCwSwitchOn);
    EXPECT_EQ(cws[2], cia402::kCwEnableOperation);
    EXPECT_EQ(drive.state, DriveState::OperationEnabled);
    EXPECT_TRUE(ctrl.requestSatisfied());
    EXPECT_EQ(cws.back(), cia402::kCwEnableOperation);  // keeps holding Enable Operation
}

TEST(Cia402, NoCommandWhileStateUnknown) {
    Cia402Drive ctrl;
    ctrl.requestEnable();
    ctrl.update(0x0000);  // not ready to switch on (drive booting)
    EXPECT_EQ(ctrl.controlword(), cia402::kCwDisableVoltage);
}

TEST(Cia402, ControlledDisable) {
    Cia402Drive ctrl;
    FakeDrive drive;
    ctrl.requestEnable();
    run(ctrl, drive, 5);
    ctrl.requestDisable();
    const auto cws = run(ctrl, drive, 4);
    EXPECT_EQ(cws[0], cia402::kCwDisableOperation);  // OE -> Switched On
    EXPECT_EQ(cws[1], cia402::kCwDisableVoltage);    // Switched On -> Switch On Disabled
    EXPECT_EQ(drive.state, DriveState::SwitchOnDisabled);
}

TEST(Cia402, DisableVoltageIsImmediate) {
    Cia402Drive ctrl;
    FakeDrive drive;
    ctrl.requestEnable();
    run(ctrl, drive, 5);
    ctrl.requestDisableVoltage();
    const auto cws = run(ctrl, drive, 2);
    EXPECT_EQ(cws[0], cia402::kCwDisableVoltage);
    EXPECT_EQ(drive.state, DriveState::SwitchOnDisabled);
}

TEST(Cia402, QuickStopAndRecovery) {
    Cia402Drive ctrl;
    FakeDrive drive;
    ctrl.requestEnable();
    run(ctrl, drive, 5);
    ctrl.requestQuickStop();
    auto cws = run(ctrl, drive, 3);
    EXPECT_EQ(cws[0], cia402::kCwQuickStop);
    EXPECT_EQ(drive.state, DriveState::QuickStopActive);
    EXPECT_EQ(cws.back(), cia402::kCwQuickStop);  // held while requested
    // Re-enable goes through Switch On Disabled, not transition 16.
    ctrl.requestEnable();
    cws = run(ctrl, drive, 6);
    EXPECT_EQ(cws[0], cia402::kCwDisableVoltage);
    EXPECT_EQ(drive.state, DriveState::OperationEnabled);
}

TEST(Cia402, FaultResetUsesRisingEdgeAndDoesNotReenable) {
    Cia402Config cfg;
    cfg.fault_reset_pulse_cycles = 3;
    Cia402Drive ctrl(cfg);
    FakeDrive drive;
    ctrl.requestEnable();
    run(ctrl, drive, 5);
    drive.state = DriveState::Fault;
    run(ctrl, drive, 2);
    EXPECT_TRUE(ctrl.isFault());
    ctrl.requestFaultReset();
    const auto cws = run(ctrl, drive, 12);
    // bit 7 low first, then high (rising edge)
    EXPECT_EQ(cws[0] & 0x80, 0);
    bool saw_pulse = false;
    for (auto cw : cws) saw_pulse = saw_pulse || (cw & 0x80);
    EXPECT_TRUE(saw_pulse);
    EXPECT_EQ(drive.state, DriveState::SwitchOnDisabled);
    EXPECT_FALSE(ctrl.isFault());
    EXPECT_EQ(ctrl.request(), DriveRequest::Disable);
    EXPECT_FALSE(ctrl.faultResetFailed());
}

TEST(Cia402, FaultResetOnHealthyDriveIsNoOp) {
    Cia402Drive ctrl;
    FakeDrive drive;
    ctrl.requestEnable();
    run(ctrl, drive, 5);
    ctrl.requestFaultReset();
    run(ctrl, drive, 5);
    EXPECT_EQ(drive.state, DriveState::OperationEnabled);
    EXPECT_EQ(ctrl.request(), DriveRequest::Enable);
}

TEST(Cia402, FaultResetFailureIsReported) {
    Cia402Config cfg;
    cfg.fault_reset_timeout_cycles = 20;
    Cia402Drive ctrl(cfg);
    FakeDrive drive;
    drive.state = DriveState::Fault;
    drive.ignore_commands = true;  // fault persists
    ctrl.update(drive.statusword());
    ctrl.requestFaultReset();
    run(ctrl, drive, 60);
    EXPECT_TRUE(ctrl.faultResetFailed());
}

TEST(Cia402, TransitionTimeoutFallsBackToDisable) {
    Cia402Config cfg;
    cfg.transition_timeout_cycles = 10;
    Cia402Drive ctrl(cfg);
    FakeDrive drive;
    drive.ignore_commands = true;  // e.g. no DC bus voltage
    ctrl.requestEnable();
    run(ctrl, drive, 20);
    EXPECT_TRUE(ctrl.transitionTimedOut());
    EXPECT_EQ(ctrl.request(), DriveRequest::Disable);
    EXPECT_EQ(ctrl.controlword(), cia402::kCwDisableVoltage);
}

TEST(Cia402, NoTimeoutWhileWaitingInFault) {
    Cia402Config cfg;
    cfg.transition_timeout_cycles = 5;
    Cia402Drive ctrl(cfg);
    FakeDrive drive;
    drive.state = DriveState::Fault;
    ctrl.requestEnable();
    run(ctrl, drive, 20);
    EXPECT_FALSE(ctrl.transitionTimedOut());
    EXPECT_EQ(ctrl.controlword(), cia402::kCwDisableVoltage);
}
