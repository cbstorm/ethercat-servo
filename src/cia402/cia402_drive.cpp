// SPDX-License-Identifier: MIT
#include "ethercat_motion/cia402_drive.hpp"

namespace ethercat_motion {

namespace cia402 {

DriveState decodeState(std::uint16_t sw) noexcept {
    // Masks from CiA 402 / IEC 61800-7-201 table "statusword state coding".
    if ((sw & 0x004F) == 0x0000) return DriveState::NotReadyToSwitchOn;
    if ((sw & 0x004F) == 0x0040) return DriveState::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0021) return DriveState::ReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0023) return DriveState::SwitchedOn;
    if ((sw & 0x006F) == 0x0027) return DriveState::OperationEnabled;
    if ((sw & 0x006F) == 0x0007) return DriveState::QuickStopActive;
    if ((sw & 0x004F) == 0x000F) return DriveState::FaultReactionActive;
    if ((sw & 0x004F) == 0x0008) return DriveState::Fault;
    return DriveState::Unknown;
}

std::uint32_t supportedModeBit(OperationMode mode) noexcept {
    switch (mode) {
        case OperationMode::PP: return 1U << 0;
        case OperationMode::PV: return 1U << 2;
        case OperationMode::Homing: return 1U << 5;
        case OperationMode::CSP: return 1U << 7;
        case OperationMode::CSV: return 1U << 8;
        case OperationMode::CST: return 1U << 9;
        case OperationMode::None: break;
    }
    return 0;
}

}  // namespace cia402

Cia402Drive::Cia402Drive(const Cia402Config& config) noexcept : config_(config) {}

void Cia402Drive::clearRequestFlags() noexcept {
    cycles_in_state_ = 0;
    timed_out_ = false;
}

void Cia402Drive::requestEnable() noexcept {
    if (request_ != DriveRequest::Enable) {
        clearRequestFlags();
    }
    request_ = DriveRequest::Enable;
}

void Cia402Drive::requestDisable() noexcept {
    if (request_ != DriveRequest::Disable) {
        clearRequestFlags();
    }
    request_ = DriveRequest::Disable;
}

void Cia402Drive::requestQuickStop() noexcept {
    if (request_ != DriveRequest::QuickStop) {
        clearRequestFlags();
    }
    request_ = DriveRequest::QuickStop;
}

void Cia402Drive::requestDisableVoltage() noexcept {
    if (request_ != DriveRequest::DisableVoltage) {
        clearRequestFlags();
    }
    request_ = DriveRequest::DisableVoltage;
}

void Cia402Drive::requestFaultReset() noexcept {
    // Only meaningful for a faulted drive: a healthy (possibly enabled)
    // drive must not be disturbed by a global "reset faults".
    if (!isFault() && state_ != DriveState::Unknown) {
        return;
    }
    // A fault reset always ends with the drive disabled; re-enabling is an
    // explicit, separate operator action.
    request_ = DriveRequest::Disable;
    clearRequestFlags();
    reset_phase_ = ResetPhase::Low;
    reset_counter_ = 0;
    reset_failed_ = false;
}

bool Cia402Drive::requestSatisfied() const noexcept {
    switch (request_) {
        case DriveRequest::Enable:
            return state_ == DriveState::OperationEnabled;
        case DriveRequest::Disable:
        case DriveRequest::DisableVoltage:
            return state_ == DriveState::SwitchOnDisabled || state_ == DriveState::NotReadyToSwitchOn ||
                   state_ == DriveState::Fault || state_ == DriveState::FaultReactionActive;
        case DriveRequest::QuickStop:
            return state_ == DriveState::QuickStopActive || state_ == DriveState::SwitchOnDisabled ||
                   state_ == DriveState::NotReadyToSwitchOn || state_ == DriveState::Fault ||
                   state_ == DriveState::FaultReactionActive;
    }
    return false;
}

std::uint16_t Cia402Drive::commandForState() noexcept {
    using namespace cia402;
    if (request_ == DriveRequest::DisableVoltage) {
        return kCwDisableVoltage;
    }
    switch (state_) {
        case DriveState::Unknown:
        case DriveState::NotReadyToSwitchOn:
        case DriveState::FaultReactionActive:
        case DriveState::Fault:
            return kCwDisableVoltage;
        case DriveState::SwitchOnDisabled:
            return request_ == DriveRequest::Enable ? kCwShutdown : kCwDisableVoltage;  // transition 2
        case DriveState::ReadyToSwitchOn:
            return request_ == DriveRequest::Enable ? kCwSwitchOn : kCwDisableVoltage;  // 3 / 7
        case DriveState::SwitchedOn:
            return request_ == DriveRequest::Enable ? kCwEnableOperation : kCwDisableVoltage;  // 4 / 10
        case DriveState::OperationEnabled:
            switch (request_) {
                case DriveRequest::Enable:
                    return kCwEnableOperation;
                case DriveRequest::Disable:
                    return kCwDisableOperation;  // transition 5, then 10 from Switched On
                case DriveRequest::QuickStop:
                    return kCwQuickStop;  // transition 11
                case DriveRequest::DisableVoltage:
                    return kCwDisableVoltage;  // transition 9
            }
            break;
        case DriveState::QuickStopActive:
            // Hold quick stop while requested; otherwise leave via Disable
            // Voltage (transition 12). Re-enabling always goes through
            // Switch On Disabled rather than transition 16.
            return request_ == DriveRequest::QuickStop ? kCwQuickStop : kCwDisableVoltage;
    }
    return kCwDisableVoltage;
}

void Cia402Drive::recompute() noexcept {
    if (reset_phase_ == ResetPhase::Idle) {
        controlword_ = commandForState();
    }
}

void Cia402Drive::update(std::uint16_t statusword) noexcept {
    previous_state_ = state_;
    state_ = cia402::decodeState(statusword);
    if (state_ != previous_state_) {
        cycles_in_state_ = 0;
    } else if (cycles_in_state_ < 0xFFFFFFFFU) {
        ++cycles_in_state_;
    }

    if (reset_phase_ != ResetPhase::Idle) {
        ++reset_counter_;
        switch (reset_phase_) {
            case ResetPhase::Low:
                // Ensure bit 7 is low before the rising edge.
                controlword_ = cia402::kCwDisableVoltage;
                if (!isFault() && state_ != DriveState::Unknown) {
                    reset_phase_ = ResetPhase::Idle;  // nothing to reset
                } else if (state_ == DriveState::Fault && reset_counter_ >= 2) {
                    reset_phase_ = ResetPhase::Pulse;
                    reset_counter_ = 0;
                } else if (reset_counter_ >= config_.fault_reset_timeout_cycles) {
                    reset_failed_ = true;
                    reset_phase_ = ResetPhase::Idle;
                }
                break;
            case ResetPhase::Pulse:
                controlword_ = cia402::kCwFaultReset;
                if (reset_counter_ >= config_.fault_reset_pulse_cycles) {
                    reset_phase_ = ResetPhase::Wait;
                    reset_counter_ = 0;
                }
                break;
            case ResetPhase::Wait:
                controlword_ = cia402::kCwDisableVoltage;
                if (!isFault()) {
                    reset_phase_ = ResetPhase::Idle;
                } else if (reset_counter_ >= config_.fault_reset_timeout_cycles) {
                    reset_failed_ = true;
                    reset_phase_ = ResetPhase::Idle;
                }
                break;
            case ResetPhase::Idle:
                break;
        }
        if (reset_phase_ != ResetPhase::Idle) {
            return;
        }
    }

    controlword_ = commandForState();

    // Transition supervision. A drive sitting in Fault is waiting for an
    // explicit reset, which is not a transition in progress.
    if (!requestSatisfied() && !isFault() && cycles_in_state_ >= config_.transition_timeout_cycles) {
        timed_out_ = true;
        if (request_ == DriveRequest::Enable) {
            request_ = DriveRequest::Disable;
            cycles_in_state_ = 0;
            controlword_ = commandForState();
        }
    }
}

}  // namespace ethercat_motion
