// SPDX-License-Identifier: MIT
#include "ethercat_motion/types.hpp"

#include <algorithm>
#include <cctype>

namespace ethercat_motion {

const char* toString(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::Ok: return "ok";
        case ErrorCode::InvalidArgument: return "invalid argument";
        case ErrorCode::InvalidConfig: return "invalid configuration";
        case ErrorCode::NotFound: return "not found";
        case ErrorCode::InvalidState: return "invalid state";
        case ErrorCode::Timeout: return "timeout";
        case ErrorCode::QueueFull: return "queue full";
        case ErrorCode::LimitViolation: return "limit violation";
        case ErrorCode::NotSupported: return "not supported";
        case ErrorCode::EthercatError: return "EtherCAT error";
        case ErrorCode::DeviceMismatch: return "device mismatch";
        case ErrorCode::DriveFault: return "drive fault";
        case ErrorCode::SafetyFault: return "safety fault";
        case ErrorCode::IoError: return "I/O error";
        case ErrorCode::Aborted: return "aborted";
        case ErrorCode::Internal: return "internal error";
    }
    return "unknown error";
}

std::string Status::toString() const {
    if (isOk()) {
        return "ok";
    }
    std::string text = ethercat_motion::toString(code_);
    if (!message_.empty()) {
        text += ": ";
        text += message_;
    }
    return text;
}

const char* toString(OperationMode mode) noexcept {
    switch (mode) {
        case OperationMode::None: return "NONE";
        case OperationMode::PP: return "PP";
        case OperationMode::PV: return "PV";
        case OperationMode::Homing: return "HOMING";
        case OperationMode::CSP: return "CSP";
        case OperationMode::CSV: return "CSV";
        case OperationMode::CST: return "CST";
    }
    return "UNKNOWN";
}

std::optional<OperationMode> parseOperationMode(const std::string& text) noexcept {
    std::string t = text;
    std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (t == "PP" || t == "PROFILE_POSITION" || t == "1") return OperationMode::PP;
    if (t == "PV" || t == "PROFILE_VELOCITY" || t == "3") return OperationMode::PV;
    if (t == "HOMING" || t == "HM" || t == "6") return OperationMode::Homing;
    if (t == "CSP" || t == "CYCLIC_SYNC_POSITION" || t == "8") return OperationMode::CSP;
    if (t == "CSV" || t == "CYCLIC_SYNC_VELOCITY" || t == "9") return OperationMode::CSV;
    if (t == "CST" || t == "CYCLIC_SYNC_TORQUE" || t == "10") return OperationMode::CST;
    return std::nullopt;
}

bool isCyclicMode(OperationMode mode) noexcept {
    return mode == OperationMode::CSP || mode == OperationMode::CSV || mode == OperationMode::CST;
}

const char* toString(DriveState state) noexcept {
    switch (state) {
        case DriveState::Unknown: return "Unknown";
        case DriveState::NotReadyToSwitchOn: return "NotReadyToSwitchOn";
        case DriveState::SwitchOnDisabled: return "SwitchOnDisabled";
        case DriveState::ReadyToSwitchOn: return "ReadyToSwitchOn";
        case DriveState::SwitchedOn: return "SwitchedOn";
        case DriveState::OperationEnabled: return "OperationEnabled";
        case DriveState::QuickStopActive: return "QuickStopActive";
        case DriveState::FaultReactionActive: return "FaultReactionActive";
        case DriveState::Fault: return "Fault";
    }
    return "Unknown";
}

}  // namespace ethercat_motion
