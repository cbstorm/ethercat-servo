// SPDX-License-Identifier: MIT
//
// Common value types shared by every module: error codes, Status / Result,
// CiA 402 operation modes and drive states.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace ethercat_motion {

/// Error codes used throughout the library and mirrored 1:1 by the C API.
enum class ErrorCode : int {
    Ok = 0,
    InvalidArgument = 1,
    InvalidConfig = 2,
    NotFound = 3,
    InvalidState = 4,
    Timeout = 5,
    QueueFull = 6,
    LimitViolation = 7,
    NotSupported = 8,
    EthercatError = 9,
    DeviceMismatch = 10,
    DriveFault = 11,
    SafetyFault = 12,
    IoError = 13,
    Aborted = 14,
    Internal = 15,
};

const char* toString(ErrorCode code) noexcept;

/// Outcome of a non-real-time operation. Carries a human readable message.
/// Never constructed with a message inside the cyclic task (allocation).
class [[nodiscard]] Status {
public:
    Status() = default;
    Status(ErrorCode code, std::string message) : code_(code), message_(std::move(message)) {}

    static Status ok() { return {}; }

    bool isOk() const noexcept { return code_ == ErrorCode::Ok; }
    explicit operator bool() const noexcept { return isOk(); }
    ErrorCode code() const noexcept { return code_; }
    const std::string& message() const noexcept { return message_; }
    std::string toString() const;

private:
    ErrorCode code_ = ErrorCode::Ok;
    std::string message_;
};

/// Value-or-error for non-real-time code.
template <typename T>
class [[nodiscard]] Result {
public:
    Result(T value) : value_(std::move(value)) {}  // NOLINT(google-explicit-constructor)
    Result(Status status) : status_(std::move(status)) {  // NOLINT(google-explicit-constructor)
        if (status_.isOk()) {
            status_ = Status(ErrorCode::Internal, "Result constructed from OK status without value");
        }
    }

    bool isOk() const noexcept { return value_.has_value(); }
    explicit operator bool() const noexcept { return isOk(); }
    const Status& status() const noexcept { return status_; }

    T& value() & { return *value_; }
    const T& value() const& { return *value_; }
    T&& value() && { return std::move(*value_); }
    T* operator->() { return &*value_; }
    const T* operator->() const { return &*value_; }

private:
    std::optional<T> value_;
    Status status_;
};

/// CiA 402 modes of operation (object 0x6060 / 0x6061).
enum class OperationMode : std::int8_t {
    None = 0,
    PP = 1,      ///< Profile Position
    PV = 3,      ///< Profile Velocity
    Homing = 6,  ///< Homing
    CSP = 8,     ///< Cyclic Synchronous Position
    CSV = 9,     ///< Cyclic Synchronous Velocity
    CST = 10,    ///< Cyclic Synchronous Torque
};

const char* toString(OperationMode mode) noexcept;
std::optional<OperationMode> parseOperationMode(const std::string& text) noexcept;
bool isCyclicMode(OperationMode mode) noexcept;

/// CiA 402 power state machine states, decoded from the statusword (0x6041).
enum class DriveState : std::uint8_t {
    Unknown = 0,
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

const char* toString(DriveState state) noexcept;

}  // namespace ethercat_motion
