// SPDX-License-Identifier: MIT
//
// Public axis handle. All methods are non-real-time, thread-safe and
// non-blocking (except the explicit wait*() helpers): they validate the
// request against the axis configuration and enqueue a command for the
// cyclic task. Actual values come from a wait-free status snapshot.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include "ethercat_motion/config.hpp"
#include "ethercat_motion/lockfree.hpp"
#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

class MotionController;

enum class AxisCommandType : std::uint8_t {
    Enable,
    Disable,
    QuickStop,
    ResetFault,
    SetMode,
    MoveAbsolute,
    MoveRelative,
    MoveVelocity,
    SetTorque,
    Stop,
    Home,
    SetPid,
    SetPosition,   ///< Redefine the current position (software homing).
};

const char* toString(AxisCommandType type) noexcept;

/// Which tracking slot a command belongs to (commands in the same slot
/// supersede each other).
enum class CommandSlot : std::uint8_t { Power, Motion, Config };
CommandSlot slotOf(AxisCommandType type) noexcept;

/// POD command exchanged with the cyclic task.
struct AxisCommand {
    std::uint64_t id = 0;
    std::uint16_t axis = 0;
    AxisCommandType type = AxisCommandType::Stop;
    OperationMode mode = OperationMode::None;
    double value = 0.0;          ///< position / distance / velocity / torque / kp
    double velocity = 0.0;       ///< profile velocity (0 = default) / ki
    double acceleration = 0.0;   ///< (0 = default) / kd
    double deceleration = 0.0;   ///< (0 = default)
    std::uint16_t batch_size = 0; ///< >1: applied in the same cycle as the other batch members.
};

/// Optional per-move overrides; 0 means "use configured default".
struct MoveParams {
    double velocity = 0.0;
    double acceleration = 0.0;
    double deceleration = 0.0;
};

enum class MotionState : std::uint8_t {
    Idle,
    Moving,        ///< Point-to-point move in progress.
    Velocity,      ///< Velocity (jog) command active.
    Torque,        ///< Torque command active.
    Stopping,
    Homing,
    SwitchingMode,
    Disabling,     ///< Controlled stop before disabling.
};

const char* toString(MotionState state) noexcept;

enum class CommandState : std::uint8_t { None, Pending, Completed, Failed };

struct CommandTracking {
    std::uint64_t id = 0;
    CommandState state = CommandState::None;
    ErrorCode result = ErrorCode::Ok;
};

/// Snapshot of one axis, published by the cyclic task every cycle.
struct AxisStatus {
    FixedString<32> name;
    std::uint16_t index = 0;

    DriveState drive_state = DriveState::Unknown;
    OperationMode mode = OperationMode::None;          ///< Requested mode.
    OperationMode mode_display = OperationMode::None;  ///< Reported by the drive (0x6061).
    MotionState motion_state = MotionState::Idle;

    bool comm_ok = false;
    bool enabled = false;
    bool fault = false;
    bool warning = false;
    bool moving = false;
    bool motion_done = true;
    bool homed = false;
    bool target_reached = false;
    bool at_positive_limit = false;
    bool at_negative_limit = false;
    bool motion_permitted = false;

    std::uint16_t statusword = 0;
    std::uint16_t controlword = 0;
    std::uint16_t error_code = 0;      ///< 0x603F
    std::uint32_t safety_faults = 0;   ///< Latched axis-local safety faults.

    double position = 0.0;             ///< units
    double velocity = 0.0;             ///< units/s
    double torque = 0.0;               ///< % rated torque (if mapped)
    double setpoint_position = 0.0;
    double setpoint_velocity = 0.0;
    double target_position = 0.0;
    double following_error = 0.0;
    std::int32_t position_counts = 0;

    CommandTracking power_command;
    CommandTracking motion_command;
    CommandTracking config_command;
    CommandTracking last_rejected;     ///< Most recent command rejected by the cyclic task.

    std::uint64_t cycle = 0;
    std::int64_t timestamp_ns = 0;
};

/// Result notification for a finished command (completed / failed / aborted).
struct CommandResult {
    std::uint64_t id = 0;
    std::uint16_t axis = 0;
    AxisCommandType type = AxisCommandType::Stop;
    ErrorCode code = ErrorCode::Ok;
};

class Axis {
public:
    const std::string& name() const noexcept { return name_; }
    std::size_t index() const noexcept { return index_; }
    const AxisConfig& config() const noexcept { return *config_; }

    // ---- Power -----------------------------------------------------------
    Status enable();
    Status disable();       ///< Controlled stop, then disable operation.
    Status quickStop();     ///< CiA 402 quick stop (drive ramp, 0x6085).
    Status resetFault();    ///< Rising edge on controlword bit 7; never re-enables.

    // ---- Modes and motion -----------------------------------------------
    Status setMode(OperationMode mode);
    Status moveAbsolute(double position, const MoveParams& params = {});
    Status moveTo(double position, const MoveParams& params = {}) { return moveAbsolute(position, params); }
    Status moveRelative(double distance, const MoveParams& params = {});
    Status moveVelocity(double velocity, double acceleration = 0.0);
    Status setVelocity(double velocity) { return moveVelocity(velocity); }
    Status setTorque(double torque_percent);
    Status stop(double deceleration = 0.0);
    Status home();
    /// Redefines the current actual position as `position` (software offset).
    Status setPosition(double position);
    Status setPid(double kp, double ki, double kd);

    // ---- Feedback --------------------------------------------------------
    AxisStatus status() const noexcept;
    AxisStatus getStatus() const noexcept { return status(); }
    double getActualPosition() const noexcept { return status().position; }
    double getActualVelocity() const noexcept { return status().velocity; }

    // ---- Waiting (non-real-time helpers, poll the snapshot) -------------
    /// Waits for completion of the most recently submitted command.
    Status wait(std::chrono::milliseconds timeout) const;
    Status waitForCommand(std::uint64_t id, CommandSlot slot, std::chrono::milliseconds timeout) const;
    std::uint64_t lastCommandId() const noexcept { return last_id_.load(); }

private:
    friend class MotionController;
    Axis(MotionController* owner, std::size_t index, const AxisConfig* config);

    Status submit(AxisCommand command);

    MotionController* owner_;
    std::size_t index_;
    const AxisConfig* config_;
    std::string name_;
    std::atomic<std::uint64_t> last_id_{0};
    std::atomic<std::uint8_t> last_slot_{0};
};

}  // namespace ethercat_motion
