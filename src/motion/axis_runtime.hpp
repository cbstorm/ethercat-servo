// SPDX-License-Identifier: MIT
//
// Real-time part of one axis (private header). Everything called from the
// cyclic task is noexcept, allocation free and lock free.
#pragma once

#include <array>
#include <cstdint>

#include "ethercat_motion/axis.hpp"
#include "ethercat_motion/cia402_drive.hpp"
#include "ethercat_motion/config.hpp"
#include "ethercat_motion/diagnostics.hpp"
#include "ethercat_motion/ethercat_backend.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/lockfree.hpp"
#include "ethercat_motion/motion_controller.hpp"
#include "ethercat_motion/pdo_mapping.hpp"
#include "ethercat_motion/pid_controller.hpp"
#include "ethercat_motion/safety_manager.hpp"
#include "ethercat_motion/trajectory.hpp"

namespace ethercat_motion {

/// Output channels of the cyclic task (events + command results).
struct RtSinks {
    EventRing* events = nullptr;
    ResultQueue* results = nullptr;
    std::int64_t now_ns = 0;

    void event(EventType type, std::int16_t axis, std::int32_t code, std::int64_t value) noexcept;
    void result(const CommandResult& result) noexcept;
};

class AxisRuntime {
public:
    AxisRuntime(std::size_t index, const AxisConfig& config);

    /// Non-RT: resolves PDO entries and creates SDO requests for parameter
    /// objects that are not mapped. Called after master.configure().
    Status resolve(const PdoLayout& layout, EthercatBackend& backend, int slave_handle, std::uint32_t cycle_us);
    int slaveHandle() const noexcept { return slave_handle_; }

    // ---- Cyclic task -----------------------------------------------------
    void readInputs(const CycleContext& ctx, bool slave_ok, RtSinks& sinks) noexcept;
    void handleCommand(const AxisCommand& command, bool permitted, RtSinks& sinks) noexcept;
    void control(const CycleContext& ctx, SafetyReaction reaction, bool permitted, RtSinks& sinks) noexcept;
    const SafetyAxisInput& safetyInput() const noexcept { return safety_; }

    AxisStatus status() const noexcept { return status_.load(); }

private:
    enum Param : std::size_t {
        kParamMode,
        kParamProfileVelocity,
        kParamProfileAcceleration,
        kParamProfileDeceleration,
        kParamHomingMethod,
        kParamHomingSpeedSearch,
        kParamHomingSpeedZero,
        kParamHomingAcceleration,
        kParamCount,
    };

    struct Loc {
        bool present = false;
        PdoEntryLocation loc;
    };

    struct ParamSlot {
        Loc pdo;
        int sdo = -1;
        std::uint8_t size = 0;
        std::int64_t pdo_value = 0;
        bool pdo_valid = false;
    };

    struct PendingParam {
        Param param = kParamMode;
        std::int64_t value = 0;
    };

    enum class PpPhase : std::uint8_t { Idle, WaitParams, WaitAck, Moving };
    enum class HomingPhase : std::uint8_t { Idle, WaitReady, Running, Restore };

    // Scaling
    double countsPerUnit() const noexcept { return counts_per_unit_; }
    std::int64_t unitsToCounts(double units) const noexcept;
    double countsToUnits(std::int64_t counts) const noexcept;
    std::int64_t velocityToDrive(double units_per_s) const noexcept;
    double velocityFromDrive(std::int64_t raw) const noexcept;
    std::int64_t profileVelocityToDrive(double units_per_s) const noexcept;
    std::int64_t accelerationToDrive(double units_per_s2) const noexcept;

    // Parameters (PDO or async SDO)
    bool queueParam(Param param, std::int64_t value) noexcept;
    void stepParams(EthercatBackend& backend, RtSinks& sinks) noexcept;
    bool paramsIdle() const noexcept { return !sdo_busy_ && queue_count_ == 0; }
    void requestMode(OperationMode mode) noexcept;
    bool modeConfirmed() const noexcept;

    // Command bookkeeping
    void begin(CommandTracking& slot, AxisCommandType& slot_type, const AxisCommand& command, RtSinks& sinks) noexcept;
    void finish(CommandTracking& slot, AxisCommandType type, ErrorCode code, RtSinks& sinks) noexcept;
    void reject(const AxisCommand& command, ErrorCode code, RtSinks& sinks) noexcept;
    void abortMotion(ErrorCode code, RtSinks& sinks) noexcept;

    // Motion helpers
    bool modeSupportsPosition(OperationMode mode) const noexcept;
    bool modeSupportsVelocity(OperationMode mode) const noexcept;
    bool usesTrajectory() const noexcept;
    void startStop(double deceleration) noexcept;
    void holdSetpoint() noexcept;
    void runTrajectory(double dt, RtSinks& sinks) noexcept;
    void runProfilePosition(RtSinks& sinks) noexcept;
    void runProfileVelocity(RtSinks& sinks) noexcept;
    void runHoming(double dt, RtSinks& sinks) noexcept;
    void computeCyclicOutputs(double dt) noexcept;
    void checkSoftLimitGuard(RtSinks& sinks) noexcept;
    void applyReaction(SafetyReaction reaction, RtSinks& sinks) noexcept;
    void updatePowerCommand(RtSinks& sinks) noexcept;
    void writeOutputs(std::uint8_t* pd) noexcept;
    void publish(const CycleContext& ctx, bool permitted) noexcept;

    std::size_t index_;
    const AxisConfig& cfg_;
    int slave_handle_ = -1;
    double counts_per_unit_ = 1.0;
    std::int64_t offset_counts_ = 0;
    double dt_ = 0.001;

    Cia402Drive drive_;
    TrapezoidalProfile traj_;
    double traj_t_ = 0.0;
    PIDController pid_;

    // PDO entries used cyclically.
    Loc cw_, sw_, mode_, mode_display_, target_pos_, target_vel_, target_torque_;
    Loc pos_actual_, vel_actual_, torque_actual_, error_code_, fe_actual_;
    std::array<ParamSlot, kParamCount> params_{};
    std::array<PendingParam, 16> queue_{};
    std::size_t queue_head_ = 0;
    std::size_t queue_count_ = 0;
    bool sdo_busy_ = false;
    Param sdo_param_ = kParamMode;
    bool param_error_ = false;
    EthercatBackend* backend_ = nullptr;

    // Feedback
    bool comm_ok_ = false;
    bool feedback_valid_ = false;
    bool first_feedback_ = true;
    std::uint16_t statusword_ = 0;
    std::int8_t mode_display_value_ = 0;
    std::int32_t pos_counts_ = 0;
    std::int64_t vel_raw_ = 0;
    std::int16_t torque_raw_ = 0;
    std::uint16_t error_code_value_ = 0;
    double act_pos_ = 0.0;
    double act_vel_ = 0.0;
    double act_torque_ = 0.0;
    double prev_act_pos_ = 0.0;

    // Setpoints and outputs
    double sp_pos_ = 0.0;
    double sp_vel_ = 0.0;
    double target_ = 0.0;            // final target of the active move (units)
    double move_velocity_ = 0.0;
    double move_acceleration_ = 0.0;
    double move_deceleration_ = 0.0;
    double velocity_cmd_ = 0.0;
    double torque_cmd_ = 0.0;
    std::int32_t out_target_counts_ = 0;
    std::int32_t out_target_velocity_ = 0;
    std::int16_t out_torque_ = 0;
    std::uint16_t mode_bits_ = 0;

    OperationMode requested_mode_ = OperationMode::None;
    OperationMode saved_mode_ = OperationMode::None;
    bool mode_write_done_ = false;
    bool initial_mode_sent_ = false;
    std::uint32_t mode_switch_cycles_ = 0;

    MotionState motion_ = MotionState::Idle;
    PpPhase pp_phase_ = PpPhase::Idle;
    std::int32_t pp_target_counts_ = 0;
    HomingPhase homing_phase_ = HomingPhase::Idle;
    ErrorCode homing_result_ = ErrorCode::Ok;
    double motion_elapsed_ = 0.0;
    double motion_timeout_ = 0.0;
    std::uint32_t in_position_cycles_ = 0;
    bool pending_disable_ = false;
    bool velocity_reached_ = false;
    double watchdog_elapsed_ = 0.0;
    bool at_positive_limit_ = false;
    bool at_negative_limit_ = false;
    bool homed_ = false;

    CommandTracking power_cmd_;
    CommandTracking motion_cmd_;
    CommandTracking config_cmd_;
    CommandTracking last_rejected_;
    AxisCommandType power_type_ = AxisCommandType::Enable;
    AxisCommandType motion_type_ = AxisCommandType::Stop;
    AxisCommandType config_type_ = AxisCommandType::SetPid;

    // Local supervision
    SafetyAxisInput safety_;
    std::uint32_t following_error_cycles_ = 0;
    std::uint32_t velocity_violation_cycles_ = 0;
    bool was_inside_limits_ = true;
    SafetyReaction applied_reaction_ = SafetyReaction::None;
    bool timeout_reported_ = false;
    std::uint16_t last_controlword_ = 0;

    std::uint64_t cycle_ = 0;
    FixedString<32> name_;
    SeqLock<AxisStatus> status_;
};

}  // namespace ethercat_motion
