// SPDX-License-Identifier: MIT
#include "axis_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ethercat_motion {

void RtSinks::event(EventType type, std::int16_t axis, std::int32_t code, std::int64_t value) noexcept {
    if (events == nullptr) return;
    Event e;
    e.time_ns = now_ns;
    e.type = type;
    e.axis = axis;
    e.code = code;
    e.value = value;
    (void)events->tryPush(e);
}

void RtSinks::result(const CommandResult& r) noexcept {
    if (results != nullptr) (void)results->tryPush(r);
}

namespace {

std::int32_t clampToInt32(std::int64_t v) noexcept {
    return static_cast<std::int32_t>(std::clamp<std::int64_t>(v, std::numeric_limits<std::int32_t>::min(),
                                                               std::numeric_limits<std::int32_t>::max()));
}

std::uint32_t msToCycles(std::uint32_t ms, std::uint32_t cycle_us) noexcept {
    const std::uint64_t cycles = (static_cast<std::uint64_t>(ms) * 1000U + cycle_us - 1) / cycle_us;
    return static_cast<std::uint32_t>(std::max<std::uint64_t>(1, cycles));
}

}  // namespace

AxisRuntime::AxisRuntime(std::size_t index, const AxisConfig& config)
    : index_(index), cfg_(config), pid_(config.pid) {
    counts_per_unit_ = config.counts_per_revolution * config.gear_ratio / config.units_per_revolution *
                       static_cast<double>(config.direction);
    requested_mode_ = config.operation_mode;
    name_.assign(config.name.c_str());
    publish(CycleContext{}, false);
}

// ---------------------------------------------------------------------------
// Scaling
// ---------------------------------------------------------------------------

std::int64_t AxisRuntime::unitsToCounts(double units) const noexcept {
    return std::llround(units * counts_per_unit_) + offset_counts_;
}

double AxisRuntime::countsToUnits(std::int64_t counts) const noexcept {
    return static_cast<double>(counts - offset_counts_) / counts_per_unit_;
}

std::int64_t AxisRuntime::velocityToDrive(double v) const noexcept {
    return std::llround(v * counts_per_unit_ * cfg_.velocity_factor);
}

double AxisRuntime::velocityFromDrive(std::int64_t raw) const noexcept {
    return static_cast<double>(raw) / (counts_per_unit_ * cfg_.velocity_factor);
}

std::int64_t AxisRuntime::profileVelocityToDrive(double v) const noexcept {
    return std::llround(std::fabs(v * counts_per_unit_) * cfg_.velocity_factor);
}

std::int64_t AxisRuntime::accelerationToDrive(double a) const noexcept {
    return std::llround(std::fabs(a * counts_per_unit_) * cfg_.acceleration_factor);
}

// ---------------------------------------------------------------------------
// Resolution (non-RT)
// ---------------------------------------------------------------------------

Status AxisRuntime::resolve(const PdoLayout& layout, EthercatBackend& backend, int slave_handle,
                            std::uint32_t cycle_us) {
    slave_handle_ = slave_handle;
    backend_ = &backend;
    dt_ = static_cast<double>(cycle_us) * 1e-6;

    auto find = [&](std::uint16_t idx, std::uint8_t sub, PdoDirection dir) {
        Loc l;
        if (const PdoEntryLocation* e = layout.find(idx, sub); e != nullptr && e->direction == dir) {
            l.present = true;
            l.loc = *e;
        }
        return l;
    };
    using namespace cia402_obj;
    cw_ = find(kControlword, 0, PdoDirection::Output);
    sw_ = find(kStatusword, 0, PdoDirection::Input);
    mode_ = find(kModesOfOperation, 0, PdoDirection::Output);
    mode_display_ = find(kModesOfOperationDisplay, 0, PdoDirection::Input);
    target_pos_ = find(kTargetPosition, 0, PdoDirection::Output);
    target_vel_ = find(kTargetVelocity, 0, PdoDirection::Output);
    target_torque_ = find(kTargetTorque, 0, PdoDirection::Output);
    pos_actual_ = find(kPositionActual, 0, PdoDirection::Input);
    vel_actual_ = find(kVelocityActual, 0, PdoDirection::Input);
    torque_actual_ = find(kTorqueActual, 0, PdoDirection::Input);
    error_code_ = find(kErrorCode, 0, PdoDirection::Input);
    fe_actual_ = find(kFollowingErrorActual, 0, PdoDirection::Input);

    if (!cw_.present || !sw_.present || !pos_actual_.present) {
        return {ErrorCode::InvalidConfig, "axis '" + cfg_.name + "': 0x6040, 0x6041 and 0x6064 must be mapped"};
    }

    std::vector<OperationMode> modes = cfg_.allowed_modes;
    modes.push_back(cfg_.operation_mode);
    auto allowed = [&](OperationMode m) { return std::find(modes.begin(), modes.end(), m) != modes.end(); };

    struct Def {
        Param param;
        std::uint16_t index;
        std::uint8_t sub;
        std::uint8_t size;
        bool needed;
    };
    const Def defs[] = {
        {kParamMode, kModesOfOperation, 0, 1, true},
        {kParamProfileVelocity, kProfileVelocity, 0, 4, allowed(OperationMode::PP)},
        {kParamProfileAcceleration, kProfileAcceleration, 0, 4,
         allowed(OperationMode::PP) || allowed(OperationMode::PV)},
        {kParamProfileDeceleration, kProfileDeceleration, 0, 4,
         allowed(OperationMode::PP) || allowed(OperationMode::PV)},
        {kParamHomingMethod, kHomingMethod, 0, 1, allowed(OperationMode::Homing)},
        {kParamHomingSpeedSearch, kHomingSpeeds, 1, 4, allowed(OperationMode::Homing)},
        {kParamHomingSpeedZero, kHomingSpeeds, 2, 4, allowed(OperationMode::Homing)},
        {kParamHomingAcceleration, kHomingAcceleration, 0, 4, allowed(OperationMode::Homing)},
    };
    for (const Def& d : defs) {
        ParamSlot& slot = params_[d.param];
        slot.size = d.size;
        slot.pdo = find(d.index, d.sub, PdoDirection::Output);
        if (d.needed && !slot.pdo.present) {
            auto req = backend.createSdoRequest(slave_handle, d.index, d.sub, d.size, cfg_.timeouts.sdo_ms);
            if (!req) {
                return {req.status().code(), "axis '" + cfg_.name + "': " + req.status().message()};
            }
            slot.sdo = req.value();
        }
    }
    if (params_[kParamMode].pdo.present) {
        params_[kParamMode].pdo_value = static_cast<std::int8_t>(requested_mode_);
        params_[kParamMode].pdo_valid = true;
    }

    Cia402Config dc;
    dc.transition_timeout_cycles = msToCycles(cfg_.timeouts.state_transition_ms, cycle_us);
    dc.fault_reset_timeout_cycles = msToCycles(cfg_.timeouts.fault_reset_ms, cycle_us);
    dc.fault_reset_pulse_cycles = std::max<std::uint32_t>(2, msToCycles(10, cycle_us));
    drive_ = Cia402Drive(dc);
    return Status::ok();
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

bool AxisRuntime::queueParam(Param param, std::int64_t value) noexcept {
    ParamSlot& slot = params_[param];
    if (slot.pdo.present) {
        slot.pdo_value = value;
        slot.pdo_valid = true;
        if (param == kParamMode) mode_write_done_ = true;
        return true;
    }
    if (slot.sdo < 0 || queue_count_ >= queue_.size()) {
        param_error_ = true;
        return false;
    }
    queue_[(queue_head_ + queue_count_) % queue_.size()] = {param, value};
    ++queue_count_;
    return true;
}

void AxisRuntime::stepParams(EthercatBackend& backend, RtSinks& sinks) noexcept {
    if (sdo_busy_) {
        const ParamSlot& slot = params_[sdo_param_];
        switch (backend.sdoState(slot.sdo)) {
            case SdoRequestState::Success:
                sdo_busy_ = false;
                if (sdo_param_ == kParamMode) mode_write_done_ = true;
                break;
            case SdoRequestState::Error:
                sdo_busy_ = false;
                param_error_ = true;
                queue_count_ = 0;
                sinks.event(EventType::SdoError, static_cast<std::int16_t>(index_), static_cast<std::int32_t>(sdo_param_),
                            0);
                break;
            case SdoRequestState::Busy:
            case SdoRequestState::Unused:
                return;
        }
    }
    if (!sdo_busy_ && queue_count_ > 0) {
        const PendingParam p = queue_[queue_head_];
        queue_head_ = (queue_head_ + 1) % queue_.size();
        --queue_count_;
        const ParamSlot& slot = params_[p.param];
        std::uint8_t* data = backend.sdoData(slot.sdo);
        if (data == nullptr) {
            param_error_ = true;
            return;
        }
        for (std::size_t i = 0; i < slot.size; ++i) {
            data[i] = static_cast<std::uint8_t>((p.value >> (8 * i)) & 0xFF);
        }
        backend.sdoWrite(slot.sdo);
        sdo_busy_ = true;
        sdo_param_ = p.param;
    }
}

void AxisRuntime::requestMode(OperationMode mode) noexcept {
    requested_mode_ = mode;
    mode_write_done_ = false;
    mode_switch_cycles_ = 0;
    (void)queueParam(kParamMode, static_cast<std::int8_t>(mode));
}

bool AxisRuntime::modeConfirmed() const noexcept {
    if (!mode_write_done_) return false;
    if (mode_display_.present) return mode_display_value_ == static_cast<std::int8_t>(requested_mode_);
    return true;  // no 0x6061 mapped: trust the successful write
}

// ---------------------------------------------------------------------------
// Command bookkeeping
// ---------------------------------------------------------------------------

void AxisRuntime::begin(CommandTracking& slot, AxisCommandType& slot_type, const AxisCommand& command,
                        RtSinks& sinks) noexcept {
    if (slot.state == CommandState::Pending && slot.id != command.id) {
        finish(slot, slot_type, ErrorCode::Aborted, sinks);
    }
    slot.id = command.id;
    slot.state = CommandState::Pending;
    slot.result = ErrorCode::Ok;
    slot_type = command.type;
}

void AxisRuntime::finish(CommandTracking& slot, AxisCommandType type, ErrorCode code, RtSinks& sinks) noexcept {
    if (slot.state != CommandState::Pending) return;
    slot.state = code == ErrorCode::Ok ? CommandState::Completed : CommandState::Failed;
    slot.result = code;
    sinks.result({slot.id, static_cast<std::uint16_t>(index_), type, code});
    sinks.event(code == ErrorCode::Ok ? EventType::CommandCompleted : EventType::CommandFailed,
                static_cast<std::int16_t>(index_), static_cast<std::int32_t>(type),
                static_cast<std::int64_t>(code));
}

void AxisRuntime::reject(const AxisCommand& command, ErrorCode code, RtSinks& sinks) noexcept {
    last_rejected_.id = command.id;
    last_rejected_.state = CommandState::Failed;
    last_rejected_.result = code;
    sinks.result({command.id, static_cast<std::uint16_t>(index_), command.type, code});
    sinks.event(EventType::CommandRejected, static_cast<std::int16_t>(index_), static_cast<std::int32_t>(command.type),
                static_cast<std::int64_t>(code));
}

void AxisRuntime::abortMotion(ErrorCode code, RtSinks& sinks) noexcept {
    finish(motion_cmd_, motion_type_, code, sinks);
    switch (motion_) {
        case MotionState::Homing:
            mode_bits_ = 0;
            homing_phase_ = HomingPhase::Idle;
            requestMode(saved_mode_);
            motion_ = MotionState::SwitchingMode;
            return;
        case MotionState::SwitchingMode:
            return;  // let the mode switch settle
        default:
            break;
    }
    if (pp_phase_ != PpPhase::Idle) {
        mode_bits_ = static_cast<std::uint16_t>((mode_bits_ & ~cia402::kCwBit4) | cia402::kCwHalt);
        pp_phase_ = PpPhase::Idle;
    }
    velocity_cmd_ = 0.0;
    torque_cmd_ = 0.0;
    pending_disable_ = false;
    motion_ = MotionState::Idle;
}

// ---------------------------------------------------------------------------
// Inputs and local supervision
// ---------------------------------------------------------------------------

void AxisRuntime::readInputs(const CycleContext& ctx, bool slave_ok, RtSinks& sinks) noexcept {
    dt_ = ctx.dt_s;
    comm_ok_ = slave_ok;
    safety_ = SafetyAxisInput{};
    const bool data_valid = slave_ok && ctx.comm != nullptr && ctx.comm->wc_complete && ctx.process_data != nullptr;
    if (!data_valid) {
        if (!slave_ok) feedback_valid_ = false;
        return;
    }
    const std::uint8_t* pd = ctx.process_data;
    statusword_ = static_cast<std::uint16_t>(pd::readEntry(pd, sw_.loc, false));
    if (mode_display_.present) mode_display_value_ = static_cast<std::int8_t>(pd::readEntry(pd, mode_display_.loc, true));
    pos_counts_ = static_cast<std::int32_t>(pd::readEntry(pd, pos_actual_.loc, true));
    if (vel_actual_.present) vel_raw_ = pd::readEntry(pd, vel_actual_.loc, true);
    if (torque_actual_.present) torque_raw_ = static_cast<std::int16_t>(pd::readEntry(pd, torque_actual_.loc, true));
    if (error_code_.present) error_code_value_ = static_cast<std::uint16_t>(pd::readEntry(pd, error_code_.loc, false));

    prev_act_pos_ = act_pos_;
    act_pos_ = countsToUnits(pos_counts_);
    if (first_feedback_) {
        prev_act_pos_ = act_pos_;
        sp_pos_ = act_pos_;
        target_ = act_pos_;
        out_target_counts_ = pos_counts_;
        pp_target_counts_ = pos_counts_;
    }
    act_vel_ = vel_actual_.present ? velocityFromDrive(vel_raw_) : (act_pos_ - prev_act_pos_) / dt_;
    act_torque_ = static_cast<double>(torque_raw_) / 10.0 * cfg_.direction;
    feedback_valid_ = true;

    const bool was_fault = drive_.isFault();
    drive_.update(statusword_);
    const auto axis = static_cast<std::int16_t>(index_);
    if (drive_.stateChanged()) {
        sinks.event(EventType::DriveStateChanged, axis, static_cast<std::int32_t>(drive_.state()), statusword_);
        if (drive_.isFault() && !was_fault) {
            sinks.event(EventType::DriveFault, axis, error_code_value_, statusword_);
        } else if (!drive_.isFault() && was_fault) {
            sinks.event(EventType::DriveFaultCleared, axis, 0, statusword_);
        }
    }

    safety_.drive_fault = drive_.isFault();
    safety_.transition_timeout = drive_.transitionTimedOut();

    const bool enabled = drive_.isOperationEnabled();
    const AxisLimits& lim = cfg_.limits;
    if (lim.soft_limits_enabled) {
        const bool inside = act_pos_ >= lim.position_min - lim.position_tolerance &&
                            act_pos_ <= lim.position_max + lim.position_tolerance;
        if (!first_feedback_ && enabled && !inside && was_inside_limits_) {
            safety_.position_limit = true;
            sinks.event(EventType::PositionLimit, axis, act_pos_ > lim.position_max ? 1 : -1, pos_counts_);
        }
        was_inside_limits_ = inside;
    }

    if (enabled && std::fabs(act_vel_) > lim.velocity_max * (1.0 + lim.velocity_tolerance)) {
        if (++velocity_violation_cycles_ >= lim.velocity_violation_cycles) {
            safety_.velocity_limit = true;
            if (velocity_violation_cycles_ == lim.velocity_violation_cycles) {
                sinks.event(EventType::VelocityLimit, axis, 0, std::llround(act_vel_ * 1000.0));
            }
        }
    } else {
        velocity_violation_cycles_ = 0;
    }

    const bool position_loop_active =
        requested_mode_ == OperationMode::CSP ||
        (cfg_.control_loop == ControlLoop::Position &&
         (requested_mode_ == OperationMode::CSV || requested_mode_ == OperationMode::CST));
    if (lim.following_error_max > 0.0 && enabled && modeConfirmed() && position_loop_active &&
        motion_ != MotionState::SwitchingMode && motion_ != MotionState::Homing) {
        if (std::fabs(sp_pos_ - act_pos_) > lim.following_error_max) {
            if (++following_error_cycles_ >= lim.following_error_cycles) {
                safety_.following_error = true;
                if (following_error_cycles_ == lim.following_error_cycles) {
                    sinks.event(EventType::FollowingError, axis, 0, std::llround((sp_pos_ - act_pos_) * 1000.0));
                }
            }
        } else {
            following_error_cycles_ = 0;
        }
    } else {
        following_error_cycles_ = 0;
    }
    first_feedback_ = false;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

bool AxisRuntime::modeSupportsPosition(OperationMode mode) const noexcept {
    switch (mode) {
        case OperationMode::CSP:
        case OperationMode::PP:
            return true;
        case OperationMode::CSV:
        case OperationMode::CST:
            return cfg_.control_loop == ControlLoop::Position;
        default:
            return false;
    }
}

bool AxisRuntime::modeSupportsVelocity(OperationMode mode) const noexcept {
    switch (mode) {
        case OperationMode::CSP:
        case OperationMode::CSV:
        case OperationMode::PV:
            return true;
        case OperationMode::CST:
            return cfg_.control_loop != ControlLoop::None;
        default:
            return false;
    }
}

bool AxisRuntime::usesTrajectory() const noexcept { return isCyclicMode(requested_mode_); }

void AxisRuntime::handleCommand(const AxisCommand& cmd, bool permitted, RtSinks& sinks) noexcept {
    const bool enabled = drive_.isOperationEnabled();
    const bool confirmed = modeConfirmed();
    const bool busy_state = motion_ == MotionState::SwitchingMode || motion_ == MotionState::Homing ||
                            motion_ == MotionState::Disabling;
    const AxisLimits& lim = cfg_.limits;

    auto motion_preconditions = [&]() -> ErrorCode {
        if (!comm_ok_ || !feedback_valid_) return ErrorCode::EthercatError;
        if (!enabled) return ErrorCode::InvalidState;
        if (!permitted) return ErrorCode::SafetyFault;
        if (!confirmed || busy_state) return ErrorCode::InvalidState;
        return ErrorCode::Ok;
    };

    switch (cmd.type) {
        case AxisCommandType::Enable: {
            if (!comm_ok_ || !feedback_valid_) return reject(cmd, ErrorCode::EthercatError, sinks);
            if (!permitted) return reject(cmd, ErrorCode::SafetyFault, sinks);
            if (drive_.isFault()) return reject(cmd, ErrorCode::DriveFault, sinks);
            begin(power_cmd_, power_type_, cmd, sinks);
            pending_disable_ = false;
            drive_.requestEnable();
            return;
        }
        case AxisCommandType::Disable: {
            begin(power_cmd_, power_type_, cmd, sinks);
            const bool moving = motion_ == MotionState::Moving || motion_ == MotionState::Velocity ||
                                motion_ == MotionState::Stopping;
            if (enabled && moving) {
                finish(motion_cmd_, motion_type_, ErrorCode::Aborted, sinks);
                startStop(cfg_.motion.deceleration);
                if (motion_ == MotionState::Stopping) {
                    motion_ = MotionState::Disabling;
                    pending_disable_ = true;
                    return;
                }
            } else if (motion_ != MotionState::Idle) {
                abortMotion(ErrorCode::Aborted, sinks);
            }
            drive_.requestDisable();
            return;
        }
        case AxisCommandType::QuickStop:
            begin(power_cmd_, power_type_, cmd, sinks);
            abortMotion(ErrorCode::Aborted, sinks);
            drive_.requestQuickStop();
            return;
        case AxisCommandType::ResetFault:
            begin(power_cmd_, power_type_, cmd, sinks);
            drive_.requestFaultReset();
            return;
        case AxisCommandType::SetMode: {
            if (motion_ != MotionState::Idle && motion_ != MotionState::Torque) {
                return reject(cmd, ErrorCode::InvalidState, sinks);
            }
            begin(motion_cmd_, motion_type_, cmd, sinks);
            torque_cmd_ = 0.0;
            if (cmd.mode == requested_mode_ && confirmed) {
                motion_ = MotionState::Idle;
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                return;
            }
            param_error_ = false;
            holdSetpoint();
            requestMode(cmd.mode);
            motion_ = MotionState::SwitchingMode;
            return;
        }
        case AxisCommandType::MoveAbsolute:
        case AxisCommandType::MoveRelative: {
            if (const ErrorCode pre = motion_preconditions(); pre != ErrorCode::Ok) return reject(cmd, pre, sinks);
            if (!modeSupportsPosition(requested_mode_)) return reject(cmd, ErrorCode::NotSupported, sinks);
            double base = sp_pos_;
            if (motion_ == MotionState::Moving) base = target_;
            else if (requested_mode_ == OperationMode::PP) base = act_pos_;
            const double target = cmd.type == AxisCommandType::MoveAbsolute ? cmd.value : base + cmd.value;
            if (!std::isfinite(target)) return reject(cmd, ErrorCode::InvalidArgument, sinks);
            if (lim.soft_limits_enabled && (target < lim.position_min || target > lim.position_max)) {
                return reject(cmd, ErrorCode::LimitViolation, sinks);
            }
            const double v = cmd.velocity > 0.0 ? std::min(cmd.velocity, lim.velocity_max) : cfg_.motion.velocity;
            const double a =
                cmd.acceleration > 0.0 ? std::min(cmd.acceleration, lim.acceleration_max) : cfg_.motion.acceleration;
            const double d =
                cmd.deceleration > 0.0 ? std::min(cmd.deceleration, lim.deceleration_max) : cfg_.motion.deceleration;

            TrapezoidalProfile estimate;
            const double from = requested_mode_ == OperationMode::PP ? act_pos_ : sp_pos_;
            const double from_v = requested_mode_ == OperationMode::PP ? 0.0 : sp_vel_;
            if (!estimate.planPosition(from, from_v, target, v, a, d)) {
                return reject(cmd, ErrorCode::InvalidArgument, sinks);
            }
            begin(motion_cmd_, motion_type_, cmd, sinks);
            target_ = target;
            move_velocity_ = v;
            move_acceleration_ = a;
            move_deceleration_ = d;
            motion_elapsed_ = 0.0;
            in_position_cycles_ = 0;
            motion_timeout_ =
                cfg_.motion.motion_timeout_s > 0.0 ? cfg_.motion.motion_timeout_s : estimate.duration() * 1.5 + 2.0;

            if (requested_mode_ == OperationMode::PP) {
                param_error_ = false;
                (void)queueParam(kParamProfileVelocity, profileVelocityToDrive(v));
                (void)queueParam(kParamProfileAcceleration, accelerationToDrive(a));
                (void)queueParam(kParamProfileDeceleration, accelerationToDrive(d));
                pp_target_counts_ = clampToInt32(unitsToCounts(target));
                mode_bits_ = static_cast<std::uint16_t>(mode_bits_ & ~cia402::kCwHalt);
                pp_phase_ = PpPhase::WaitParams;
            } else {
                traj_ = estimate;
                traj_t_ = 0.0;
            }
            motion_ = MotionState::Moving;
            return;
        }
        case AxisCommandType::MoveVelocity: {
            if (const ErrorCode pre = motion_preconditions(); pre != ErrorCode::Ok) return reject(cmd, pre, sinks);
            if (!modeSupportsVelocity(requested_mode_)) return reject(cmd, ErrorCode::NotSupported, sinks);
            const double v = cmd.value;
            if (!std::isfinite(v) || std::fabs(v) > lim.velocity_max) return reject(cmd, ErrorCode::LimitViolation, sinks);
            if (lim.soft_limits_enabled &&
                ((v > 0.0 && act_pos_ >= lim.position_max) || (v < 0.0 && act_pos_ <= lim.position_min))) {
                return reject(cmd, ErrorCode::LimitViolation, sinks);
            }
            const double a =
                cmd.acceleration > 0.0 ? std::min(cmd.acceleration, lim.acceleration_max) : cfg_.motion.acceleration;
            const double d = cfg_.motion.deceleration;
            begin(motion_cmd_, motion_type_, cmd, sinks);
            move_acceleration_ = a;
            move_deceleration_ = d;
            move_velocity_ = std::fabs(v);
            velocity_reached_ = false;
            watchdog_elapsed_ = 0.0;
            motion_elapsed_ = 0.0;
            at_positive_limit_ = false;
            at_negative_limit_ = false;
            if (requested_mode_ == OperationMode::PV) {
                param_error_ = false;
                (void)queueParam(kParamProfileAcceleration, accelerationToDrive(a));
                (void)queueParam(kParamProfileDeceleration, accelerationToDrive(d));
                velocity_cmd_ = v;
            } else {
                traj_.planVelocity(sp_pos_, sp_vel_, v, a, d);
                traj_t_ = 0.0;
            }
            motion_ = MotionState::Velocity;
            return;
        }
        case AxisCommandType::SetTorque: {
            if (const ErrorCode pre = motion_preconditions(); pre != ErrorCode::Ok) return reject(cmd, pre, sinks);
            if (requested_mode_ != OperationMode::CST || cfg_.control_loop != ControlLoop::None) {
                return reject(cmd, ErrorCode::NotSupported, sinks);
            }
            if (!std::isfinite(cmd.value) || std::fabs(cmd.value) > lim.torque_max) {
                return reject(cmd, ErrorCode::LimitViolation, sinks);
            }
            begin(motion_cmd_, motion_type_, cmd, sinks);
            torque_cmd_ = cmd.value;
            watchdog_elapsed_ = 0.0;
            motion_ = MotionState::Torque;
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
            return;
        }
        case AxisCommandType::Stop: {
            begin(motion_cmd_, motion_type_, cmd, sinks);
            if (!enabled || motion_ == MotionState::Idle || motion_ == MotionState::SwitchingMode) {
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                return;
            }
            if (motion_ == MotionState::Torque) {
                torque_cmd_ = 0.0;
                motion_ = MotionState::Idle;
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                return;
            }
            const double dec = cmd.deceleration > 0.0 ? std::min(cmd.deceleration, lim.quick_stop_deceleration)
                                                      : cfg_.motion.deceleration;
            const bool disabling = motion_ == MotionState::Disabling;
            startStop(dec);
            if (disabling && motion_ == MotionState::Stopping) motion_ = MotionState::Disabling;
            return;
        }
        case AxisCommandType::Home: {
            if (const ErrorCode pre = motion_preconditions(); pre != ErrorCode::Ok) return reject(cmd, pre, sinks);
            if (motion_ != MotionState::Idle) return reject(cmd, ErrorCode::InvalidState, sinks);
            const ParamSlot& method_slot = params_[kParamHomingMethod];
            if (cfg_.homing.method == 0 || (!method_slot.pdo.present && method_slot.sdo < 0)) {
                return reject(cmd, ErrorCode::NotSupported, sinks);
            }
            begin(motion_cmd_, motion_type_, cmd, sinks);
            saved_mode_ = requested_mode_;
            param_error_ = false;
            homing_result_ = ErrorCode::Ok;
            (void)queueParam(kParamHomingMethod, cfg_.homing.method);
            (void)queueParam(kParamHomingSpeedSearch, profileVelocityToDrive(cfg_.homing.speed_search));
            (void)queueParam(kParamHomingSpeedZero, profileVelocityToDrive(cfg_.homing.speed_zero));
            (void)queueParam(kParamHomingAcceleration, accelerationToDrive(cfg_.homing.acceleration));
            holdSetpoint();
            requestMode(OperationMode::Homing);
            homing_phase_ = HomingPhase::WaitReady;
            motion_elapsed_ = 0.0;
            motion_ = MotionState::Homing;
            return;
        }
        case AxisCommandType::SetPid:
            begin(config_cmd_, config_type_, cmd, sinks);
            pid_.setTunings(cmd.value, cmd.velocity, cmd.acceleration);
            finish(config_cmd_, config_type_, ErrorCode::Ok, sinks);
            return;
        case AxisCommandType::SetPosition: {
            if (!feedback_valid_) return reject(cmd, ErrorCode::EthercatError, sinks);
            if (motion_ != MotionState::Idle) return reject(cmd, ErrorCode::InvalidState, sinks);
            begin(config_cmd_, config_type_, cmd, sinks);
            offset_counts_ = static_cast<std::int64_t>(pos_counts_) - std::llround(cmd.value * counts_per_unit_);
            act_pos_ = countsToUnits(pos_counts_);
            prev_act_pos_ = act_pos_;
            holdSetpoint();
            target_ = act_pos_;
            homed_ = true;
            was_inside_limits_ = !lim.soft_limits_enabled ||
                                 (act_pos_ >= lim.position_min - lim.position_tolerance &&
                                  act_pos_ <= lim.position_max + lim.position_tolerance);
            finish(config_cmd_, config_type_, ErrorCode::Ok, sinks);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

void AxisRuntime::holdSetpoint() noexcept {
    sp_pos_ = act_pos_;
    sp_vel_ = 0.0;
    traj_.hold(act_pos_);
    traj_t_ = 0.0;
    pid_.reset();
    out_target_counts_ = pos_counts_;
    pp_target_counts_ = pos_counts_;
    out_target_velocity_ = 0;
    out_torque_ = 0;
}

void AxisRuntime::startStop(double deceleration) noexcept {
    const double dec = deceleration > 0.0 ? deceleration : cfg_.motion.deceleration;
    move_deceleration_ = dec;
    motion_elapsed_ = 0.0;
    switch (modeConfirmed() ? requested_mode_ : OperationMode::None) {
        case OperationMode::PP:
            mode_bits_ = static_cast<std::uint16_t>((mode_bits_ & ~cia402::kCwBit4) | cia402::kCwHalt);
            pp_phase_ = PpPhase::Idle;
            motion_ = MotionState::Stopping;
            return;
        case OperationMode::PV:
            velocity_cmd_ = 0.0;
            motion_ = MotionState::Stopping;
            return;
        case OperationMode::CST:
            if (cfg_.control_loop == ControlLoop::None) {
                torque_cmd_ = 0.0;
                motion_ = MotionState::Idle;
                return;
            }
            break;
        case OperationMode::CSP:
        case OperationMode::CSV:
            break;
        default:
            motion_ = MotionState::Idle;
            return;
    }
    traj_.planStop(sp_pos_, sp_vel_, dec);
    traj_t_ = 0.0;
    motion_ = MotionState::Stopping;
}

void AxisRuntime::checkSoftLimitGuard(RtSinks& sinks) noexcept {
    const AxisLimits& lim = cfg_.limits;
    if (!lim.soft_limits_enabled) return;
    const double dec = move_deceleration_ > 0.0 ? move_deceleration_ : cfg_.motion.deceleration;
    // Look one cycle ahead so the ramp starts before the limit is passed.
    const double stop = TrapezoidalProfile::stoppingDistance(sp_vel_, dec) + std::fabs(sp_vel_) * dt_;
    const auto axis = static_cast<std::int16_t>(index_);
    const bool toward_max = sp_vel_ > 0.0 && sp_pos_ + stop >= lim.position_max;
    const bool toward_min = sp_vel_ < 0.0 && sp_pos_ - stop <= lim.position_min;
    if (!toward_max && !toward_min) return;

    const double limit = toward_max ? lim.position_max : lim.position_min;
    const double remaining = std::fabs(limit - sp_pos_);
    // Decelerate exactly onto the limit (never beyond the quick-stop ramp).
    double stop_dec = dec;
    if (remaining > 1e-12) {
        stop_dec = std::min(std::max(dec, sp_vel_ * sp_vel_ / (2.0 * remaining)), cfg_.limits.quick_stop_deceleration);
    } else {
        stop_dec = cfg_.limits.quick_stop_deceleration;
    }
    traj_.planPosition(sp_pos_, sp_vel_, limit, std::max(std::fabs(sp_vel_), 1e-9), move_acceleration_, stop_dec);
    traj_t_ = 0.0;
    motion_ = MotionState::Stopping;
    at_positive_limit_ = toward_max;
    at_negative_limit_ = toward_min;
    finish(motion_cmd_, motion_type_, ErrorCode::LimitViolation, sinks);
    sinks.event(EventType::PositionLimit, axis, toward_max ? 2 : -2, 0);
}

void AxisRuntime::runTrajectory(double dt, RtSinks& sinks) noexcept {
    if (motion_ == MotionState::Idle || motion_ == MotionState::Torque) return;
    traj_t_ += dt;
    const TrajectoryPoint p = traj_.sample(traj_t_);
    sp_pos_ = p.position;
    sp_vel_ = p.velocity;
    const auto axis = static_cast<std::int16_t>(index_);

    switch (motion_) {
        case MotionState::Moving: {
            if (traj_.finished(traj_t_)) {
                sp_vel_ = 0.0;
                const std::uint32_t needed =
                    std::max<std::uint32_t>(1, static_cast<std::uint32_t>(cfg_.motion.in_position_time_ms * 1e-3 / dt));
                if (std::fabs(act_pos_ - target_) <= cfg_.motion.in_position_window) {
                    if (++in_position_cycles_ >= needed) {
                        motion_ = MotionState::Idle;
                        finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                        return;
                    }
                } else {
                    in_position_cycles_ = 0;
                }
            }
            if (motion_elapsed_ > motion_timeout_) {
                sinks.event(EventType::MotionTimeout, axis, 0, std::llround(motion_elapsed_ * 1000.0));
                finish(motion_cmd_, motion_type_, ErrorCode::Timeout, sinks);
                if (traj_.finished(traj_t_)) {
                    motion_ = MotionState::Idle;
                } else {
                    startStop(move_deceleration_);
                }
            }
            return;
        }
        case MotionState::Velocity:
            checkSoftLimitGuard(sinks);
            if (motion_ == MotionState::Velocity && !velocity_reached_ && traj_t_ >= traj_.duration()) {
                velocity_reached_ = true;
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                if (traj_.finalVelocity() == 0.0) motion_ = MotionState::Idle;
            }
            return;
        case MotionState::Stopping:
        case MotionState::Disabling:
            if (traj_.finished(traj_t_)) {
                sp_vel_ = 0.0;
                if (motion_ == MotionState::Disabling) {
                    drive_.requestDisable();
                    pending_disable_ = false;
                }
                motion_ = MotionState::Idle;
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
            }
            return;
        default:
            return;
    }
}

void AxisRuntime::runProfilePosition(RtSinks& sinks) noexcept {
    sp_pos_ = act_pos_;
    sp_vel_ = act_vel_;
    const double eps = 0.01 * cfg_.limits.velocity_max;
    const auto axis = static_cast<std::int16_t>(index_);

    if (motion_ == MotionState::Stopping || motion_ == MotionState::Disabling) {
        mode_bits_ = static_cast<std::uint16_t>((mode_bits_ & ~cia402::kCwBit4) | cia402::kCwHalt);
        if ((statusword_ & cia402::kSwTargetReached) != 0 && std::fabs(act_vel_) <= eps) {
            if (motion_ == MotionState::Disabling) drive_.requestDisable();
            motion_ = MotionState::Idle;
            pp_target_counts_ = pos_counts_;
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
        }
        return;
    }
    if (motion_ != MotionState::Moving) return;

    switch (pp_phase_) {
        case PpPhase::WaitParams:
            if (param_error_) {
                finish(motion_cmd_, motion_type_, ErrorCode::EthercatError, sinks);
                pp_phase_ = PpPhase::Idle;
                motion_ = MotionState::Idle;
                return;
            }
            if (paramsIdle()) {
                out_target_counts_ = pp_target_counts_;
                mode_bits_ = static_cast<std::uint16_t>((mode_bits_ & ~cia402::kCwHalt) | cia402::kCwBit4 | cia402::kCwBit5);
                pp_phase_ = PpPhase::WaitAck;
            }
            break;
        case PpPhase::WaitAck:
            if ((statusword_ & cia402::kSwBit12) != 0) {
                mode_bits_ = static_cast<std::uint16_t>(mode_bits_ & ~cia402::kCwBit4);
                pp_phase_ = PpPhase::Moving;
            }
            break;
        case PpPhase::Moving:
            if ((statusword_ & cia402::kSwBit12) == 0 && (statusword_ & cia402::kSwTargetReached) != 0) {
                pp_phase_ = PpPhase::Idle;
                motion_ = MotionState::Idle;
                finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
                return;
            }
            break;
        case PpPhase::Idle:
            break;
    }
    if (motion_elapsed_ > motion_timeout_) {
        sinks.event(EventType::MotionTimeout, axis, 1, std::llround(motion_elapsed_ * 1000.0));
        finish(motion_cmd_, motion_type_, ErrorCode::Timeout, sinks);
        startStop(move_deceleration_);
    }
}

void AxisRuntime::runProfileVelocity(RtSinks& sinks) noexcept {
    sp_pos_ = act_pos_;
    sp_vel_ = act_vel_;
    const double eps = 0.01 * cfg_.limits.velocity_max;
    const AxisLimits& lim = cfg_.limits;
    const auto axis = static_cast<std::int16_t>(index_);

    if (motion_ == MotionState::Velocity) {
        if (param_error_) {
            finish(motion_cmd_, motion_type_, ErrorCode::EthercatError, sinks);
            velocity_cmd_ = 0.0;
            motion_ = MotionState::Stopping;
            return;
        }
        if (lim.soft_limits_enabled) {
            const double stop = TrapezoidalProfile::stoppingDistance(act_vel_, move_deceleration_);
            const bool hit_max = act_vel_ > 0.0 && act_pos_ + stop >= lim.position_max;
            const bool hit_min = act_vel_ < 0.0 && act_pos_ - stop <= lim.position_min;
            if (hit_max || hit_min) {
                // The drive ramps down with 0x6084; exact stop position is drive dependent.
                velocity_cmd_ = 0.0;
                at_positive_limit_ = hit_max;
                at_negative_limit_ = hit_min;
                finish(motion_cmd_, motion_type_, ErrorCode::LimitViolation, sinks);
                sinks.event(EventType::PositionLimit, axis, hit_max ? 2 : -2, 0);
                motion_ = MotionState::Stopping;
                return;
            }
        }
        if (!velocity_reached_ && paramsIdle() && std::fabs(act_vel_ - velocity_cmd_) <= eps) {
            velocity_reached_ = true;
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
            if (velocity_cmd_ == 0.0) motion_ = MotionState::Idle;
        }
        return;
    }
    if (motion_ == MotionState::Stopping || motion_ == MotionState::Disabling) {
        velocity_cmd_ = 0.0;
        if (std::fabs(act_vel_) <= eps) {
            if (motion_ == MotionState::Disabling) drive_.requestDisable();
            motion_ = MotionState::Idle;
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
        }
    }
}

void AxisRuntime::runHoming(double dt, RtSinks& sinks) noexcept {
    (void)dt;
    sp_pos_ = act_pos_;
    sp_vel_ = act_vel_;
    out_target_counts_ = pos_counts_;
    const auto axis = static_cast<std::int16_t>(index_);
    auto restore = [&](ErrorCode result) {
        homing_result_ = result;
        mode_bits_ = 0;
        requestMode(saved_mode_);
        homing_phase_ = HomingPhase::Restore;
    };

    switch (homing_phase_) {
        case HomingPhase::WaitReady:
            if (param_error_) {
                sinks.event(EventType::HomingFailed, axis, 1, 0);
                restore(ErrorCode::EthercatError);
            } else if (paramsIdle() && modeConfirmed()) {
                mode_bits_ = cia402::kCwBit4;
                homing_phase_ = HomingPhase::Running;
                motion_elapsed_ = 0.0;
            } else if (motion_elapsed_ * 1000.0 > cfg_.timeouts.mode_switch_ms) {
                sinks.event(EventType::HomingFailed, axis, 2, 0);
                restore(ErrorCode::Timeout);
            }
            break;
        case HomingPhase::Running:
            if ((statusword_ & cia402::kSwBit13) != 0) {
                sinks.event(EventType::HomingFailed, axis, 3, statusword_);
                restore(ErrorCode::DriveFault);
            } else if ((statusword_ & cia402::kSwBit12) != 0 && (statusword_ & cia402::kSwTargetReached) != 0) {
                homed_ = true;
                offset_counts_ = 0;  // the drive redefined its position (home offset 0x607C)
                sinks.event(EventType::HomingCompleted, axis, 0, pos_counts_);
                restore(ErrorCode::Ok);
            } else if (motion_elapsed_ > cfg_.homing.timeout_s) {
                sinks.event(EventType::HomingFailed, axis, 4, 0);
                restore(ErrorCode::Timeout);
            }
            break;
        case HomingPhase::Restore:
            if (modeConfirmed()) {
                homing_phase_ = HomingPhase::Idle;
                motion_ = MotionState::Idle;
                act_pos_ = countsToUnits(pos_counts_);
                holdSetpoint();
                was_inside_limits_ = true;
                finish(motion_cmd_, motion_type_, homing_result_, sinks);
            } else if (++mode_switch_cycles_ * dt_ * 1000.0 > cfg_.timeouts.mode_switch_ms) {
                homing_phase_ = HomingPhase::Idle;
                motion_ = MotionState::Idle;
                finish(motion_cmd_, motion_type_, ErrorCode::Timeout, sinks);
            }
            break;
        case HomingPhase::Idle:
            break;
    }
}

void AxisRuntime::computeCyclicOutputs(double dt) noexcept {
    const AxisLimits& lim = cfg_.limits;
    const OperationMode mode = modeConfirmed() ? requested_mode_ : OperationMode::None;
    switch (mode) {
        case OperationMode::CSP:
            out_target_counts_ = clampToInt32(unitsToCounts(sp_pos_));
            out_target_velocity_ = 0;
            out_torque_ = 0;
            break;
        case OperationMode::CSV: {
            double v = sp_vel_;
            if (cfg_.control_loop == ControlLoop::Position) {
                pid_.setSetpoint(sp_pos_);
                v += pid_.update(act_pos_, dt);
            }
            const double vmax = lim.velocity_max * (1.0 + lim.velocity_tolerance);
            v = std::clamp(v, -vmax, vmax);
            out_target_velocity_ = clampToInt32(velocityToDrive(v));
            out_target_counts_ = pos_counts_;
            break;
        }
        case OperationMode::CST: {
            double t = 0.0;
            if (cfg_.control_loop == ControlLoop::Position) {
                pid_.setSetpoint(sp_pos_);
                t = pid_.update(act_pos_, dt);
            } else if (cfg_.control_loop == ControlLoop::Velocity) {
                pid_.setSetpoint(sp_vel_);
                t = pid_.update(act_vel_, dt);
            } else if (motion_ == MotionState::Torque) {
                t = torque_cmd_;
            }
            t = std::clamp(t, -lim.torque_max, lim.torque_max);
            out_torque_ = static_cast<std::int16_t>(std::lround(t * 10.0 * cfg_.direction));
            out_target_counts_ = pos_counts_;
            break;
        }
        case OperationMode::PV:
            if (paramsIdle()) out_target_velocity_ = clampToInt32(velocityToDrive(velocity_cmd_));
            out_target_counts_ = pos_counts_;
            break;
        case OperationMode::PP:
            out_target_counts_ = pp_target_counts_;
            break;
        default:
            out_target_counts_ = pos_counts_;
            out_target_velocity_ = 0;
            out_torque_ = 0;
            break;
    }
}

void AxisRuntime::applyReaction(SafetyReaction reaction, RtSinks& sinks) noexcept {
    if (reaction == applied_reaction_) return;
    if (static_cast<std::uint8_t>(reaction) > static_cast<std::uint8_t>(applied_reaction_)) {
        if (power_cmd_.state == CommandState::Pending && power_type_ == AxisCommandType::Enable) {
            finish(power_cmd_, power_type_, ErrorCode::SafetyFault, sinks);
        }
        switch (reaction) {
            case SafetyReaction::Stop:
                if (drive_.isOperationEnabled() && motion_ != MotionState::Idle && motion_ != MotionState::Stopping &&
                    motion_ != MotionState::SwitchingMode) {
                    finish(motion_cmd_, motion_type_, ErrorCode::SafetyFault, sinks);
                    const bool disabling = motion_ == MotionState::Disabling;
                    if (motion_ == MotionState::Homing) {
                        abortMotion(ErrorCode::SafetyFault, sinks);
                    } else {
                        startStop(cfg_.limits.quick_stop_deceleration);
                        if (disabling && motion_ == MotionState::Stopping) motion_ = MotionState::Disabling;
                    }
                }
                if (drive_.request() == DriveRequest::Enable && !drive_.isOperationEnabled()) drive_.requestDisable();
                break;
            case SafetyReaction::QuickStop:
                abortMotion(ErrorCode::SafetyFault, sinks);
                if (drive_.isOperationEnabled()) {
                    drive_.requestQuickStop();
                } else {
                    drive_.requestDisable();
                }
                break;
            case SafetyReaction::DisableVoltage:
                abortMotion(ErrorCode::SafetyFault, sinks);
                drive_.requestDisableVoltage();
                break;
            case SafetyReaction::None:
                break;
        }
    }
    applied_reaction_ = reaction;
}

void AxisRuntime::updatePowerCommand(RtSinks& sinks) noexcept {
    if (drive_.transitionTimedOut() && !timeout_reported_) {
        timeout_reported_ = true;
        sinks.event(EventType::TransitionTimeout, static_cast<std::int16_t>(index_), static_cast<std::int32_t>(drive_.state()),
                    statusword_);
    } else if (!drive_.transitionTimedOut()) {
        timeout_reported_ = false;
    }
    if (power_cmd_.state != CommandState::Pending) return;
    switch (power_type_) {
        case AxisCommandType::Enable:
            if (drive_.isOperationEnabled()) finish(power_cmd_, power_type_, ErrorCode::Ok, sinks);
            else if (drive_.transitionTimedOut()) finish(power_cmd_, power_type_, ErrorCode::Timeout, sinks);
            else if (drive_.isFault()) finish(power_cmd_, power_type_, ErrorCode::DriveFault, sinks);
            else if (drive_.request() != DriveRequest::Enable) finish(power_cmd_, power_type_, ErrorCode::Aborted, sinks);
            break;
        case AxisCommandType::Disable:
        case AxisCommandType::QuickStop:
            if (motion_ != MotionState::Disabling && drive_.requestSatisfied()) {
                finish(power_cmd_, power_type_, ErrorCode::Ok, sinks);
            } else if (drive_.transitionTimedOut()) {
                finish(power_cmd_, power_type_, ErrorCode::Timeout, sinks);
            }
            break;
        case AxisCommandType::ResetFault:
            if (!drive_.faultResetPending()) {
                finish(power_cmd_, power_type_,
                       drive_.faultResetFailed() || drive_.isFault() ? ErrorCode::DriveFault : ErrorCode::Ok, sinks);
            }
            break;
        default:
            break;
    }
}

void AxisRuntime::control(const CycleContext& ctx, SafetyReaction reaction, bool permitted, RtSinks& sinks) noexcept {
    ++cycle_;
    const double dt = ctx.dt_s;

    if (!comm_ok_) {
        // Outputs cannot reach the drive. Its own SM watchdog handles the
        // stop; make sure we never re-enable automatically afterwards.
        if (drive_.request() != DriveRequest::Disable && drive_.request() != DriveRequest::DisableVoltage) {
            drive_.requestDisable();
        }
        if (motion_ != MotionState::Idle) {
            finish(motion_cmd_, motion_type_, ErrorCode::EthercatError, sinks);
            motion_ = MotionState::Idle;
        }
        homing_phase_ = HomingPhase::Idle;
        pp_phase_ = PpPhase::Idle;
        mode_bits_ = 0;
        finish(power_cmd_, power_type_, ErrorCode::EthercatError, sinks);
        initial_mode_sent_ = false;
        first_feedback_ = true;
        publish(ctx, false);
        return;
    }

    if (!initial_mode_sent_ && feedback_valid_) {
        requestMode(requested_mode_);
        initial_mode_sent_ = true;
    }

    applyReaction(reaction, sinks);
    updatePowerCommand(sinks);

    if (motion_ == MotionState::SwitchingMode) {
        holdSetpoint();
        ++mode_switch_cycles_;
        if (modeConfirmed()) {
            motion_ = MotionState::Idle;
            sinks.event(EventType::ModeChanged, static_cast<std::int16_t>(index_), static_cast<std::int32_t>(requested_mode_),
                        0);
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
        } else if (param_error_ || static_cast<double>(mode_switch_cycles_) * dt * 1000.0 > cfg_.timeouts.mode_switch_ms) {
            motion_ = MotionState::Idle;
            finish(motion_cmd_, motion_type_, param_error_ ? ErrorCode::EthercatError : ErrorCode::Timeout, sinks);
        }
    }

    if (!drive_.isOperationEnabled()) {
        if (motion_ == MotionState::Homing) {
            abortMotion(drive_.isFault() ? ErrorCode::DriveFault : ErrorCode::Aborted, sinks);
        } else if (motion_ == MotionState::Disabling) {
            motion_ = MotionState::Idle;
            finish(motion_cmd_, motion_type_, ErrorCode::Ok, sinks);
        } else if (motion_ != MotionState::Idle && motion_ != MotionState::SwitchingMode) {
            abortMotion(drive_.isFault() ? ErrorCode::DriveFault : ErrorCode::Aborted, sinks);
        }
        pp_phase_ = PpPhase::Idle;
        mode_bits_ = 0;
        velocity_cmd_ = 0.0;
        torque_cmd_ = 0.0;
        holdSetpoint();
    } else {
        motion_elapsed_ += dt;
        if (motion_ == MotionState::Homing) {
            runHoming(dt, sinks);
        } else if (motion_ != MotionState::SwitchingMode && modeConfirmed()) {
            switch (requested_mode_) {
                case OperationMode::PP: runProfilePosition(sinks); break;
                case OperationMode::PV: runProfileVelocity(sinks); break;
                default: runTrajectory(dt, sinks); break;
            }
        }
        if (cfg_.motion.command_watchdog_ms > 0 &&
            (motion_ == MotionState::Velocity || motion_ == MotionState::Torque)) {
            watchdog_elapsed_ += dt;
            if (watchdog_elapsed_ * 1000.0 > cfg_.motion.command_watchdog_ms) {
                sinks.event(EventType::MotionTimeout, static_cast<std::int16_t>(index_), 2,
                            std::llround(watchdog_elapsed_ * 1000.0));
                startStop(cfg_.motion.deceleration);
            }
        }
        if (motion_ != MotionState::Homing) computeCyclicOutputs(dt);
    }

    if (backend_ != nullptr) stepParams(*backend_, sinks);
    // Commands and safety reactions of this cycle take effect immediately,
    // also when no fresh feedback arrived.
    drive_.recompute();
    writeOutputs(ctx.process_data);
    publish(ctx, permitted);
}

void AxisRuntime::writeOutputs(std::uint8_t* pd) noexcept {
    if (pd == nullptr) return;
    std::uint16_t cw = drive_.controlword();
    if (drive_.isOperationEnabled()) {
        cw = static_cast<std::uint16_t>(cw | (mode_bits_ & cia402::kCwModeSpecificMask));
    }
    last_controlword_ = cw;
    pd::writeEntry(pd, cw_.loc, cw);
    if (target_pos_.present) pd::writeEntry(pd, target_pos_.loc, out_target_counts_);
    if (target_vel_.present) pd::writeEntry(pd, target_vel_.loc, out_target_velocity_);
    if (target_torque_.present) pd::writeEntry(pd, target_torque_.loc, out_torque_);
    for (const ParamSlot& slot : params_) {
        if (slot.pdo.present && slot.pdo_valid) pd::writeEntry(pd, slot.pdo.loc, slot.pdo_value);
    }
}

void AxisRuntime::publish(const CycleContext& ctx, bool permitted) noexcept {
    const AxisLimits& lim = cfg_.limits;
    AxisStatus s;
    s.name = name_;
    s.index = static_cast<std::uint16_t>(index_);
    s.drive_state = drive_.state();
    s.mode = requested_mode_;
    s.mode_display = mode_display_.present ? static_cast<OperationMode>(mode_display_value_)
                                           : (modeConfirmed() ? requested_mode_ : OperationMode::None);
    s.motion_state = motion_;
    s.comm_ok = comm_ok_;
    s.enabled = drive_.isOperationEnabled();
    s.fault = drive_.isFault();
    s.warning = (statusword_ & cia402::kSwWarning) != 0;
    s.moving = motion_ == MotionState::Moving || motion_ == MotionState::Velocity ||
               motion_ == MotionState::Stopping || motion_ == MotionState::Homing ||
               motion_ == MotionState::Disabling;
    s.motion_done = !s.moving && motion_ != MotionState::SwitchingMode;
    s.homed = homed_;
    s.target_reached = (statusword_ & cia402::kSwTargetReached) != 0;
    s.at_positive_limit = lim.soft_limits_enabled && (at_positive_limit_ || act_pos_ >= lim.position_max);
    s.at_negative_limit = lim.soft_limits_enabled && (at_negative_limit_ || act_pos_ <= lim.position_min);
    s.motion_permitted = permitted;
    s.statusword = statusword_;
    s.controlword = last_controlword_;
    s.error_code = error_code_value_;
    s.position = act_pos_;
    s.velocity = act_vel_;
    s.torque = act_torque_;
    s.setpoint_position = sp_pos_;
    s.setpoint_velocity = sp_vel_;
    s.target_position = target_;
    s.following_error = sp_pos_ - act_pos_;
    s.position_counts = pos_counts_;
    s.power_command = power_cmd_;
    s.motion_command = motion_cmd_;
    s.config_command = config_cmd_;
    s.last_rejected = last_rejected_;
    s.cycle = cycle_;
    s.timestamp_ns = ctx.now_ns;
    status_.store(s);
}

}  // namespace ethercat_motion
