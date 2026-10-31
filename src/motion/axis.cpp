// SPDX-License-Identifier: MIT
#include "ethercat_motion/axis.hpp"

#include <thread>

#include "ethercat_motion/motion_controller.hpp"

namespace ethercat_motion {

const char* toString(AxisCommandType type) noexcept {
    switch (type) {
        case AxisCommandType::Enable: return "enable";
        case AxisCommandType::Disable: return "disable";
        case AxisCommandType::QuickStop: return "quick_stop";
        case AxisCommandType::ResetFault: return "reset_fault";
        case AxisCommandType::SetMode: return "set_mode";
        case AxisCommandType::MoveAbsolute: return "move_absolute";
        case AxisCommandType::MoveRelative: return "move_relative";
        case AxisCommandType::MoveVelocity: return "set_velocity";
        case AxisCommandType::SetTorque: return "set_torque";
        case AxisCommandType::Stop: return "stop";
        case AxisCommandType::Home: return "home";
        case AxisCommandType::SetPid: return "set_pid";
        case AxisCommandType::SetPosition: return "set_position";
    }
    return "unknown";
}

CommandSlot slotOf(AxisCommandType type) noexcept {
    switch (type) {
        case AxisCommandType::Enable:
        case AxisCommandType::Disable:
        case AxisCommandType::QuickStop:
        case AxisCommandType::ResetFault:
            return CommandSlot::Power;
        case AxisCommandType::SetPid:
        case AxisCommandType::SetPosition:
            return CommandSlot::Config;
        default:
            return CommandSlot::Motion;
    }
}

const char* toString(MotionState state) noexcept {
    switch (state) {
        case MotionState::Idle: return "idle";
        case MotionState::Moving: return "moving";
        case MotionState::Velocity: return "velocity";
        case MotionState::Torque: return "torque";
        case MotionState::Stopping: return "stopping";
        case MotionState::Homing: return "homing";
        case MotionState::SwitchingMode: return "switching_mode";
        case MotionState::Disabling: return "disabling";
    }
    return "unknown";
}

Axis::Axis(MotionController* owner, std::size_t index, const AxisConfig* config)
    : owner_(owner), index_(index), config_(config), name_(config->name) {}

Status Axis::submit(AxisCommand command) {
    command.axis = static_cast<std::uint16_t>(index_);
    auto id = owner_->submit(command);
    if (!id) {
        return id.status();
    }
    last_slot_.store(static_cast<std::uint8_t>(slotOf(command.type)));
    last_id_.store(id.value());
    return Status::ok();
}

Status Axis::enable() {
    AxisCommand c;
    c.type = AxisCommandType::Enable;
    return submit(c);
}

Status Axis::disable() {
    AxisCommand c;
    c.type = AxisCommandType::Disable;
    return submit(c);
}

Status Axis::quickStop() {
    AxisCommand c;
    c.type = AxisCommandType::QuickStop;
    return submit(c);
}

Status Axis::resetFault() {
    AxisCommand c;
    c.type = AxisCommandType::ResetFault;
    return submit(c);
}

Status Axis::setMode(OperationMode mode) {
    AxisCommand c;
    c.type = AxisCommandType::SetMode;
    c.mode = mode;
    return submit(c);
}

Status Axis::moveAbsolute(double position, const MoveParams& params) {
    AxisCommand c;
    c.type = AxisCommandType::MoveAbsolute;
    c.value = position;
    c.velocity = params.velocity;
    c.acceleration = params.acceleration;
    c.deceleration = params.deceleration;
    return submit(c);
}

Status Axis::moveRelative(double distance, const MoveParams& params) {
    AxisCommand c;
    c.type = AxisCommandType::MoveRelative;
    c.value = distance;
    c.velocity = params.velocity;
    c.acceleration = params.acceleration;
    c.deceleration = params.deceleration;
    return submit(c);
}

Status Axis::moveVelocity(double velocity, double acceleration) {
    AxisCommand c;
    c.type = AxisCommandType::MoveVelocity;
    c.value = velocity;
    c.acceleration = acceleration;
    return submit(c);
}

Status Axis::setTorque(double torque_percent) {
    AxisCommand c;
    c.type = AxisCommandType::SetTorque;
    c.value = torque_percent;
    return submit(c);
}

Status Axis::stop(double deceleration) {
    AxisCommand c;
    c.type = AxisCommandType::Stop;
    c.deceleration = deceleration;
    return submit(c);
}

Status Axis::home() {
    AxisCommand c;
    c.type = AxisCommandType::Home;
    return submit(c);
}

Status Axis::setPosition(double position) {
    AxisCommand c;
    c.type = AxisCommandType::SetPosition;
    c.value = position;
    return submit(c);
}

Status Axis::setPid(double kp, double ki, double kd) {
    AxisCommand c;
    c.type = AxisCommandType::SetPid;
    c.value = kp;
    c.velocity = ki;
    c.acceleration = kd;
    return submit(c);
}

AxisStatus Axis::status() const noexcept { return owner_->axisStatus(index_); }

Status Axis::wait(std::chrono::milliseconds timeout) const {
    const std::uint64_t id = last_id_.load();
    if (id == 0) {
        return Status::ok();
    }
    return waitForCommand(id, static_cast<CommandSlot>(last_slot_.load()), timeout);
}

Status Axis::waitForCommand(std::uint64_t id, CommandSlot slot, std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        const AxisStatus s = status();
        if (s.last_rejected.id == id) {
            return {s.last_rejected.result, "axis '" + name_ + "': command rejected"};
        }
        const CommandTracking& t = slot == CommandSlot::Power    ? s.power_command
                                   : slot == CommandSlot::Motion ? s.motion_command
                                                                 : s.config_command;
        if (t.id == id) {
            if (t.state == CommandState::Completed) return Status::ok();
            if (t.state == CommandState::Failed) {
                return {t.result, "axis '" + name_ + "': command failed (" + std::string(toString(t.result)) + ")"};
            }
        } else if (t.id > id) {
            return {ErrorCode::Aborted, "axis '" + name_ + "': command superseded by a newer command"};
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return {ErrorCode::Timeout, "axis '" + name_ + "': timeout waiting for command completion"};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}  // namespace ethercat_motion
