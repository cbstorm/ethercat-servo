// SPDX-License-Identifier: MIT
#include "ethercat_motion/motion_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "axis_runtime.hpp"

namespace ethercat_motion {

MotionController::MotionController() : MotionController(nullptr) {}

MotionController::MotionController(SafetyManager* safety) : safety_(safety != nullptr ? safety : &own_safety_) {}

MotionController::~MotionController() = default;

Status MotionController::addAxis(const std::string& name, const AxisConfig& config) {
    if (attached_) {
        return {ErrorCode::InvalidState, "axes must be added before attach()"};
    }
    if (findAxis(name) != nullptr) {
        return {ErrorCode::InvalidArgument, "duplicate axis name '" + name + "'"};
    }
    if (axes_.size() >= kMaxAxes) {
        return {ErrorCode::LimitViolation, "too many axes (max " + std::to_string(kMaxAxes) + ")"};
    }
    auto stored = std::make_unique<AxisConfig>(config);
    stored->name = name;
    const std::size_t index = axes_.size();
    runtimes_.push_back(std::make_unique<AxisRuntime>(index, *stored));
    axes_.push_back(std::unique_ptr<Axis>(new Axis(this, index, stored.get())));
    configs_.push_back(std::move(stored));
    return Status::ok();
}

Axis& MotionController::getAxis(const std::string& name) {
    if (Axis* a = findAxis(name)) return *a;
    throw std::out_of_range("unknown axis '" + name + "'");
}

const Axis& MotionController::getAxis(const std::string& name) const {
    for (const auto& a : axes_) {
        if (a->name() == name) return *a;
    }
    throw std::out_of_range("unknown axis '" + name + "'");
}

Axis* MotionController::findAxis(const std::string& name) noexcept {
    for (auto& a : axes_) {
        if (a->name() == name) return a.get();
    }
    return nullptr;
}

std::vector<std::string> MotionController::axisNames() const {
    std::vector<std::string> names;
    for (const auto& a : axes_) names.push_back(a->name());
    return names;
}

namespace {

template <typename Fn>
Status forAll(std::vector<std::unique_ptr<Axis>>& axes, Fn fn) {
    Status first;
    for (auto& a : axes) {
        Status s = fn(*a);
        if (!s && first.isOk()) first = s;
    }
    return first;
}

}  // namespace

Status MotionController::enableAll() { return forAll(axes_, [](Axis& a) { return a.enable(); }); }
Status MotionController::disableAll() { return forAll(axes_, [](Axis& a) { return a.disable(); }); }
Status MotionController::stopAll() { return forAll(axes_, [](Axis& a) { return a.stop(); }); }
Status MotionController::quickStopAll() { return forAll(axes_, [](Axis& a) { return a.quickStop(); }); }
Status MotionController::resetFaultAll() { return forAll(axes_, [](Axis& a) { return a.resetFault(); }); }

Status MotionController::moveAbsolute(const std::string& axis, double position, const MoveParams& params) {
    Axis* a = findAxis(axis);
    return a != nullptr ? a->moveAbsolute(position, params) : Status(ErrorCode::NotFound, "unknown axis '" + axis + "'");
}

Status MotionController::moveRelative(const std::string& axis, double distance, const MoveParams& params) {
    Axis* a = findAxis(axis);
    return a != nullptr ? a->moveRelative(distance, params) : Status(ErrorCode::NotFound, "unknown axis '" + axis + "'");
}

Status MotionController::moveVelocity(const std::string& axis, double velocity, double acceleration) {
    Axis* a = findAxis(axis);
    return a != nullptr ? a->moveVelocity(velocity, acceleration)
                        : Status(ErrorCode::NotFound, "unknown axis '" + axis + "'");
}

Status MotionController::stop(const std::string& axis, double deceleration) {
    Axis* a = findAxis(axis);
    return a != nullptr ? a->stop(deceleration) : Status(ErrorCode::NotFound, "unknown axis '" + axis + "'");
}

Status MotionController::waitAllIdle(std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        bool idle = true;
        for (std::size_t i = 0; i < runtimes_.size(); ++i) {
            const AxisStatus s = runtimes_[i]->status();
            idle = idle && !s.moving && s.motion_state != MotionState::SwitchingMode;
        }
        if (idle) return Status::ok();
        if (std::chrono::steady_clock::now() >= deadline) {
            return {ErrorCode::Timeout, "axes did not become idle in time"};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

Status MotionController::validate(const AxisCommand& c) const {
    if (c.axis >= configs_.size()) {
        return {ErrorCode::NotFound, "invalid axis index " + std::to_string(c.axis)};
    }
    const AxisConfig& cfg = *configs_[c.axis];
    const AxisLimits& lim = cfg.limits;
    const std::string who = "axis '" + cfg.name + "': ";
    for (double v : {c.value, c.velocity, c.acceleration, c.deceleration}) {
        if (!std::isfinite(v)) return {ErrorCode::InvalidArgument, who + "non-finite argument"};
    }
    auto check_profile = [&]() -> Status {
        if (c.velocity < 0.0 || c.velocity > lim.velocity_max) {
            return {ErrorCode::LimitViolation, who + "velocity must be within (0, " + std::to_string(lim.velocity_max) + "]"};
        }
        if (c.acceleration < 0.0 || c.acceleration > lim.acceleration_max) {
            return {ErrorCode::LimitViolation, who + "acceleration exceeds acceleration_max"};
        }
        if (c.deceleration < 0.0 || c.deceleration > lim.deceleration_max) {
            return {ErrorCode::LimitViolation, who + "deceleration exceeds deceleration_max"};
        }
        return Status::ok();
    };

    switch (c.type) {
        case AxisCommandType::MoveAbsolute:
            if (lim.soft_limits_enabled && (c.value < lim.position_min || c.value > lim.position_max)) {
                return {ErrorCode::LimitViolation, who + "target " + std::to_string(c.value) + " outside soft limits [" +
                                                       std::to_string(lim.position_min) + ", " +
                                                       std::to_string(lim.position_max) + "]"};
            }
            return check_profile();
        case AxisCommandType::MoveRelative:
            if (lim.soft_limits_enabled && std::fabs(c.value) > lim.position_max - lim.position_min) {
                return {ErrorCode::LimitViolation, who + "distance larger than the soft-limit range"};
            }
            return check_profile();
        case AxisCommandType::MoveVelocity:
            if (std::fabs(c.value) > lim.velocity_max) {
                return {ErrorCode::LimitViolation, who + "|velocity| exceeds velocity_max"};
            }
            if (c.acceleration < 0.0 || c.acceleration > lim.acceleration_max) {
                return {ErrorCode::LimitViolation, who + "acceleration exceeds acceleration_max"};
            }
            return Status::ok();
        case AxisCommandType::SetTorque:
            if (std::fabs(c.value) > lim.torque_max) {
                return {ErrorCode::LimitViolation, who + "|torque| exceeds torque_max"};
            }
            return Status::ok();
        case AxisCommandType::Stop:
            if (c.deceleration < 0.0) return {ErrorCode::InvalidArgument, who + "deceleration must be >= 0"};
            return Status::ok();
        case AxisCommandType::SetMode: {
            if (c.mode == OperationMode::None || c.mode == OperationMode::Homing) {
                return {ErrorCode::InvalidArgument, who + "use home() for homing; mode must be PP/PV/CSP/CSV/CST"};
            }
            const bool allowed = c.mode == cfg.operation_mode ||
                                 std::find(cfg.allowed_modes.begin(), cfg.allowed_modes.end(), c.mode) !=
                                     cfg.allowed_modes.end();
            if (!allowed) {
                return {ErrorCode::NotSupported, who + "mode " + toString(c.mode) + " not in allowed_modes"};
            }
            return Status::ok();
        }
        case AxisCommandType::Home:
            if (cfg.homing.method == 0) return {ErrorCode::NotSupported, who + "homing not configured"};
            return Status::ok();
        case AxisCommandType::SetPid:
            if (c.value < 0.0 || c.velocity < 0.0 || c.acceleration < 0.0) {
                return {ErrorCode::InvalidArgument, who + "PID gains must be >= 0"};
            }
            return Status::ok();
        default:
            return Status::ok();
    }
}

Result<std::uint64_t> MotionController::submit(AxisCommand command) {
    Status status = validate(command);
    if (!status) return status;
    command.id = next_id_.fetch_add(1);
    if (!commands_.tryPush(command)) {
        return Status(ErrorCode::QueueFull, "motion command queue full");
    }
    return command.id;
}

Result<std::vector<std::uint64_t>> MotionController::submitBatch(std::vector<AxisCommand> commands) {
    if (commands.empty()) return std::vector<std::uint64_t>{};
    if (commands.size() > kMaxAxes) {
        return Status(ErrorCode::InvalidArgument, "batch larger than " + std::to_string(kMaxAxes) + " commands");
    }
    std::vector<std::uint64_t> ids;
    for (AxisCommand& c : commands) {
        Status status = validate(c);
        if (!status) return status;
        c.batch_size = static_cast<std::uint16_t>(commands.size());
    }
    for (AxisCommand& c : commands) {
        c.id = next_id_.fetch_add(1);
        ids.push_back(c.id);
    }
    if (!commands_.tryPushAll(commands.data(), commands.size())) {
        return Status(ErrorCode::QueueFull, "motion command queue full");
    }
    return ids;
}

AxisStatus MotionController::axisStatus(std::size_t index) const {
    AxisStatus s = runtimes_.at(index)->status();
    if (index < kMaxAxes) s.safety_faults = safety_->status().axis_faults[index];
    return s;
}

Status MotionController::attach(EthercatMaster& master) {
    if (attached_) return {ErrorCode::InvalidState, "already attached"};
    if (axes_.empty()) return {ErrorCode::InvalidConfig, "no axes"};
    for (const auto& cfg : configs_) {
        auto handle = master.addSlave(toSlaveDeviceConfig(*cfg));
        if (!handle) return handle.status();
        slave_handles_.push_back(handle.value());
    }
    attached_ = true;
    return Status::ok();
}

Status MotionController::resolve(EthercatMaster& master) {
    if (!attached_) return {ErrorCode::InvalidState, "attach() first"};
    for (std::size_t i = 0; i < runtimes_.size(); ++i) {
        const int slave = slave_handles_[i];
        Status s = runtimes_[i]->resolve(master.layout(slave), master.backend(), master.backendHandle(slave),
                                         master.config().cycle_time_us);
        if (!s) return s;
    }
    resolved_ = true;
    return Status::ok();
}

void MotionController::onCycle(const CycleContext& ctx) noexcept {
    if (!resolved_) return;
    RtSinks sinks;
    sinks.events = events_;
    sinks.results = &results_;
    sinks.now_ns = ctx.now_ns;
    const std::size_t n = runtimes_.size();

    // 1. Inputs, drive state machines and local supervision.
    for (std::size_t i = 0; i < n; ++i) {
        const auto slave = static_cast<unsigned>(slave_handles_[i]);
        const bool slave_ok = ctx.comm != nullptr && ctx.comm->link_up && slave < 32 &&
                              ((ctx.comm->operational_mask_lo >> slave) & 1U) != 0;
        runtimes_[i]->readInputs(ctx, slave_ok, sinks);
    }

    // 2. Supervisory commands (bounded per cycle).
    // Batches are pushed contiguously; their members are applied together
    // once complete (possibly one cycle later), never partially.
    AxisCommand command;
    for (int k = 0; k < 32 && commands_.tryPop(command); ++k) {
        if (command.axis >= n) continue;
        if (command.batch_size > 1) {
            batch_[batch_fill_++] = command;
            if (batch_fill_ < command.batch_size && batch_fill_ < batch_.size()) continue;
            for (std::size_t b = 0; b < batch_fill_; ++b) {
                const AxisCommand& member = batch_[b];
                runtimes_[member.axis]->handleCommand(member, safety_->motionPermitted(member.axis), sinks);
            }
            batch_fill_ = 0;
            continue;
        }
        runtimes_[command.axis]->handleCommand(command, safety_->motionPermitted(command.axis), sinks);
    }

    // 3. Safety supervision.
    SafetyInputs inputs;
    inputs.comm = ctx.comm;
    inputs.timing = ctx.timing;
    inputs.axis_count = n;
    inputs.now_ns = ctx.now_ns;
    for (std::size_t i = 0; i < n && i < kMaxAxes; ++i) inputs.axes[i] = runtimes_[i]->safetyInput();
    safety_->evaluate(inputs);

    // 4. Motion, PID and outputs.
    for (std::size_t i = 0; i < n; ++i) {
        runtimes_[i]->control(ctx, safety_->axisReaction(i), safety_->motionPermitted(i), sinks);
    }
}

}  // namespace ethercat_motion
