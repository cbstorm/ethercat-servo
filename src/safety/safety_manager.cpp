// SPDX-License-Identifier: MIT
#include "ethercat_motion/safety_manager.hpp"

#include <algorithm>
#include <cstring>

namespace ethercat_motion {

namespace {

constexpr std::int64_t kResetWindowNs = 2'000'000'000;  // drive fault reset may take a while

SafetyReaction maxReaction(SafetyReaction a, SafetyReaction b) noexcept {
    return static_cast<std::uint8_t>(a) >= static_cast<std::uint8_t>(b) ? a : b;
}

struct FaultName {
    std::uint32_t bit;
    const char* name;
};

constexpr FaultName kFaultNames[] = {
    {safety_fault::kEmergencyStop, "emergency_stop"},
    {safety_fault::kCommunicationLost, "communication_lost"},
    {safety_fault::kWorkingCounter, "working_counter"},
    {safety_fault::kSlaveNotOperational, "slave_not_operational"},
    {safety_fault::kDeadlineOverrun, "deadline_overrun"},
    {safety_fault::kDriveFault, "drive_fault"},
    {safety_fault::kPositionLimit, "position_limit"},
    {safety_fault::kVelocityLimit, "velocity_limit"},
    {safety_fault::kFollowingError, "following_error"},
    {safety_fault::kHeartbeatTimeout, "heartbeat_timeout"},
    {safety_fault::kTransitionTimeout, "transition_timeout"},
};

}  // namespace

void describeSafetyFaults(std::uint32_t faults, char* buffer, std::size_t size) noexcept {
    if (buffer == nullptr || size == 0) return;
    buffer[0] = '\0';
    std::size_t used = 0;
    for (const auto& f : kFaultNames) {
        if ((faults & f.bit) == 0) continue;
        const std::size_t len = std::strlen(f.name);
        const std::size_t need = len + (used > 0 ? 1 : 0);
        if (used + need + 1 > size) break;
        if (used > 0) buffer[used++] = ',';
        std::memcpy(buffer + used, f.name, len);
        used += len;
        buffer[used] = '\0';
    }
}

SafetyManager::SafetyManager(const SafetyConfig& config) noexcept : config_(config) {
    axis_reaction_.fill(SafetyReaction::None);
    status_.store(SafetyStatus{});
}

void SafetyManager::requestEmergencyStop() noexcept {
    estop_input_.store(true);
    estop_latched_request_.store(true);
}

void SafetyManager::releaseEmergencyStop() noexcept { estop_input_.store(false); }

void SafetyManager::requestReset() noexcept { reset_request_.store(true); }

void SafetyManager::heartbeat(std::int64_t now_ns) noexcept { last_heartbeat_ns_.store(now_ns); }

SafetyReaction SafetyManager::reactionFor(std::uint32_t fault) const noexcept {
    using namespace safety_fault;
    if (fault & kEmergencyStop) return config_.emergency_stop_reaction;
    if (fault & (kCommunicationLost | kWorkingCounter | kSlaveNotOperational)) {
        return config_.communication_loss_reaction;
    }
    if (fault & kDeadlineOverrun) return config_.overrun_reaction;
    if (fault & kHeartbeatTimeout) return SafetyReaction::Stop;
    return SafetyReaction::None;
}

void SafetyManager::pushEvent(EventType type, std::int16_t axis, std::int32_t code, std::int64_t value,
                              std::int64_t now) noexcept {
    if (events_ == nullptr) return;
    Event e;
    e.time_ns = now;
    e.type = type;
    e.axis = axis;
    e.code = code;
    e.value = value;
    (void)events_->tryPush(e);
}

void SafetyManager::latch(std::uint32_t fault, std::int16_t axis, std::int64_t now) noexcept {
    if (latched_ == 0 && first_fault_ == 0) {
        first_fault_ = fault;
        first_fault_axis_ = axis;
        first_fault_time_ = now;
    }
    latched_ |= fault;
    pushEvent(EventType::SafetyFault, axis, static_cast<std::int32_t>(fault), 0, now);
}

void SafetyManager::evaluate(const SafetyInputs& in) noexcept {
    using namespace safety_fault;
    const std::int64_t now = in.now_ns;
    const std::size_t axis_count = std::min(in.axis_count, kMaxAxes);

    // ---- Current conditions ---------------------------------------------
    std::uint32_t active = 0;
    if (estop_latched_request_.exchange(false)) {
        if ((latched_ & kEmergencyStop) == 0) {
            pushEvent(EventType::EmergencyStop, -1, 1, 0, now);
            latch(kEmergencyStop, -1, now);
        }
    }
    if (estop_input_.load()) active |= kEmergencyStop;

    if (in.comm != nullptr) {
        const CommStatus& c = *in.comm;
        comm_ready_ = c.reached_operational && c.link_up;
        // Communication supervision starts once the bus reached OP.
        if (c.reached_operational) {
            if (!c.link_up) active |= kCommunicationLost;
            if (config_.working_counter_error_cycles > 0 &&
                c.consecutive_wc_errors >= config_.working_counter_error_cycles) {
                active |= kWorkingCounter;
            }
            if (!c.all_operational) active |= kSlaveNotOperational;
        }
    }
    if (in.timing != nullptr && config_.max_consecutive_overruns > 0 &&
        in.timing->consecutive_overruns >= config_.max_consecutive_overruns) {
        active |= kDeadlineOverrun;
    }
    if (config_.heartbeat_timeout_ms > 0) {
        const std::int64_t last = last_heartbeat_ns_.load();
        if (last != 0 && now - last > static_cast<std::int64_t>(config_.heartbeat_timeout_ms) * 1'000'000) {
            active |= kHeartbeatTimeout;
        }
    }

    std::array<std::uint32_t, kMaxAxes> axis_active{};
    std::uint32_t any_axis_active = 0;
    for (std::size_t i = 0; i < axis_count; ++i) {
        const SafetyAxisInput& a = in.axes[i];
        std::uint32_t bits = 0;
        if (a.drive_fault) bits |= kDriveFault;
        if (a.position_limit) bits |= kPositionLimit;
        if (a.velocity_limit) bits |= kVelocityLimit;
        if (a.following_error) bits |= kFollowingError;
        if (a.transition_timeout) bits |= kTransitionTimeout;
        axis_active[i] = bits;
        any_axis_active |= bits;
    }

    // ---- Latch new faults -------------------------------------------------
    const std::uint32_t new_global = active & ~latched_;
    for (const auto& f : kFaultNames) {
        if (new_global & f.bit) latch(f.bit, -1, now);
    }
    for (std::size_t i = 0; i < axis_count; ++i) {
        const std::uint32_t fresh = axis_active[i] & ~axis_latched_[i];
        if (fresh != 0) {
            axis_latched_[i] |= fresh;
            latch(fresh, static_cast<std::int16_t>(i), now);
        }
    }

    // ---- Reset --------------------------------------------------------------
    if (reset_request_.exchange(false)) {
        reset_deadline_ns_ = now + kResetWindowNs;
        reset_blocked_ = false;
    }
    if (reset_deadline_ns_ != 0) {
        if (active == 0 && any_axis_active == 0) {
            if (latched_ != 0) {
                ++reset_count_;
                pushEvent(EventType::SafetyReset, -1, static_cast<std::int32_t>(latched_), 0, now);
            }
            latched_ = 0;
            axis_latched_.fill(0);
            first_fault_ = 0;
            first_fault_axis_ = -1;
            first_fault_time_ = 0;
            reset_deadline_ns_ = 0;
        } else if (now >= reset_deadline_ns_) {
            reset_blocked_ = true;
            reset_deadline_ns_ = 0;
        }
    }

    // ---- Reactions ----------------------------------------------------------
    SafetyReaction global = SafetyReaction::None;
    for (const auto& f : kFaultNames) {
        if ((latched_ & f.bit) && (f.bit & kAxisLocalMask) == 0) global = maxReaction(global, reactionFor(f.bit));
    }
    bool any_axis_latched = false;
    for (std::size_t i = 0; i < axis_count; ++i) any_axis_latched = any_axis_latched || axis_latched_[i] != 0;

    for (std::size_t i = 0; i < axis_count; ++i) {
        SafetyReaction r = global;
        const std::uint32_t own = axis_latched_[i];
        if (own & (kPositionLimit | kVelocityLimit | kFollowingError)) r = maxReaction(r, config_.limit_reaction);
        if (own & (kDriveFault | kTransitionTimeout)) r = maxReaction(r, SafetyReaction::DisableVoltage);
        if (own == 0 && any_axis_latched && config_.stop_all_on_axis_fault) {
            r = maxReaction(r, config_.drive_fault_reaction);
        }
        axis_reaction_[i] = r;
    }

    SafetyStatus s;
    s.active_faults = active | any_axis_active;
    s.latched_faults = latched_;
    s.first_fault = first_fault_;
    s.first_fault_axis = first_fault_axis_;
    s.first_fault_time_ns = first_fault_time_;
    s.axis_faults = axis_latched_;
    s.axis_reaction = axis_reaction_;
    s.emergency_stop_active = (active & kEmergencyStop) != 0;
    s.reset_blocked = reset_blocked_;
    s.reset_count = reset_count_;
    s.communication_ready = comm_ready_;
    status_.store(s);
}

SafetyReaction SafetyManager::axisReaction(std::size_t axis) const noexcept {
    return axis < kMaxAxes ? axis_reaction_[axis] : SafetyReaction::DisableVoltage;
}

bool SafetyManager::motionPermitted(std::size_t axis) const noexcept {
    if (axis >= kMaxAxes) return false;
    return (latched_ & ~safety_fault::kAxisLocalMask) == 0 && axis_latched_[axis] == 0 &&
           axis_reaction_[axis] == SafetyReaction::None;
}

}  // namespace ethercat_motion
