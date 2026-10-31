// SPDX-License-Identifier: MIT
//
// Software safety supervision: fault detection, latching and reactions.
//
// !!! IMPORTANT !!!
// This is NOT a safety function in the sense of ISO 13849 / IEC 62061 /
// IEC 61800-5-2. It does not replace a certified safety PLC, a safety-rated
// STO circuit, hard-wired emergency-stop hardware or any other machinery
// safety system required by a risk assessment. It is an additional,
// non-safety-rated layer that brings the machine to a defined state on
// detected software-level faults.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ethercat_motion/config.hpp"
#include "ethercat_motion/diagnostics.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/lockfree.hpp"

namespace ethercat_motion {

inline constexpr std::size_t kMaxAxes = 16;

namespace safety_fault {
inline constexpr std::uint32_t kEmergencyStop = 1U << 0;
inline constexpr std::uint32_t kCommunicationLost = 1U << 1;
inline constexpr std::uint32_t kWorkingCounter = 1U << 2;
inline constexpr std::uint32_t kSlaveNotOperational = 1U << 3;
inline constexpr std::uint32_t kDeadlineOverrun = 1U << 4;
inline constexpr std::uint32_t kDriveFault = 1U << 5;
inline constexpr std::uint32_t kPositionLimit = 1U << 6;
inline constexpr std::uint32_t kVelocityLimit = 1U << 7;
inline constexpr std::uint32_t kFollowingError = 1U << 8;
inline constexpr std::uint32_t kHeartbeatTimeout = 1U << 9;
inline constexpr std::uint32_t kTransitionTimeout = 1U << 10;

inline constexpr std::uint32_t kAxisLocalMask =
    kDriveFault | kPositionLimit | kVelocityLimit | kFollowingError | kTransitionTimeout;
}  // namespace safety_fault

/// Writes a comma separated list of fault names into `buffer`.
void describeSafetyFaults(std::uint32_t faults, char* buffer, std::size_t size) noexcept;

/// Per-axis conditions evaluated by the motion layer each cycle.
struct SafetyAxisInput {
    bool drive_fault = false;
    bool position_limit = false;
    bool velocity_limit = false;
    bool following_error = false;
    bool transition_timeout = false;
};

struct SafetyInputs {
    const CommStatus* comm = nullptr;
    const CycleStats* timing = nullptr;
    std::array<SafetyAxisInput, kMaxAxes> axes{};
    std::size_t axis_count = 0;
    std::int64_t now_ns = 0;
};

struct SafetyStatus {
    std::uint32_t active_faults = 0;   ///< Conditions present right now.
    std::uint32_t latched_faults = 0;  ///< Latched until reset; motion blocked while non-zero.
    std::uint32_t first_fault = 0;     ///< First fault since the last reset.
    std::int16_t first_fault_axis = -1;
    std::int64_t first_fault_time_ns = 0;
    std::array<std::uint32_t, kMaxAxes> axis_faults{};   ///< Latched axis-local faults.
    std::array<SafetyReaction, kMaxAxes> axis_reaction{};
    bool emergency_stop_active = false;
    bool reset_blocked = false;        ///< Last reset refused: a condition is still present.
    std::uint64_t reset_count = 0;
    bool communication_ready = false;  ///< All slaves reached OP once and link is up.
};

class SafetyManager {
public:
    explicit SafetyManager(const SafetyConfig& config = {}) noexcept;

    // ---- Any thread ------------------------------------------------------
    /// Latches an emergency stop request. Software E-stop is supervisory
    /// only and not a substitute for hard-wired E-stop circuits.
    void requestEmergencyStop() noexcept;
    /// Releases the E-stop *input*; the latched fault still needs reset.
    void releaseEmergencyStop() noexcept;
    /// Requests clearing of latched faults (refused while conditions persist).
    void requestReset() noexcept;
    /// Application heartbeat for the heartbeat watchdog.
    void heartbeat(std::int64_t now_ns) noexcept;
    SafetyStatus status() const noexcept { return status_.load(); }
    const SafetyConfig& config() const noexcept { return config_; }

    // ---- Cyclic task -----------------------------------------------------
    void evaluate(const SafetyInputs& inputs) noexcept;
    SafetyReaction axisReaction(std::size_t axis) const noexcept;
    bool motionPermitted(std::size_t axis) const noexcept;
    std::uint32_t latchedFaults() const noexcept { return latched_; }
    void setEventRing(EventRing* ring) noexcept { events_ = ring; }

private:
    void latch(std::uint32_t fault, std::int16_t axis, std::int64_t now) noexcept;
    SafetyReaction reactionFor(std::uint32_t fault) const noexcept;
    void pushEvent(EventType type, std::int16_t axis, std::int32_t code, std::int64_t value, std::int64_t now) noexcept;

    SafetyConfig config_;
    std::atomic<bool> estop_input_{false};
    std::atomic<bool> estop_latched_request_{false};
    std::atomic<bool> reset_request_{false};
    std::atomic<std::int64_t> last_heartbeat_ns_{0};

    // Cyclic-task state.
    std::uint32_t latched_ = 0;
    std::uint32_t first_fault_ = 0;
    std::int16_t first_fault_axis_ = -1;
    std::int64_t first_fault_time_ = 0;
    std::array<std::uint32_t, kMaxAxes> axis_latched_{};
    std::array<SafetyReaction, kMaxAxes> axis_reaction_{};
    std::uint64_t reset_count_ = 0;
    std::int64_t reset_deadline_ns_ = 0;  ///< Pending reset retried until conditions clear.
    bool reset_blocked_ = false;
    bool comm_ready_ = false;
    EventRing* events_ = nullptr;

    SeqLock<SafetyStatus> status_;
};

}  // namespace ethercat_motion
