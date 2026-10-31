// SPDX-License-Identifier: MIT
//
// Diagnostics: cyclic timing statistics, real-time event records and
// logging setup. The cyclic task never logs directly; it pushes fixed-size
// Event records into a wait-free ring which a non-real-time service thread
// drains and forwards to spdlog.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ethercat_motion/lockfree.hpp"
#include "ethercat_motion/types.hpp"

namespace spdlog {
class logger;
}

namespace ethercat_motion {

/// Timing of the cyclic task. Updated by the real-time thread every cycle.
struct CycleStats {
    std::uint64_t cycles = 0;
    std::uint64_t overruns = 0;              ///< Cycles whose deadline was missed.
    std::uint32_t consecutive_overruns = 0;
    std::uint32_t max_consecutive_overruns = 0;
    std::int64_t period_last_ns = 0;
    std::int64_t period_min_ns = 0;
    std::int64_t period_max_ns = 0;
    std::int64_t exec_last_ns = 0;           ///< receive -> send execution time.
    std::int64_t exec_max_ns = 0;
    double exec_avg_ns = 0.0;
    std::int64_t latency_max_ns = 0;         ///< Wake-up latency vs. scheduled time.
    std::int64_t jitter_max_ns = 0;          ///< max |period - nominal|.

    void reset() noexcept { *this = CycleStats{}; }
};

enum class EventType : std::uint8_t {
    Info,
    DriveStateChanged,
    DriveFault,
    DriveFaultCleared,
    ModeChanged,
    CommandRejected,
    CommandCompleted,
    CommandFailed,
    MotionTimeout,
    FollowingError,
    PositionLimit,
    VelocityLimit,
    TransitionTimeout,
    SdoError,
    HomingCompleted,
    HomingFailed,
    WorkingCounterChanged,
    SlaveStateChanged,
    LinkStateChanged,
    DeadlineOverrun,
    SafetyFault,
    SafetyReset,
    EmergencyStop,
};

const char* toString(EventType type) noexcept;

/// Fixed-size record produced inside the cyclic task.
struct Event {
    std::int64_t time_ns = 0;
    EventType type = EventType::Info;
    std::int16_t axis = -1;   ///< Axis index or -1 for global events.
    std::int32_t code = 0;    ///< Event specific (state, error code, ...).
    std::int64_t value = 0;   ///< Event specific.
};

using EventRing = SpscRing<Event, 1024>;

struct LoggingConfig {
    std::string level = "info";   ///< trace, debug, info, warn, error, critical, off
    std::string file;             ///< Empty = no file sink.
    bool console = true;
    std::size_t max_file_size = 10 * 1024 * 1024;
    std::size_t max_files = 5;
};

/// Creates (or replaces) the library logger "ethercat_motion".
/// Must be called from non-real-time context.
Result<std::shared_ptr<spdlog::logger>> createLogger(const LoggingConfig& config);

/// Returns the library logger, creating a console logger on first use.
std::shared_ptr<spdlog::logger> logger();

}  // namespace ethercat_motion
