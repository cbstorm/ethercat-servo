// SPDX-License-Identifier: MIT
#include "ethercat_motion/diagnostics.hpp"

#include <mutex>
#include <vector>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace ethercat_motion {

namespace {

constexpr const char* kLoggerName = "ethercat_motion";

std::mutex& loggerMutex() {
    static std::mutex mutex;
    return mutex;
}

std::shared_ptr<spdlog::logger>& loggerSlot() {
    static std::shared_ptr<spdlog::logger> instance;
    return instance;
}

}  // namespace

const char* toString(EventType type) noexcept {
    switch (type) {
        case EventType::Info: return "info";
        case EventType::DriveStateChanged: return "drive_state_changed";
        case EventType::DriveFault: return "drive_fault";
        case EventType::DriveFaultCleared: return "drive_fault_cleared";
        case EventType::ModeChanged: return "mode_changed";
        case EventType::CommandRejected: return "command_rejected";
        case EventType::CommandCompleted: return "command_completed";
        case EventType::CommandFailed: return "command_failed";
        case EventType::MotionTimeout: return "motion_timeout";
        case EventType::FollowingError: return "following_error";
        case EventType::PositionLimit: return "position_limit";
        case EventType::VelocityLimit: return "velocity_limit";
        case EventType::TransitionTimeout: return "transition_timeout";
        case EventType::SdoError: return "sdo_error";
        case EventType::HomingCompleted: return "homing_completed";
        case EventType::HomingFailed: return "homing_failed";
        case EventType::WorkingCounterChanged: return "working_counter_changed";
        case EventType::SlaveStateChanged: return "slave_state_changed";
        case EventType::LinkStateChanged: return "link_state_changed";
        case EventType::DeadlineOverrun: return "deadline_overrun";
        case EventType::SafetyFault: return "safety_fault";
        case EventType::SafetyReset: return "safety_reset";
        case EventType::EmergencyStop: return "emergency_stop";
    }
    return "unknown";
}

Result<std::shared_ptr<spdlog::logger>> createLogger(const LoggingConfig& config) {
    std::vector<spdlog::sink_ptr> sinks;
    try {
        if (config.console) {
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        }
        if (!config.file.empty()) {
            sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(config.file, config.max_file_size,
                                                                                   config.max_files));
        }
    } catch (const spdlog::spdlog_ex& ex) {
        return Status(ErrorCode::IoError, std::string("cannot open log file '") + config.file + "': " + ex.what());
    }

    const auto level = spdlog::level::from_str(config.level);
    if (level == spdlog::level::off && config.level != "off") {
        return Status(ErrorCode::InvalidConfig, "unknown log level '" + config.level + "'");
    }

    auto created = std::make_shared<spdlog::logger>(kLoggerName, sinks.begin(), sinks.end());
    created->set_level(level);
    created->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %v");
    created->flush_on(spdlog::level::warn);

    std::lock_guard<std::mutex> lock(loggerMutex());
    loggerSlot() = created;
    return created;
}

std::shared_ptr<spdlog::logger> logger() {
    std::lock_guard<std::mutex> lock(loggerMutex());
    auto& slot = loggerSlot();
    if (!slot) {
        slot = std::make_shared<spdlog::logger>(kLoggerName, std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        slot->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %v");
    }
    return slot;
}

}  // namespace ethercat_motion
