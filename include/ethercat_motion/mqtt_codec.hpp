// SPDX-License-Identifier: MIT
//
// MQTT payload codec: JSON command parsing/validation and status/telemetry
// serialization, plus the topic scheme. Independent of any MQTT client
// library so it can be unit tested and reused.
//
// Topics (prefix "machine"):
//   machine/ethercat/status       retained, "online"/"offline" + bus status (LWT)
//   machine/ethercat/diagnostics  timing + safety
//   machine/ethercat/command      emergency_stop, reset_faults, get_status, ...
//   machine/ethercat/response
//   machine/<axis>/command        axis commands
//   machine/<axis>/response       acknowledgements / completion / errors
//   machine/<axis>/status         state changes + periodic
//   machine/<axis>/telemetry      periodic actual values
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "ethercat_motion/axis.hpp"
#include "ethercat_motion/config.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/safety_manager.hpp"

namespace ethercat_motion {

class TopicScheme {
public:
    TopicScheme(std::string prefix, std::string ethercat_name = "ethercat");

    std::string axis(const std::string& axis_name, const char* leaf) const;
    std::string machine(const char* leaf) const;
    /// "machine/+/command"
    std::string commandWildcard() const;
    /// Extracts <name> from "<prefix>/<name>/command"; nullopt otherwise.
    std::optional<std::string> nameFromCommandTopic(const std::string& topic) const;
    const std::string& ethercatName() const noexcept { return ethercat_name_; }

private:
    std::string prefix_;
    std::string ethercat_name_;
};

enum class MachineCommand : std::uint8_t {
    GetStatus,
    EmergencyStop,
    ResetFaults,
    EnableAll,
    DisableAll,
    StopAll,
};

struct ParsedAxisCommand {
    std::string request_id;
    std::string name;          ///< Command name as received.
    bool get_status = false;   ///< "get_status": answered without touching the cyclic task.
    AxisCommand command;
};

struct ParsedMachineCommand {
    std::string request_id;
    std::string name;
    MachineCommand command = MachineCommand::GetStatus;
};

/// Parses and validates an axis command against the axis limits.
/// Unknown fields are ignored; wrong types, missing fields, non-finite
/// numbers and out-of-range values are rejected with InvalidArgument /
/// LimitViolation.
Result<ParsedAxisCommand> parseAxisCommand(const std::string& payload, const AxisConfig& axis,
                                           std::uint16_t axis_index);
Result<ParsedMachineCommand> parseMachineCommand(const std::string& payload);

/// Best effort extraction of "request_id" for error responses.
std::string extractRequestId(const std::string& payload);

std::string serializeTelemetry(const AxisStatus& status, std::int64_t unix_time_ms);
std::string serializeAxisStatus(const AxisStatus& status, std::int64_t unix_time_ms);
std::string serializeResponse(const std::string& request_id, const std::string& command, const char* state,
                              ErrorCode code, const std::string& message, std::uint64_t command_id,
                              std::int64_t unix_time_ms);
std::string serializeMachineStatus(const MasterStatus& master, const SafetyStatus& safety, bool online,
                                   std::int64_t unix_time_ms);
std::string serializeDiagnostics(const MasterStatus& master, const SafetyStatus& safety, std::int64_t unix_time_ms);

}  // namespace ethercat_motion
