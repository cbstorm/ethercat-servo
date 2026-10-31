// SPDX-License-Identifier: MIT
#include "ethercat_motion/mqtt_codec.hpp"

#include <cmath>
#include <cstdio>

#include <nlohmann/json.hpp>

namespace ethercat_motion {

using json = nlohmann::json;

TopicScheme::TopicScheme(std::string prefix, std::string ethercat_name)
    : prefix_(std::move(prefix)), ethercat_name_(std::move(ethercat_name)) {
    while (!prefix_.empty() && prefix_.back() == '/') prefix_.pop_back();
}

std::string TopicScheme::axis(const std::string& axis_name, const char* leaf) const {
    return prefix_ + "/" + axis_name + "/" + leaf;
}

std::string TopicScheme::machine(const char* leaf) const { return prefix_ + "/" + ethercat_name_ + "/" + leaf; }

std::string TopicScheme::commandWildcard() const { return prefix_ + "/+/command"; }

std::optional<std::string> TopicScheme::nameFromCommandTopic(const std::string& topic) const {
    const std::string head = prefix_ + "/";
    const std::string tail = "/command";
    if (topic.size() <= head.size() + tail.size()) return std::nullopt;
    if (topic.compare(0, head.size(), head) != 0) return std::nullopt;
    if (topic.compare(topic.size() - tail.size(), tail.size(), tail) != 0) return std::nullopt;
    std::string name = topic.substr(head.size(), topic.size() - head.size() - tail.size());
    if (name.empty() || name.find('/') != std::string::npos) return std::nullopt;
    return name;
}

namespace {

Status invalid(const std::string& message) { return {ErrorCode::InvalidArgument, message}; }

Result<json> parseObject(const std::string& payload) {
    json j = json::parse(payload, nullptr, false);
    if (j.is_discarded()) return Status(ErrorCode::InvalidArgument, "payload is not valid JSON");
    if (!j.is_object()) return Status(ErrorCode::InvalidArgument, "payload must be a JSON object");
    return j;
}

std::string requestIdOf(const json& j) {
    if (!j.contains("request_id")) return {};
    const json& r = j["request_id"];
    if (r.is_string()) return r.get<std::string>();
    if (r.is_number()) return r.dump();
    return {};
}

/// Reads a finite number. `required`: missing -> error.
Result<double> number(const json& j, const char* key, bool required, double fallback = 0.0) {
    if (!j.contains(key) || j[key].is_null()) {
        if (required) return Status(ErrorCode::InvalidArgument, std::string("missing field '") + key + "'");
        return fallback;
    }
    const json& v = j[key];
    if (!v.is_number()) return Status(ErrorCode::InvalidArgument, std::string("field '") + key + "' must be a number");
    const double d = v.get<double>();
    if (!std::isfinite(d)) return Status(ErrorCode::InvalidArgument, std::string("field '") + key + "' must be finite");
    return d;
}

Status checkRange(const char* key, double value, double min, double max) {
    if (value < min || value > max) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "field '%s' = %g outside [%g, %g]", key, value, min, max);
        return {ErrorCode::LimitViolation, buf};
    }
    return Status::ok();
}

json faultList(std::uint32_t faults) {
    char buf[256];
    describeSafetyFaults(faults, buf, sizeof(buf));
    json list = json::array();
    std::string current;
    for (const char* p = buf; *p != '\0'; ++p) {
        if (*p == ',') {
            list.push_back(current);
            current.clear();
        } else {
            current += *p;
        }
    }
    if (!current.empty()) list.push_back(current);
    return list;
}

std::string hex16(std::uint16_t v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%04X", v);
    return buf;
}

}  // namespace

Result<ParsedAxisCommand> parseAxisCommand(const std::string& payload, const AxisConfig& axis,
                                           std::uint16_t axis_index) {
    auto parsed = parseObject(payload);
    if (!parsed) return parsed.status();
    const json& j = parsed.value();
    if (!j.contains("command") || !j["command"].is_string()) return invalid("missing string field 'command'");

    ParsedAxisCommand out;
    out.request_id = requestIdOf(j);
    out.name = j["command"].get<std::string>();
    AxisCommand& c = out.command;
    c.axis = axis_index;
    const AxisLimits& lim = axis.limits;

    // Optional profile overrides shared by the move commands.
    auto profile = [&]() -> Status {
        auto v = number(j, "velocity", false);
        auto a = number(j, "acceleration", false);
        auto d = number(j, "deceleration", false);
        if (!v) return v.status();
        if (!a) return a.status();
        if (!d) return d.status();
        if (Status s = checkRange("velocity", v.value(), 0.0, lim.velocity_max); !s) return s;
        if (Status s = checkRange("acceleration", a.value(), 0.0, lim.acceleration_max); !s) return s;
        if (Status s = checkRange("deceleration", d.value(), 0.0, lim.deceleration_max); !s) return s;
        c.velocity = v.value();
        c.acceleration = a.value();
        c.deceleration = d.value();
        return Status::ok();
    };

    const std::string& name = out.name;
    if (name == "enable") {
        c.type = AxisCommandType::Enable;
    } else if (name == "disable") {
        c.type = AxisCommandType::Disable;
    } else if (name == "reset_fault") {
        c.type = AxisCommandType::ResetFault;
    } else if (name == "quick_stop") {
        c.type = AxisCommandType::QuickStop;
    } else if (name == "home") {
        c.type = AxisCommandType::Home;
    } else if (name == "get_status") {
        out.get_status = true;
    } else if (name == "stop") {
        c.type = AxisCommandType::Stop;
        auto d = number(j, "deceleration", false);
        if (!d) return d.status();
        if (Status s = checkRange("deceleration", d.value(), 0.0, lim.quick_stop_deceleration); !s) return s;
        c.deceleration = d.value();
    } else if (name == "move_absolute") {
        c.type = AxisCommandType::MoveAbsolute;
        auto p = number(j, "position", true);
        if (!p) return p.status();
        if (lim.soft_limits_enabled) {
            if (Status s = checkRange("position", p.value(), lim.position_min, lim.position_max); !s) return s;
        }
        c.value = p.value();
        if (Status s = profile(); !s) return s;
    } else if (name == "move_relative") {
        c.type = AxisCommandType::MoveRelative;
        auto p = number(j, j.contains("distance") ? "distance" : "position", true);
        if (!p) return p.status();
        c.value = p.value();
        if (Status s = profile(); !s) return s;
    } else if (name == "set_velocity") {
        c.type = AxisCommandType::MoveVelocity;
        auto v = number(j, "velocity", true);
        auto a = number(j, "acceleration", false);
        if (!v) return v.status();
        if (!a) return a.status();
        if (Status s = checkRange("velocity", v.value(), -lim.velocity_max, lim.velocity_max); !s) return s;
        if (Status s = checkRange("acceleration", a.value(), 0.0, lim.acceleration_max); !s) return s;
        c.value = v.value();
        c.acceleration = a.value();
    } else if (name == "set_torque") {
        c.type = AxisCommandType::SetTorque;
        auto t = number(j, "torque", true);
        if (!t) return t.status();
        if (Status s = checkRange("torque", t.value(), -lim.torque_max, lim.torque_max); !s) return s;
        c.value = t.value();
    } else if (name == "set_mode") {
        c.type = AxisCommandType::SetMode;
        if (!j.contains("mode") || !j["mode"].is_string()) return invalid("missing string field 'mode'");
        const auto mode = parseOperationMode(j["mode"].get<std::string>());
        if (!mode) return invalid("unknown mode '" + j["mode"].get<std::string>() + "'");
        c.mode = *mode;
    } else if (name == "set_pid") {
        c.type = AxisCommandType::SetPid;
        auto kp = number(j, "kp", true);
        auto ki = number(j, "ki", true);
        auto kd = number(j, "kd", true);
        if (!kp) return kp.status();
        if (!ki) return ki.status();
        if (!kd) return kd.status();
        if (kp.value() < 0.0 || ki.value() < 0.0 || kd.value() < 0.0) return invalid("PID gains must be >= 0");
        c.value = kp.value();
        c.velocity = ki.value();
        c.acceleration = kd.value();
    } else if (name == "set_position") {
        c.type = AxisCommandType::SetPosition;
        auto p = number(j, "position", true);
        if (!p) return p.status();
        c.value = p.value();
    } else {
        return invalid("unknown command '" + name + "'");
    }
    return out;
}

Result<ParsedMachineCommand> parseMachineCommand(const std::string& payload) {
    auto parsed = parseObject(payload);
    if (!parsed) return parsed.status();
    const json& j = parsed.value();
    if (!j.contains("command") || !j["command"].is_string()) return invalid("missing string field 'command'");
    ParsedMachineCommand out;
    out.request_id = requestIdOf(j);
    out.name = j["command"].get<std::string>();
    if (out.name == "get_status") out.command = MachineCommand::GetStatus;
    else if (out.name == "emergency_stop") out.command = MachineCommand::EmergencyStop;
    else if (out.name == "reset_faults") out.command = MachineCommand::ResetFaults;
    else if (out.name == "enable_all") out.command = MachineCommand::EnableAll;
    else if (out.name == "disable_all") out.command = MachineCommand::DisableAll;
    else if (out.name == "stop_all") out.command = MachineCommand::StopAll;
    else return invalid("unknown command '" + out.name + "'");
    return out;
}

std::string extractRequestId(const std::string& payload) {
    const json j = json::parse(payload, nullptr, false);
    return j.is_object() ? requestIdOf(j) : std::string();
}

std::string serializeTelemetry(const AxisStatus& s, std::int64_t unix_time_ms) {
    json j;
    j["axis"] = s.name.c_str();
    j["position"] = s.position;
    j["velocity"] = s.velocity;
    j["torque"] = s.torque;
    j["setpoint_position"] = s.setpoint_position;
    j["following_error"] = s.following_error;
    j["enabled"] = s.enabled;
    j["fault"] = s.fault;
    j["moving"] = s.moving;
    j["timestamp"] = unix_time_ms / 1000;
    j["timestamp_ms"] = unix_time_ms;
    return j.dump();
}

std::string serializeAxisStatus(const AxisStatus& s, std::int64_t unix_time_ms) {
    json j;
    j["axis"] = s.name.c_str();
    j["drive_state"] = toString(s.drive_state);
    j["mode"] = toString(s.mode);
    j["mode_display"] = toString(s.mode_display);
    j["motion_state"] = toString(s.motion_state);
    j["comm_ok"] = s.comm_ok;
    j["enabled"] = s.enabled;
    j["fault"] = s.fault;
    j["error_code"] = hex16(s.error_code);
    j["warning"] = s.warning;
    j["moving"] = s.moving;
    j["motion_done"] = s.motion_done;
    j["homed"] = s.homed;
    j["motion_permitted"] = s.motion_permitted;
    j["at_positive_limit"] = s.at_positive_limit;
    j["at_negative_limit"] = s.at_negative_limit;
    j["statusword"] = hex16(s.statusword);
    j["controlword"] = hex16(s.controlword);
    j["safety_faults"] = faultList(s.safety_faults);
    j["position"] = s.position;
    j["target_position"] = s.target_position;
    j["velocity"] = s.velocity;
    j["timestamp"] = unix_time_ms / 1000;
    j["timestamp_ms"] = unix_time_ms;
    return j.dump();
}

std::string serializeResponse(const std::string& request_id, const std::string& command, const char* state,
                              ErrorCode code, const std::string& message, std::uint64_t command_id,
                              std::int64_t unix_time_ms) {
    json j;
    j["request_id"] = request_id;
    j["command"] = command;
    j["status"] = state;
    j["ok"] = code == ErrorCode::Ok;
    if (code != ErrorCode::Ok) j["error"] = toString(code);
    if (!message.empty()) j["message"] = message;
    if (command_id != 0) j["command_id"] = command_id;
    j["timestamp"] = unix_time_ms / 1000;
    j["timestamp_ms"] = unix_time_ms;
    return j.dump();
}

std::string serializeMachineStatus(const MasterStatus& master, const SafetyStatus& safety, bool online,
                                   std::int64_t unix_time_ms) {
    json j;
    j["state"] = online ? "online" : "offline";
    j["master_state"] = toString(master.state);
    j["link_up"] = master.comm.link_up;
    j["slaves_responding"] = master.comm.slaves_responding;
    j["all_operational"] = master.comm.all_operational;
    j["working_counter"] = master.comm.working_counter;
    j["expected_working_counter"] = master.comm.expected_working_counter;
    j["safety"] = {{"latched", faultList(safety.latched_faults)},
                   {"active", faultList(safety.active_faults)},
                   {"emergency_stop", safety.emergency_stop_active},
                   {"reset_blocked", safety.reset_blocked}};
    j["timestamp"] = unix_time_ms / 1000;
    j["timestamp_ms"] = unix_time_ms;
    return j.dump();
}

std::string serializeDiagnostics(const MasterStatus& master, const SafetyStatus& safety, std::int64_t unix_time_ms) {
    const CycleStats& t = master.timing;
    json j;
    j["timing"] = {{"cycles", t.cycles},
                   {"overruns", t.overruns},
                   {"max_consecutive_overruns", t.max_consecutive_overruns},
                   {"period_min_us", static_cast<double>(t.period_min_ns) / 1000.0},
                   {"period_max_us", static_cast<double>(t.period_max_ns) / 1000.0},
                   {"jitter_max_us", static_cast<double>(t.jitter_max_ns) / 1000.0},
                   {"latency_max_us", static_cast<double>(t.latency_max_ns) / 1000.0},
                   {"exec_avg_us", t.exec_avg_ns / 1000.0},
                   {"exec_max_us", static_cast<double>(t.exec_max_ns) / 1000.0}};
    j["realtime_scheduling"] = master.realtime_scheduling;
    j["memory_locked"] = master.memory_locked;
    json slaves = json::array();
    for (std::size_t i = 0; i < master.slave_count && i < master.slaves.size(); ++i) {
        const SlaveStatus& s = master.slaves[i];
        slaves.push_back({{"position", s.ring_position},
                          {"al_state", alStateName(s.al_state)},
                          {"online", s.online},
                          {"operational", s.operational}});
    }
    j["slaves"] = slaves;
    j["safety"] = {{"latched", faultList(safety.latched_faults)},
                   {"first_fault", faultList(safety.first_fault)},
                   {"first_fault_axis", safety.first_fault_axis},
                   {"reset_count", safety.reset_count}};
    j["timestamp"] = unix_time_ms / 1000;
    j["timestamp_ms"] = unix_time_ms;
    return j.dump();
}

}  // namespace ethercat_motion
