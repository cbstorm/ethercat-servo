// SPDX-License-Identifier: MIT
#include "ethercat_motion/config.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "ethercat_motion/cia402_drive.hpp"
#include "ethercat_motion/esi_parser.hpp"
#include "ethercat_motion/safety_manager.hpp"

namespace ethercat_motion {

const char* toString(ControlLoop loop) noexcept {
    switch (loop) {
        case ControlLoop::None: return "none";
        case ControlLoop::Position: return "position";
        case ControlLoop::Velocity: return "velocity";
    }
    return "none";
}

const char* toString(SafetyReaction reaction) noexcept {
    switch (reaction) {
        case SafetyReaction::None: return "none";
        case SafetyReaction::Stop: return "stop";
        case SafetyReaction::QuickStop: return "quick_stop";
        case SafetyReaction::DisableVoltage: return "disable_voltage";
    }
    return "none";
}

std::optional<SafetyReaction> parseSafetyReaction(const std::string& text) noexcept {
    if (text == "none") return SafetyReaction::None;
    if (text == "stop") return SafetyReaction::Stop;
    if (text == "quick_stop") return SafetyReaction::QuickStop;
    if (text == "disable_voltage" || text == "disable") return SafetyReaction::DisableVoltage;
    return std::nullopt;
}

namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::optional<std::int64_t> parseInteger(const std::string& raw) {
    std::string text = raw;
    bool negative = false;
    if (!text.empty() && (text[0] == '-' || text[0] == '+')) {
        negative = text[0] == '-';
        text = text.substr(1);
    }
    if (text.empty()) return std::nullopt;
    int base = 10;
    if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        text = text.substr(2);
    } else if (text.size() > 2 && text[0] == '#' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        text = text.substr(2);
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(text.c_str(), &end, base);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        v > static_cast<unsigned long long>(std::numeric_limits<std::int64_t>::max())) {
        return std::nullopt;
    }
    const auto value = static_cast<std::int64_t>(v);
    return negative ? -value : value;
}

std::optional<double> parseDouble(const std::string& text) {
    const std::string l = lower(text);
    if (l == ".inf" || l == "inf") return std::numeric_limits<double>::infinity();
    if (l == "-.inf" || l == "-inf") return -std::numeric_limits<double>::infinity();
    if (auto i = parseInteger(text); i && text.find_first_of(".eE") == std::string::npos) {
        return static_cast<double>(*i);
    }
    errno = 0;
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0') return std::nullopt;
    return v;
}

/// Reads one YAML mapping, remembering which keys were used so unknown
/// keys (typos) can be reported.
class Reader {
public:
    Reader(YAML::Node node, std::string path, std::vector<std::string>& errors)
        : node_(std::move(node)), path_(std::move(path)), errors_(&errors) {
        if (node_ && !node_.IsNull() && !node_.IsMap()) {
            error(path_, "expected a mapping");
            node_ = YAML::Node();
        }
    }

    bool valid() const { return node_ && node_.IsMap(); }
    bool has(const char* key) {
        used_.insert(key);
        return valid() && node_[key] && !node_[key].IsNull();
    }
    std::string keyPath(const char* key) const { return path_.empty() ? key : path_ + "." + key; }

    YAML::Node raw(const char* key) {
        used_.insert(key);
        return valid() ? node_[key] : YAML::Node();
    }

    Reader child(const char* key) { return Reader(raw(key), keyPath(key), *errors_); }

    std::optional<std::string> scalar(const char* key) {
        if (!has(key)) return std::nullopt;
        const YAML::Node n = node_[key];
        if (!n.IsScalar()) {
            error(keyPath(key), "expected a scalar value");
            return std::nullopt;
        }
        return n.Scalar();
    }

    void get(const char* key, std::string& out) {
        if (auto s = scalar(key)) out = *s;
    }
    void get(const char* key, bool& out) {
        if (auto s = scalar(key)) {
            const std::string l = lower(*s);
            if (l == "true" || l == "yes" || l == "on" || l == "1") out = true;
            else if (l == "false" || l == "no" || l == "off" || l == "0") out = false;
            else error(keyPath(key), "expected boolean, got '" + *s + "'");
        }
    }
    void get(const char* key, double& out) {
        if (auto s = scalar(key)) {
            if (auto v = parseDouble(*s)) out = *v;
            else error(keyPath(key), "expected number, got '" + *s + "'");
        }
    }
    template <typename Int>
    void getInt(const char* key, Int& out) {
        if (auto s = scalar(key)) {
            auto v = parseInteger(*s);
            if (!v || *v < static_cast<std::int64_t>(std::numeric_limits<Int>::min()) ||
                static_cast<std::uint64_t>(std::max<std::int64_t>(*v, 0)) >
                    static_cast<std::uint64_t>(std::numeric_limits<Int>::max())) {
                error(keyPath(key), "expected integer in range, got '" + *s + "'");
                return;
            }
            out = static_cast<Int>(*v);
        }
    }
    void get(const char* key, int& out) { getInt(key, out); }
    void get(const char* key, std::int8_t& out) { getInt(key, out); }
    void get(const char* key, std::uint8_t& out) { getInt(key, out); }
    void get(const char* key, std::uint16_t& out) { getInt(key, out); }
    void get(const char* key, std::uint32_t& out) { getInt(key, out); }
    void get(const char* key, std::uint64_t& out) { getInt(key, out); }
    void get(const char* key, std::int64_t& out) { getInt(key, out); }
    void get(const char* key, std::optional<std::uint32_t>& out) {
        if (has(key)) {
            std::uint32_t v = 0;
            getInt(key, v);
            out = v;
        }
    }
    void get(const char* key, std::optional<std::uint16_t>& out) {
        if (has(key)) {
            std::uint16_t v = 0;
            getInt(key, v);
            out = v;
        }
    }
    void get(const char* key, SafetyReaction& out) {
        if (auto s = scalar(key)) {
            if (auto r = parseSafetyReaction(lower(*s))) out = *r;
            else error(keyPath(key), "expected none|stop|quick_stop|disable_voltage, got '" + *s + "'");
        }
    }
    void get(const char* key, OperationMode& out) {
        if (auto s = scalar(key)) {
            if (auto m = parseOperationMode(*s)) out = *m;
            else error(keyPath(key), "unknown operation mode '" + *s + "' (PP, PV, HOMING, CSP, CSV, CST)");
        }
    }

    void finish() {
        if (!valid()) return;
        for (const auto& kv : node_) {
            const std::string key = kv.first.Scalar();
            if (used_.count(key) == 0) {
                error(keyPath(key.c_str()), "unknown key");
            }
        }
    }

    void error(const std::string& path, const std::string& message) { errors_->push_back(path + ": " + message); }
    std::vector<std::string>& errors() { return *errors_; }

private:
    YAML::Node node_;
    std::string path_;
    std::vector<std::string>* errors_;
    std::set<std::string> used_;
};

void readPid(Reader r, PidConfig& pid) {
    r.get("kp", pid.kp);
    r.get("ki", pid.ki);
    r.get("kd", pid.kd);
    r.get("sample_time", pid.sample_time);
    r.get("output_min", pid.output_min);
    r.get("output_max", pid.output_max);
    r.get("integral_min", pid.integral_min);
    r.get("integral_max", pid.integral_max);
    r.get("anti_windup_gain", pid.anti_windup_gain);
    r.get("derivative_filter_time", pid.derivative_filter_time);
    r.get("derivative_on_measurement", pid.derivative_on_measurement);
    r.get("setpoint_ramp_rate", pid.setpoint_ramp_rate);
    r.get("output_rate_limit", pid.output_rate_limit);
    r.get("deadband", pid.deadband);
    r.get("enabled", pid.enabled);
    r.finish();
}

std::optional<PdoEntryConfig> readEntry(const YAML::Node& n, const std::string& path,
                                        std::vector<std::string>& errors) {
    PdoEntryConfig e;
    auto num = [&](const YAML::Node& v, const char* what) -> std::optional<std::int64_t> {
        if (!v || !v.IsScalar()) {
            errors.push_back(path + ": missing " + what);
            return std::nullopt;
        }
        auto parsed = parseInteger(v.Scalar());
        if (!parsed) errors.push_back(path + ": invalid " + what + " '" + v.Scalar() + "'");
        return parsed;
    };
    if (n.IsSequence()) {
        // [index, subindex, bits, "optional name"]
        if (n.size() < 3) {
            errors.push_back(path + ": expected [index, subindex, bit_length]");
            return std::nullopt;
        }
        const auto idx = num(n[0], "index");
        const auto sub = num(n[1], "subindex");
        const auto bits = num(n[2], "bit_length");
        if (!idx || !sub || !bits) return std::nullopt;
        e.index = static_cast<std::uint16_t>(*idx);
        e.subindex = static_cast<std::uint8_t>(*sub);
        e.bit_length = static_cast<std::uint8_t>(*bits);
        if (n.size() > 3 && n[3].IsScalar()) e.name = n[3].Scalar();
        return e;
    }
    if (n.IsMap()) {
        Reader r(n, path, errors);
        r.get("index", e.index);
        r.get("subindex", e.subindex);
        r.get("bit_length", e.bit_length);
        r.get("name", e.name);
        r.finish();
        return e;
    }
    errors.push_back(path + ": expected sequence or mapping");
    return std::nullopt;
}

void readSyncManagers(const YAML::Node& list, const std::string& path, std::vector<SyncManagerConfig>& out,
                      std::vector<std::string>& errors) {
    if (!list.IsSequence()) {
        errors.push_back(path + ": expected a list");
        return;
    }
    for (std::size_t i = 0; i < list.size(); ++i) {
        const std::string sm_path = path + "[" + std::to_string(i) + "]";
        Reader r(list[i], sm_path, errors);
        SyncManagerConfig sm;
        r.get("index", sm.index);
        std::string dir = "output";
        r.get("direction", dir);
        dir = lower(dir);
        if (dir == "output" || dir == "rx" || dir == "outputs") sm.direction = PdoDirection::Output;
        else if (dir == "input" || dir == "tx" || dir == "inputs") sm.direction = PdoDirection::Input;
        else r.error(sm_path + ".direction", "expected output|input");
        r.get("watchdog", sm.watchdog);
        const YAML::Node pdos = r.raw("pdos");
        if (!pdos || !pdos.IsSequence()) {
            r.error(sm_path + ".pdos", "expected a list of PDOs");
        } else {
            for (std::size_t p = 0; p < pdos.size(); ++p) {
                const std::string pdo_path = sm_path + ".pdos[" + std::to_string(p) + "]";
                Reader pr(pdos[p], pdo_path, errors);
                PdoConfig pdo;
                pr.get("index", pdo.index);
                const YAML::Node entries = pr.raw("entries");
                if (entries && entries.IsSequence()) {
                    for (std::size_t k = 0; k < entries.size(); ++k) {
                        if (auto e = readEntry(entries[k], pdo_path + ".entries[" + std::to_string(k) + "]", errors)) {
                            pdo.entries.push_back(*e);
                        }
                    }
                } else if (entries) {
                    pr.error(pdo_path + ".entries", "expected a list");
                }
                pr.finish();
                sm.pdos.push_back(pdo);
            }
        }
        r.finish();
        out.push_back(sm);
    }
}

void readSdoInit(const YAML::Node& list, const std::string& path, std::vector<SdoInitCommand>& out,
                 std::vector<std::string>& errors) {
    if (!list.IsSequence()) {
        errors.push_back(path + ": expected a list");
        return;
    }
    for (std::size_t i = 0; i < list.size(); ++i) {
        Reader r(list[i], path + "[" + std::to_string(i) + "]", errors);
        SdoInitCommand sdo;
        r.get("index", sdo.index);
        r.get("subindex", sdo.subindex);
        r.get("size", sdo.size);
        r.get("value", sdo.value);
        r.get("comment", sdo.comment);
        r.finish();
        out.push_back(sdo);
    }
}

void readAxis(Reader r, AxisConfig& a) {
    r.get("name", a.name);
    r.get("alias", a.alias);
    r.get("position", a.position);
    r.get("vendor_id", a.vendor_id);
    r.get("product_code", a.product_code);
    r.get("revision", a.revision);
    r.get("counts_per_revolution", a.counts_per_revolution);
    r.get("gear_ratio", a.gear_ratio);
    r.get("units_per_revolution", a.units_per_revolution);
    r.get("units", a.units);
    r.get("direction", a.direction);
    r.get("velocity_factor", a.velocity_factor);
    r.get("acceleration_factor", a.acceleration_factor);
    r.get("operation_mode", a.operation_mode);

    const YAML::Node modes = r.raw("allowed_modes");
    if (modes) {
        if (!modes.IsSequence()) {
            r.error(r.keyPath("allowed_modes"), "expected a list");
        } else {
            for (const auto& m : modes) {
                if (auto mode = parseOperationMode(m.Scalar())) a.allowed_modes.push_back(*mode);
                else r.error(r.keyPath("allowed_modes"), "unknown operation mode '" + m.Scalar() + "'");
            }
        }
    }

    std::string loop;
    r.get("control_loop", loop);
    loop = lower(loop);
    if (loop.empty() || loop == "none") a.control_loop = ControlLoop::None;
    else if (loop == "position") a.control_loop = ControlLoop::Position;
    else if (loop == "velocity") a.control_loop = ControlLoop::Velocity;
    else r.error(r.keyPath("control_loop"), "expected none|position|velocity");

    {
        Reader l = r.child("limits");
        auto& lim = a.limits;
        l.get("soft_limits_enabled", lim.soft_limits_enabled);
        l.get("position_min", lim.position_min);
        l.get("position_max", lim.position_max);
        l.get("velocity_max", lim.velocity_max);
        l.get("acceleration_max", lim.acceleration_max);
        l.get("deceleration_max", lim.deceleration_max);
        l.get("quick_stop_deceleration", lim.quick_stop_deceleration);
        l.get("following_error_max", lim.following_error_max);
        l.get("following_error_cycles", lim.following_error_cycles);
        l.get("velocity_tolerance", lim.velocity_tolerance);
        l.get("velocity_violation_cycles", lim.velocity_violation_cycles);
        l.get("position_tolerance", lim.position_tolerance);
        l.get("torque_max", lim.torque_max);
        l.finish();
    }
    {
        Reader m = r.child("motion");
        m.get("velocity", a.motion.velocity);
        m.get("acceleration", a.motion.acceleration);
        m.get("deceleration", a.motion.deceleration);
        m.get("in_position_window", a.motion.in_position_window);
        m.get("in_position_time_ms", a.motion.in_position_time_ms);
        m.get("motion_timeout_s", a.motion.motion_timeout_s);
        m.get("command_watchdog_ms", a.motion.command_watchdog_ms);
        m.finish();
    }
    {
        Reader h = r.child("homing");
        h.get("method", a.homing.method);
        h.get("speed_search", a.homing.speed_search);
        h.get("speed_zero", a.homing.speed_zero);
        h.get("acceleration", a.homing.acceleration);
        h.get("timeout_s", a.homing.timeout_s);
        h.finish();
    }
    if (r.has("pid")) readPid(r.child("pid"), a.pid);
    {
        Reader t = r.child("timeouts");
        t.get("state_transition_ms", a.timeouts.state_transition_ms);
        t.get("fault_reset_ms", a.timeouts.fault_reset_ms);
        t.get("mode_switch_ms", a.timeouts.mode_switch_ms);
        t.get("sdo_ms", a.timeouts.sdo_ms);
        t.finish();
    }
    {
        Reader d = r.child("dc");
        d.get("enabled", a.dc_enabled);
        d.get("assign_activate", a.dc_assign_activate);
        std::int32_t shift_us = 0;
        d.get("sync0_shift_us", shift_us);
        a.dc_sync0_shift_ns = shift_us * 1000;
        d.finish();
    }
    {
        Reader p = r.child("pdo");
        p.get("source", a.pdo_source);
        a.pdo_source = lower(a.pdo_source);
        p.get("esi_file", a.esi_file);
        const YAML::Node sms = p.raw("sync_managers");
        if (sms) readSyncManagers(sms, p.keyPath("sync_managers"), a.syncs, r.errors());
        p.finish();
    }
    if (r.has("sdo_init")) readSdoInit(r.raw("sdo_init"), r.keyPath("sdo_init"), a.sdo_init, r.errors());
    {
        Reader s = r.child("simulation");
        s.get("position_time_constant_s", a.simulation.position_time_constant_s);
        s.get("velocity_time_constant_s", a.simulation.velocity_time_constant_s);
        s.get("inertia", a.simulation.inertia);
        s.get("friction", a.simulation.friction);
        s.get("initial_position_counts", a.simulation.initial_position_counts);
        s.get("boot_cycles", a.simulation.boot_cycles);
        s.finish();
    }
    r.finish();
}

}  // namespace

namespace {

Status joinErrors(const std::vector<std::string>& errors) {
    std::string message = "configuration invalid:";
    for (const auto& e : errors) {
        message += "\n  - " + e;
    }
    return {ErrorCode::InvalidConfig, message};
}

void readDocument(const YAML::Node& root, Config& config, std::vector<std::string>& errors) {
    Reader top(root, "", errors);
    {
        Reader e = top.child("ethercat");
        std::string backend = "igh";
        e.get("backend", backend);
        backend = lower(backend);
        if (backend == "igh" || backend == "hardware") config.ethercat.backend = BackendType::Igh;
        else if (backend == "simulation" || backend == "sim") config.ethercat.backend = BackendType::Simulation;
        else e.error(e.keyPath("backend"), "expected igh|simulation");
        auto& m = config.ethercat.master;
        e.get("interface", m.interface);
        e.get("master_index", m.master_index);
        e.get("cycle_time_us", m.cycle_time_us);
        e.get("distributed_clocks", m.distributed_clocks);
        e.get("operational_timeout_ms", m.operational_timeout_ms);
        e.get("slave_state_poll_cycles", m.slave_state_poll_cycles);
        e.get("esi_file", config.ethercat.esi_file);
        Reader rt = e.child("realtime");
        rt.get("enabled", m.realtime.enabled);
        rt.get("priority", m.realtime.priority);
        rt.get("cpu_affinity", m.realtime.cpu_affinity);
        rt.get("lock_memory", m.realtime.lock_memory);
        rt.finish();
        e.finish();
    }

    const YAML::Node axes = top.raw("axes");
    if (!axes || !axes.IsSequence()) {
        errors.emplace_back("axes: expected a list of axes");
    } else {
        for (std::size_t i = 0; i < axes.size(); ++i) {
            AxisConfig axis;
            readAxis(Reader(axes[i], "axes[" + std::to_string(i) + "]", errors), axis);
            config.axes.push_back(std::move(axis));
        }
    }

    {
        Reader q = top.child("mqtt");
        auto& m = config.mqtt;
        q.get("enabled", m.enabled);
        q.get("broker", m.broker);
        q.get("client_id", m.client_id);
        q.get("username", m.username);
        q.get("password", m.password);
        if (auto v = q.scalar("version")) {
            if (*v == "3.1.1" || *v == "4" || *v == "3") m.mqtt_version = 4;
            else if (*v == "5" || *v == "5.0") m.mqtt_version = 5;
            else q.error(q.keyPath("version"), "expected 3.1.1 or 5");
        }
        q.get("keepalive_s", m.keepalive_s);
        q.get("clean_session", m.clean_session);
        q.get("qos", m.qos);
        q.get("telemetry_interval_ms", m.telemetry_interval_ms);
        q.get("status_interval_ms", m.status_interval_ms);
        q.get("reconnect_min_s", m.reconnect_min_s);
        q.get("reconnect_max_s", m.reconnect_max_s);
        std::uint32_t queue = static_cast<std::uint32_t>(m.command_queue_size);
        q.get("command_queue_size", queue);
        m.command_queue_size = queue;
        q.get("topic_prefix", m.topic_prefix);
        q.get("ethercat_topic", m.ethercat_topic);
        q.get("on_disconnect", m.on_disconnect);
        Reader tls = q.child("tls");
        tls.get("enabled", m.tls_enabled);
        tls.get("ca_file", m.tls_ca_file);
        tls.get("cert_file", m.tls_cert_file);
        tls.get("key_file", m.tls_key_file);
        tls.get("verify_server", m.tls_verify_server);
        tls.finish();
        q.finish();
    }
    {
        Reader s = top.child("safety");
        auto& c = config.safety;
        s.get("emergency_stop_reaction", c.emergency_stop_reaction);
        s.get("communication_loss_reaction", c.communication_loss_reaction);
        s.get("overrun_reaction", c.overrun_reaction);
        s.get("drive_fault_reaction", c.drive_fault_reaction);
        s.get("limit_reaction", c.limit_reaction);
        s.get("working_counter_error_cycles", c.working_counter_error_cycles);
        s.get("max_consecutive_overruns", c.max_consecutive_overruns);
        s.get("heartbeat_timeout_ms", c.heartbeat_timeout_ms);
        s.get("rt_watchdog_ms", c.rt_watchdog_ms);
        s.get("stop_all_on_axis_fault", c.stop_all_on_axis_fault);
        s.get("require_operational_at_start", c.require_operational_at_start);
        s.get("verify_supported_modes", c.verify_supported_modes);
        s.finish();
    }
    {
        Reader l = top.child("logging");
        l.get("level", config.logging.level);
        l.get("file", config.logging.file);
        l.get("console", config.logging.console);
        std::uint64_t size = config.logging.max_file_size;
        std::uint64_t files = config.logging.max_files;
        l.get("max_file_size", size);
        l.get("max_files", files);
        config.logging.max_file_size = static_cast<std::size_t>(size);
        config.logging.max_files = static_cast<std::size_t>(files);
        l.finish();
    }
    top.finish();
}

std::string resolvePath(const std::string& base, const std::string& path) {
    if (path.empty() || path[0] == '/' || base.empty()) return path;
    return base + "/" + path;
}

bool hasObject(const AxisConfig& a, std::uint16_t index, PdoDirection direction) {
    for (const auto& [entry, dir] : collectEntries(a.syncs)) {
        if (entry.index == index && dir == direction) return true;
    }
    return false;
}

std::vector<OperationMode> effectiveModes(const AxisConfig& a) {
    std::vector<OperationMode> modes = a.allowed_modes;
    if (std::find(modes.begin(), modes.end(), a.operation_mode) == modes.end()) modes.push_back(a.operation_mode);
    return modes;
}

bool validName(const std::string& name) {
    if (name.empty() || name.size() > 31) return false;
    return std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isalnum(c) || c == '_' || c == '-'; });
}

}  // namespace

Result<Config> loadConfigString(const std::string& yaml, const std::string& base_directory) {
    Config config;
    config.base_directory = base_directory;
    YAML::Node root;
    try {
        root = YAML::Load(yaml);
    } catch (const YAML::Exception& ex) {
        return Status(ErrorCode::InvalidConfig, std::string("YAML syntax error: ") + ex.what());
    }
    if (!root || !root.IsMap()) {
        return Status(ErrorCode::InvalidConfig, "configuration root must be a mapping");
    }
    std::vector<std::string> errors;
    try {
        readDocument(root, config, errors);
    } catch (const YAML::Exception& ex) {
        errors.emplace_back(std::string("YAML error: ") + ex.what());
    }
    if (!errors.empty()) return joinErrors(errors);
    Status status = finalizeConfig(config);
    if (!status) return status;
    status = validateConfig(config);
    if (!status) return status;
    return config;
}

Result<Config> loadConfigFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return Status(ErrorCode::IoError, "cannot open configuration file '" + path + "'");
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::size_t slash = path.find_last_of('/');
    const std::string base = slash == std::string::npos ? "." : path.substr(0, slash);
    auto config = loadConfigString(buffer.str(), base);
    if (!config) {
        return Status(config.status().code(), path + ": " + config.status().message());
    }
    return config;
}

Status finalizeConfig(Config& config) {
    if (config.finalized) return Status::ok();
    std::vector<std::string> errors;
    for (std::size_t i = 0; i < config.axes.size(); ++i) {
        AxisConfig& a = config.axes[i];
        const std::string path = "axes[" + std::to_string(i) + "]";

        auto& lim = a.limits;
        if (lim.deceleration_max <= 0.0) lim.deceleration_max = lim.acceleration_max;
        if (lim.quick_stop_deceleration <= 0.0) lim.quick_stop_deceleration = 2.0 * lim.deceleration_max;
        if (a.motion.velocity <= 0.0) a.motion.velocity = 0.5 * lim.velocity_max;
        if (a.motion.acceleration <= 0.0) a.motion.acceleration = lim.acceleration_max;
        if (a.motion.deceleration <= 0.0) a.motion.deceleration = lim.deceleration_max;
        const double counts_per_unit =
            a.units_per_revolution > 0.0 ? a.counts_per_revolution * a.gear_ratio / a.units_per_revolution : 0.0;
        if (a.motion.in_position_window <= 0.0 && counts_per_unit > 0.0) {
            a.motion.in_position_window = 10.0 / counts_per_unit;
        }
        // A jog stopped by the soft-limit guard ends exactly at the limit;
        // quantization and servo lag must not turn that into a fault.
        if (lim.position_tolerance <= 0.0) lim.position_tolerance = 2.0 * a.motion.in_position_window;
        if (a.homing.method != 0 &&
            std::find(a.allowed_modes.begin(), a.allowed_modes.end(), OperationMode::Homing) == a.allowed_modes.end()) {
            a.allowed_modes.push_back(OperationMode::Homing);
        }
        a.pid.sample_time = static_cast<double>(config.ethercat.master.cycle_time_us) * 1e-6;

        // ESI based device configuration.
        const std::string esi_path = resolvePath(config.base_directory, a.esi_file.empty() ? config.ethercat.esi_file
                                                                                           : a.esi_file);
        if (a.pdo_source != "config" && a.pdo_source != "esi") {
            errors.push_back(path + ".pdo.source: expected config|esi");
            continue;
        }
        if (a.pdo_source == "esi" || !esi_path.empty()) {
            if (esi_path.empty()) {
                errors.push_back(path + ".pdo.source: 'esi' requires ethercat.esi_file or pdo.esi_file");
                continue;
            }
            auto esi = parseEsiFile(esi_path);
            if (!esi) {
                errors.push_back(path + ": " + esi.status().message());
                continue;
            }
            if (esi->vendor_id != a.vendor_id) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), ".vendor_id: 0x%08X does not match ESI vendor 0x%08X", a.vendor_id,
                              esi->vendor_id);
                errors.push_back(path + buf);
                continue;
            }
            const EsiDevice* device = esi->find(a.product_code, a.revision);
            if (device == nullptr) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), ": product 0x%08X%s not found in ESI '%s'", a.product_code,
                              a.revision ? " (with configured revision)" : "", esi_path.c_str());
                errors.push_back(path + buf);
                continue;
            }
            if (const auto supported = device->supportedDriveModes()) {
                for (OperationMode mode : effectiveModes(a)) {
                    if ((*supported & cia402::supportedModeBit(mode)) == 0) {
                        char buf[192];
                        // The ESI is a declaration that can be wrong; the drive's own 0x6502 is
                        // checked at start-up (safety.verify_supported_modes) and is authoritative.
                        std::snprintf(buf, sizeof(buf),
                                      ": mode %s is not supported according to the ESI (0x6502 = 0x%X for '%s'); "
                                      "the drive's 0x6502 is verified at start-up",
                                      toString(mode), *supported, device->name.c_str());
                        config.warnings.push_back(path + buf);
                    }
                }
            }
            if (a.pdo_source == "esi") {
                if (!a.syncs.empty()) {
                    errors.push_back(path + ".pdo: sync_managers must not be given with source 'esi'");
                    continue;
                }
                a.syncs = device->defaultSyncConfig();
                std::vector<SdoInitCommand> sdos = device->startupSdos();
                sdos.insert(sdos.end(), a.sdo_init.begin(), a.sdo_init.end());
                a.sdo_init = std::move(sdos);
                if (!a.dc_assign_activate) a.dc_assign_activate = device->dcAssignActivate();
            } else if (!device->objects.empty()) {
                // Cross-check an explicit mapping against what the ESI declares mappable.
                for (const auto& [entry, dir] : collectEntries(a.syncs)) {
                    if (!device->isMappable(entry.index, dir)) {
                        char buf[160];
                        std::snprintf(buf, sizeof(buf),
                                      ".pdo: object 0x%04X is not declared %s-mappable in the ESI for '%s'",
                                      entry.index, dir == PdoDirection::Output ? "RxPDO" : "TxPDO",
                                      device->name.c_str());
                        errors.push_back(path + buf);
                    }
                }
            }
        }
    }
    if (!errors.empty()) return joinErrors(errors);
    config.finalized = true;
    return Status::ok();
}

Status validateConfig(const Config& config) {
    std::vector<std::string> e;
    const auto& m = config.ethercat.master;
    if (m.cycle_time_us < 100 || m.cycle_time_us > 100000) e.emplace_back("ethercat.cycle_time_us: must be 100..100000");
    if (m.realtime.priority < 1 || m.realtime.priority > 99) e.emplace_back("ethercat.realtime.priority: must be 1..99");
    if (config.axes.empty()) e.emplace_back("axes: at least one axis is required");
    if (config.axes.size() > kMaxAxes) e.emplace_back("axes: at most " + std::to_string(kMaxAxes) + " axes supported");

    std::set<std::string> names;
    std::set<std::uint32_t> addresses;
    for (std::size_t i = 0; i < config.axes.size(); ++i) {
        const AxisConfig& a = config.axes[i];
        const std::string p = "axes[" + std::to_string(i) + "]";
        auto err = [&](const std::string& key, const std::string& msg) { e.push_back(p + "." + key + ": " + msg); };

        if (!validName(a.name)) err("name", "required, 1-31 characters [A-Za-z0-9_-]");
        if (!names.insert(a.name).second) err("name", "duplicate axis name '" + a.name + "'");
        if (a.name == config.mqtt.ethercat_topic) err("name", "conflicts with mqtt.ethercat_topic");
        if (!addresses.insert((static_cast<std::uint32_t>(a.alias) << 16) | a.position).second) {
            err("position", "alias/position used by another axis");
        }
        if (a.vendor_id == 0) err("vendor_id", "required");
        if (a.product_code == 0) err("product_code", "required");
        if (!(a.counts_per_revolution > 0.0) || !std::isfinite(a.counts_per_revolution)) err("counts_per_revolution", "must be > 0");
        if (!(a.gear_ratio > 0.0) || !std::isfinite(a.gear_ratio)) err("gear_ratio", "must be > 0");
        if (!(a.units_per_revolution > 0.0) || !std::isfinite(a.units_per_revolution)) err("units_per_revolution", "must be > 0");
        if (a.direction != 1 && a.direction != -1) err("direction", "must be 1 or -1");
        if (!(a.velocity_factor > 0.0)) err("velocity_factor", "must be > 0");
        if (!(a.acceleration_factor > 0.0)) err("acceleration_factor", "must be > 0");

        const auto& l = a.limits;
        if (!(l.velocity_max > 0.0)) err("limits.velocity_max", "must be > 0");
        if (!(l.acceleration_max > 0.0)) err("limits.acceleration_max", "must be > 0");
        if (!(l.deceleration_max > 0.0)) err("limits.deceleration_max", "must be > 0");
        if (!(l.quick_stop_deceleration > 0.0)) err("limits.quick_stop_deceleration", "must be > 0");
        if (l.soft_limits_enabled && !(l.position_min < l.position_max)) {
            err("limits", "position_min must be < position_max (or set soft_limits_enabled: false)");
        }
        if (l.following_error_max < 0.0) err("limits.following_error_max", "must be >= 0");
        if (l.velocity_tolerance < 0.0) err("limits.velocity_tolerance", "must be >= 0");
        if (l.position_tolerance < 0.0) err("limits.position_tolerance", "must be >= 0");
        if (!(l.torque_max > 0.0) || l.torque_max > 1000.0) err("limits.torque_max", "must be in (0, 1000] %");

        const auto& mo = a.motion;
        if (mo.velocity > l.velocity_max) err("motion.velocity", "exceeds limits.velocity_max");
        if (mo.acceleration > l.acceleration_max) err("motion.acceleration", "exceeds limits.acceleration_max");
        if (mo.deceleration > l.deceleration_max) err("motion.deceleration", "exceeds limits.deceleration_max");
        if (mo.motion_timeout_s < 0.0) err("motion.motion_timeout_s", "must be >= 0");

        if (a.operation_mode == OperationMode::None || a.operation_mode == OperationMode::Homing) {
            err("operation_mode", "must be one of PP, PV, CSP, CSV, CST");
        }

        // PDO requirements.
        if (a.syncs.empty()) {
            err("pdo", "no PDO mapping: give pdo.sync_managers or use pdo.source: esi");
        } else {
            const Status s = validateSyncConfig(a.syncs, p + ".pdo");
            if (!s) e.push_back(s.message());
            if (!hasObject(a, 0x6040, PdoDirection::Output)) err("pdo", "0x6040 Controlword must be mapped (RxPDO)");
            if (!hasObject(a, 0x6041, PdoDirection::Input)) err("pdo", "0x6041 Statusword must be mapped (TxPDO)");
            if (!hasObject(a, 0x6064, PdoDirection::Input)) err("pdo", "0x6064 Position actual value must be mapped");
            for (OperationMode mode : effectiveModes(a)) {
                const std::string mode_name = toString(mode);
                if ((mode == OperationMode::CSP || mode == OperationMode::PP) &&
                    !hasObject(a, 0x607A, PdoDirection::Output)) {
                    err("pdo", mode_name + " requires 0x607A Target position in the RxPDO");
                }
                if ((mode == OperationMode::CSV || mode == OperationMode::PV) &&
                    !hasObject(a, 0x60FF, PdoDirection::Output)) {
                    err("pdo", mode_name + " requires 0x60FF Target velocity in the RxPDO");
                }
                if (mode == OperationMode::CST && !hasObject(a, 0x6071, PdoDirection::Output)) {
                    err("pdo", "CST requires 0x6071 Target torque in the RxPDO (not in the ISL-EC default mapping)");
                }
            }
        }
        for (std::size_t k = 0; k < a.sdo_init.size(); ++k) {
            const auto size = a.sdo_init[k].size;
            if (size != 1 && size != 2 && size != 4) err("sdo_init[" + std::to_string(k) + "].size", "must be 1, 2 or 4");
        }

        // Control architecture.
        const auto modes = effectiveModes(a);
        if (a.control_loop != ControlLoop::None) {
            if (!PIDController::isValid(a.pid)) {
                err("pid", "invalid (gains finite, output_min < output_max, non-negative rates/filters)");
            }
            // The external loop is only active in modes where the drive does
            // not already own that loop: position -> CSV/CST, velocity -> CST.
            const bool has_csv = std::find(modes.begin(), modes.end(), OperationMode::CSV) != modes.end();
            const bool has_cst = std::find(modes.begin(), modes.end(), OperationMode::CST) != modes.end();
            if (a.control_loop == ControlLoop::Position && !has_csv && !has_cst) {
                err("control_loop", "'position' loop requires CSV or CST; the drive owns the position loop in "
                                    "CSP/PP");
            }
            if (a.control_loop == ControlLoop::Velocity && !has_cst) {
                err("control_loop", "'velocity' loop requires CST; the drive owns the velocity loop in CSV/PV");
            }
        }
        if (std::find(modes.begin(), modes.end(), OperationMode::Homing) != modes.end()) {
            if (a.homing.method == 0) err("homing.method", "required when HOMING is allowed");
            if (!(a.homing.speed_search > 0.0)) err("homing.speed_search", "must be > 0");
            if (!(a.homing.speed_zero > 0.0)) err("homing.speed_zero", "must be > 0");
            if (!(a.homing.acceleration > 0.0)) err("homing.acceleration", "must be > 0");
            if (!(a.homing.timeout_s > 0.0)) err("homing.timeout_s", "must be > 0");
        }
        if (a.timeouts.state_transition_ms == 0 || a.timeouts.mode_switch_ms == 0 || a.timeouts.sdo_ms == 0) {
            err("timeouts", "all timeouts must be > 0");
        }
    }

    const auto& q = config.mqtt;
    if (q.enabled) {
        if (q.broker.empty()) e.emplace_back("mqtt.broker: required");
        if (q.client_id.empty()) e.emplace_back("mqtt.client_id: required");
        if (q.qos < 0 || q.qos > 2) e.emplace_back("mqtt.qos: must be 0, 1 or 2");
        if (q.keepalive_s <= 0) e.emplace_back("mqtt.keepalive_s: must be > 0");
        if (q.telemetry_interval_ms < 10) e.emplace_back("mqtt.telemetry_interval_ms: must be >= 10");
        if (q.status_interval_ms < 10) e.emplace_back("mqtt.status_interval_ms: must be >= 10");
        if (q.command_queue_size == 0 || q.command_queue_size > 10000) e.emplace_back("mqtt.command_queue_size: 1..10000");
        if (q.reconnect_min_s == 0 || q.reconnect_max_s < q.reconnect_min_s) {
            e.emplace_back("mqtt.reconnect_min_s/reconnect_max_s: 0 < min <= max");
        }
        if (q.topic_prefix.empty() || q.topic_prefix.find_first_of("+#") != std::string::npos) {
            e.emplace_back("mqtt.topic_prefix: required, no wildcards");
        }
        if (q.tls_enabled && q.broker.rfind("ssl://", 0) != 0 && q.broker.rfind("mqtts://", 0) != 0) {
            e.emplace_back("mqtt.tls.enabled: broker URI must use ssl:// or mqtts://");
        }
    }

    const std::set<std::string> levels = {"trace", "debug", "info", "warn", "warning", "error", "err", "critical", "off"};
    if (levels.count(lower(config.logging.level)) == 0) e.emplace_back("logging.level: unknown level");

    if (!e.empty()) return joinErrors(e);
    return Status::ok();
}

SlaveDeviceConfig toSlaveDeviceConfig(const AxisConfig& axis) {
    SlaveDeviceConfig s;
    s.name = axis.name;
    s.alias = axis.alias;
    s.position = axis.position;
    s.vendor_id = axis.vendor_id;
    s.product_code = axis.product_code;
    s.revision = axis.revision;
    s.syncs = axis.syncs;
    s.sdo_init = axis.sdo_init;
    s.dc.enabled = axis.dc_enabled;
    s.dc.assign_activate = axis.dc_assign_activate.value_or(0x0300);
    s.dc.sync0_shift_ns = axis.dc_sync0_shift_ns;
    return s;
}

}  // namespace ethercat_motion
