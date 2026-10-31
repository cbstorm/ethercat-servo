// SPDX-License-Identifier: MIT
#include "ethercat_motion/mqtt_adapter.hpp"

#include <spdlog/spdlog.h>

#include "ethercat_motion/diagnostics.hpp"

#ifdef ETHERCAT_MOTION_WITH_MQTT

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

#include <mqtt/async_client.h>

#include "ethercat_motion/mqtt_codec.hpp"

namespace ethercat_motion {

namespace {

std::int64_t unixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool statusChanged(const AxisStatus& a, const AxisStatus& b) {
    return a.drive_state != b.drive_state || a.enabled != b.enabled || a.fault != b.fault ||
           a.motion_state != b.motion_state || a.mode_display != b.mode_display || a.homed != b.homed ||
           a.comm_ok != b.comm_ok || a.motion_permitted != b.motion_permitted || a.error_code != b.error_code ||
           a.safety_faults != b.safety_faults;
}

}  // namespace

bool mqttAvailable() noexcept { return true; }

struct MqttAdapter::Impl {
    struct Pending {
        std::string request_id;
        std::string command;
        std::string axis;
    };

    Impl(MqttConfig c, MqttCommandTarget& t) : cfg(std::move(c)), target(t), topics(cfg.topic_prefix, cfg.ethercat_topic) {}

    MqttConfig cfg;
    MqttCommandTarget& target;
    TopicScheme topics;
    std::unique_ptr<mqtt::async_client> client;
    mqtt::connect_options connect_options;
    mqtt::token_ptr connect_token;

    std::atomic<bool> running{false};
    std::atomic<bool> connected{false};
    std::atomic<bool> lost_pending{false};

    std::mutex in_mutex;
    std::condition_variable in_cv;
    std::deque<std::pair<std::string, std::string>> inbound;

    std::mutex pending_mutex;
    std::unordered_map<std::uint64_t, Pending> pending;

    std::mutex stop_mutex;
    std::condition_variable stop_cv;
    std::thread worker;
    std::thread publisher;

    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> rejected{0};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> published{0};
    std::atomic<std::uint64_t> reconnects{0};

    void publish(const std::string& topic, const std::string& payload, int qos, bool retained) {
        if (!client || !connected.load()) return;
        try {
            client->publish(topic, payload.data(), payload.size(), qos, retained);
            ++published;
        } catch (const mqtt::exception& ex) {
            logger()->debug("MQTT publish to '{}' failed: {}", topic, ex.what());
        }
    }

    void respond(const std::string& topic, const std::string& request_id, const std::string& command,
                 const char* state, ErrorCode code, const std::string& message, std::uint64_t id) {
        publish(topic, serializeResponse(request_id, command, state, code, message, id, unixMs()), cfg.qos, false);
    }

    void onMessage(const mqtt::const_message_ptr& msg) {
        {
            std::lock_guard<std::mutex> lock(in_mutex);
            if (inbound.size() >= cfg.command_queue_size) {
                ++dropped;
            } else {
                inbound.emplace_back(msg->get_topic(), msg->to_string());
                in_cv.notify_one();
                return;
            }
        }
        // Queue full: reject without touching the motion layer.
        const auto name = topics.nameFromCommandTopic(msg->get_topic());
        if (name) {
            respond(topics.axis(*name, "response"), extractRequestId(msg->to_string()), "", "rejected",
                    ErrorCode::QueueFull, "MQTT command queue full", 0);
        }
    }

    void handleMachine(const std::string& payload) {
        const std::string response = topics.machine("response");
        auto parsed = parseMachineCommand(payload);
        if (!parsed) {
            ++rejected;
            respond(response, extractRequestId(payload), "", "rejected", parsed.status().code(),
                    parsed.status().message(), 0);
            return;
        }
        const auto& cmd = parsed.value();
        Status status;
        switch (cmd.command) {
            case MachineCommand::GetStatus:
                publish(topics.machine("status"),
                        serializeMachineStatus(target.masterStatus(), target.safetyStatus(), true, unixMs()), cfg.qos,
                        true);
                for (const auto& name : target.axisNames()) {
                    if (auto s = target.axisStatus(name)) {
                        publish(topics.axis(name, "status"), serializeAxisStatus(*s, unixMs()), cfg.qos, false);
                    }
                }
                break;
            case MachineCommand::EmergencyStop:
                target.emergencyStop();
                break;
            case MachineCommand::ResetFaults:
                status = target.resetFaults();
                break;
            case MachineCommand::EnableAll:
                status = target.enableAll();
                break;
            case MachineCommand::DisableAll:
                status = target.disableAll();
                break;
            case MachineCommand::StopAll:
                status = target.stopAll();
                break;
        }
        if (!status) ++rejected;
        respond(response, cmd.request_id, cmd.name, status ? "accepted" : "rejected", status.code(), status.message(),
                0);
    }

    void handleAxis(const std::string& axis, const std::string& payload) {
        const std::string response = topics.axis(axis, "response");
        const AxisConfig* cfg_axis = target.axisConfig(axis);
        const auto index = target.axisIndex(axis);
        if (cfg_axis == nullptr || !index) {
            ++rejected;
            respond(response, extractRequestId(payload), "", "rejected", ErrorCode::NotFound,
                    "unknown axis '" + axis + "'", 0);
            return;
        }
        auto parsed = parseAxisCommand(payload, *cfg_axis, *index);
        if (!parsed) {
            ++rejected;
            respond(response, extractRequestId(payload), "", "rejected", parsed.status().code(),
                    parsed.status().message(), 0);
            return;
        }
        const ParsedAxisCommand& cmd = parsed.value();
        if (cmd.get_status) {
            if (auto s = target.axisStatus(axis)) {
                publish(topics.axis(axis, "status"), serializeAxisStatus(*s, unixMs()), cfg.qos, false);
            }
            respond(response, cmd.request_id, cmd.name, "completed", ErrorCode::Ok, "", 0);
            return;
        }
        auto id = target.submitCommand(cmd.command);
        if (!id) {
            ++rejected;
            respond(response, cmd.request_id, cmd.name, "rejected", id.status().code(), id.status().message(), 0);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(pending_mutex);
            if (pending.size() > 10000) pending.clear();  // results lost; never grow unbounded
            pending[id.value()] = Pending{cmd.request_id, cmd.name, axis};
        }
        respond(response, cmd.request_id, cmd.name, "accepted", ErrorCode::Ok, "", id.value());
    }

    void workerLoop() {
        while (running.load()) {
            std::pair<std::string, std::string> msg;
            {
                std::unique_lock<std::mutex> lock(in_mutex);
                in_cv.wait_for(lock, std::chrono::milliseconds(100), [&] { return !inbound.empty() || !running.load(); });
                if (inbound.empty()) continue;
                msg = std::move(inbound.front());
                inbound.pop_front();
            }
            ++received;
            const auto name = topics.nameFromCommandTopic(msg.first);
            if (!name) continue;
            try {
                if (*name == topics.ethercatName()) {
                    handleMachine(msg.second);
                } else {
                    handleAxis(*name, msg.second);
                }
            } catch (const std::exception& ex) {
                logger()->error("MQTT command handling failed: {}", ex.what());
            }
        }
    }

    void tryConnect() {
        if (connect_token && !connect_token->is_complete()) return;
        try {
            connect_token = client->connect(connect_options);
        } catch (const mqtt::exception& ex) {
            logger()->debug("MQTT connect attempt failed: {}", ex.what());
        }
    }

    void publisherLoop() {
        using clock = std::chrono::steady_clock;
        auto next_telemetry = clock::now();
        auto next_status = clock::now();
        auto next_connect = clock::now();
        auto backoff = std::chrono::seconds(cfg.reconnect_min_s);
        std::vector<std::pair<std::string, AxisStatus>> last;
        for (const auto& name : target.axisNames()) last.emplace_back(name, AxisStatus{});

        while (running.load()) {
            const auto now = clock::now();
            if (lost_pending.exchange(false)) {
                logger()->warn("MQTT connection lost; applying on_disconnect reaction '{}'", toString(cfg.on_disconnect));
                target.applyDisconnectReaction(cfg.on_disconnect);
            }
            if (!client->is_connected()) {
                if (now >= next_connect) {
                    tryConnect();
                    next_connect = now + backoff;
                    backoff = std::min(backoff * 2, std::chrono::seconds(cfg.reconnect_max_s));
                }
            } else {
                backoff = std::chrono::seconds(cfg.reconnect_min_s);
                if (now >= next_telemetry) {
                    next_telemetry = now + std::chrono::milliseconds(cfg.telemetry_interval_ms);
                    const std::int64_t ts = unixMs();
                    for (auto& [name, previous] : last) {
                        const auto s = target.axisStatus(name);
                        if (!s) continue;
                        publish(topics.axis(name, "telemetry"), serializeTelemetry(*s, ts), 0, false);
                        if (statusChanged(*s, previous)) {
                            publish(topics.axis(name, "status"), serializeAxisStatus(*s, ts), cfg.qos, false);
                            previous = *s;
                        }
                    }
                }
                if (now >= next_status) {
                    next_status = now + std::chrono::milliseconds(cfg.status_interval_ms);
                    const std::int64_t ts = unixMs();
                    const MasterStatus master = target.masterStatus();
                    const SafetyStatus safety = target.safetyStatus();
                    publish(topics.machine("status"), serializeMachineStatus(master, safety, true, ts), cfg.qos, true);
                    publish(topics.machine("diagnostics"), serializeDiagnostics(master, safety, ts), 0, false);
                    for (auto& [name, previous] : last) {
                        if (const auto s = target.axisStatus(name)) {
                            publish(topics.axis(name, "status"), serializeAxisStatus(*s, ts), cfg.qos, false);
                            previous = *s;
                        }
                    }
                }
            }
            std::unique_lock<std::mutex> lock(stop_mutex);
            stop_cv.wait_for(lock, std::chrono::milliseconds(5), [&] { return !running.load(); });
        }
    }
};

MqttAdapter::MqttAdapter(MqttConfig config, MqttCommandTarget& target)
    : impl_(std::make_unique<Impl>(std::move(config), target)) {}

MqttAdapter::~MqttAdapter() { stop(); }

Status MqttAdapter::start() {
    Impl& d = *impl_;
    if (d.running.load()) return {ErrorCode::InvalidState, "MQTT adapter already running"};
    const MqttConfig& cfg = d.cfg;
    try {
        if (cfg.mqtt_version == 5) {
            d.client = std::make_unique<mqtt::async_client>(cfg.broker, cfg.client_id, mqtt::create_options(MQTTVERSION_5));
        } else {
            d.client = std::make_unique<mqtt::async_client>(cfg.broker, cfg.client_id);
        }

        auto builder = cfg.mqtt_version == 5 ? mqtt::connect_options_builder::v5() : mqtt::connect_options_builder::v3();
        if (cfg.mqtt_version == 5) {
            builder.clean_start(cfg.clean_session);
        } else {
            builder.mqtt_version(MQTTVERSION_3_1_1);
            builder.clean_session(cfg.clean_session);
        }
        const std::string offline = serializeMachineStatus(MasterStatus{}, SafetyStatus{}, false, 0);
        builder.keep_alive_interval(std::chrono::seconds(cfg.keepalive_s))
            .connect_timeout(std::chrono::seconds(5))
            .automatic_reconnect(false)  // reconnects are driven by the publisher thread
            .will(mqtt::will_options(d.topics.machine("status"), offline, cfg.qos, true));
        if (!cfg.username.empty()) builder.user_name(cfg.username);
        if (!cfg.password.empty()) builder.password(cfg.password);
        if (cfg.tls_enabled) {
            mqtt::ssl_options_builder ssl;
            if (!cfg.tls_ca_file.empty()) ssl.trust_store(cfg.tls_ca_file);
            if (!cfg.tls_cert_file.empty()) ssl.key_store(cfg.tls_cert_file);
            if (!cfg.tls_key_file.empty()) ssl.private_key(cfg.tls_key_file);
            ssl.enable_server_cert_auth(cfg.tls_verify_server).verify(cfg.tls_verify_server);
            builder.ssl(ssl.finalize());
        }
        d.connect_options = builder.finalize();

        d.client->set_connected_handler([&d](const std::string&) {
            d.connected = true;
            ++d.reconnects;
            logger()->info("MQTT connected to {}", d.cfg.broker);
            try {
                d.client->subscribe(d.topics.commandWildcard(), d.cfg.qos);
            } catch (const mqtt::exception& ex) {
                logger()->error("MQTT subscribe failed: {}", ex.what());
            }
            d.publish(d.topics.machine("status"),
                      serializeMachineStatus(d.target.masterStatus(), d.target.safetyStatus(), true, unixMs()), d.cfg.qos,
                      true);
        });
        d.client->set_connection_lost_handler([&d](const std::string& cause) {
            d.connected = false;
            logger()->warn("MQTT connection lost: {}", cause.empty() ? "unknown cause" : cause);
            d.lost_pending = true;  // reaction applied from the publisher thread
        });
        d.client->set_message_callback([&d](mqtt::const_message_ptr msg) { d.onMessage(msg); });
    } catch (const mqtt::exception& ex) {
        return {ErrorCode::InvalidConfig, std::string("MQTT client setup failed: ") + ex.what()};
    }

    d.running = true;
    d.worker = std::thread([&d] { d.workerLoop(); });
    d.publisher = std::thread([&d] { d.publisherLoop(); });
    logger()->info("MQTT adapter started (broker {}, prefix '{}')", cfg.broker, cfg.topic_prefix);
    return Status::ok();
}

void MqttAdapter::stop() {
    Impl& d = *impl_;
    if (!d.running.exchange(false)) return;
    d.in_cv.notify_all();
    d.stop_cv.notify_all();
    if (d.worker.joinable()) d.worker.join();
    if (d.publisher.joinable()) d.publisher.join();
    if (d.client && d.client->is_connected()) {
        d.publish(d.topics.machine("status"), serializeMachineStatus(MasterStatus{}, SafetyStatus{}, false, unixMs()),
                  d.cfg.qos, true);
        try {
            d.client->disconnect()->wait_for(std::chrono::seconds(2));
        } catch (const mqtt::exception& ex) {
            logger()->debug("MQTT disconnect: {}", ex.what());
        }
    }
    d.connected = false;
    d.client.reset();
}

bool MqttAdapter::connected() const noexcept { return impl_->connected.load(); }

MqttAdapter::Stats MqttAdapter::stats() const {
    Stats s;
    s.received = impl_->received.load();
    s.rejected = impl_->rejected.load();
    s.dropped = impl_->dropped.load();
    s.published = impl_->published.load();
    s.reconnects = impl_->reconnects.load();
    return s;
}

void MqttAdapter::onCommandResult(const CommandResult& result) {
    Impl& d = *impl_;
    Impl::Pending p;
    {
        std::lock_guard<std::mutex> lock(d.pending_mutex);
        auto it = d.pending.find(result.id);
        if (it == d.pending.end()) return;
        p = std::move(it->second);
        d.pending.erase(it);
    }
    d.respond(d.topics.axis(p.axis, "response"), p.request_id, p.command,
              result.code == ErrorCode::Ok ? "completed" : "failed", result.code, "", result.id);
}

}  // namespace ethercat_motion

#else  // !ETHERCAT_MOTION_WITH_MQTT

namespace ethercat_motion {

bool mqttAvailable() noexcept { return false; }

struct MqttAdapter::Impl {};

MqttAdapter::MqttAdapter(MqttConfig /*config*/, MqttCommandTarget& /*target*/) : impl_(std::make_unique<Impl>()) {}
MqttAdapter::~MqttAdapter() = default;

Status MqttAdapter::start() {
    return {ErrorCode::NotSupported, "library built without MQTT support (Paho MQTT C++ not found at configure time)"};
}
void MqttAdapter::stop() {}
bool MqttAdapter::connected() const noexcept { return false; }
MqttAdapter::Stats MqttAdapter::stats() const { return {}; }
void MqttAdapter::onCommandResult(const CommandResult& /*result*/) {}

}  // namespace ethercat_motion

#endif
