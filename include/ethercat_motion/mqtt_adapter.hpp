// SPDX-License-Identifier: MIT
//
// MQTT adapter (Eclipse Paho MQTT C++).
//
// Threading model:
//   * Paho callback threads only push received messages into a bounded
//     queue (drop + error response when full). They never touch EtherCAT
//     process data and never block the cyclic task.
//   * A worker thread parses/validates commands and submits them through
//     MqttCommandTarget (which enqueues into the motion command queue).
//   * A publisher thread publishes telemetry/status at configured rates
//     from wait-free snapshots.
//
// MQTT is supervisory only: it is not a safety-rated stop mechanism.
// The library builds and runs without MQTT (ETHERCAT_MOTION_WITH_MQTT=OFF);
// start() then returns ErrorCode::NotSupported.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ethercat_motion/axis.hpp"
#include "ethercat_motion/config.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/safety_manager.hpp"

namespace ethercat_motion {

/// What the adapter needs from the application. Implemented by Controller;
/// tests can provide a fake.
class MqttCommandTarget {
public:
    virtual ~MqttCommandTarget() = default;
    virtual std::vector<std::string> axisNames() const = 0;
    virtual const AxisConfig* axisConfig(const std::string& name) const = 0;
    virtual std::optional<std::uint16_t> axisIndex(const std::string& name) const = 0;
    virtual Result<std::uint64_t> submitCommand(const AxisCommand& command) = 0;
    virtual std::optional<AxisStatus> axisStatus(const std::string& name) const = 0;
    virtual MasterStatus masterStatus() const = 0;
    virtual SafetyStatus safetyStatus() const = 0;
    virtual void emergencyStop() = 0;
    virtual Status resetFaults() = 0;
    virtual Status enableAll() = 0;
    virtual Status disableAll() = 0;
    virtual Status stopAll() = 0;
    /// Applied when the broker connection is lost (MqttConfig::on_disconnect).
    virtual void applyDisconnectReaction(SafetyReaction reaction) = 0;
};

bool mqttAvailable() noexcept;

class MqttAdapter {
public:
    struct Stats {
        std::uint64_t received = 0;
        std::uint64_t rejected = 0;
        std::uint64_t dropped = 0;
        std::uint64_t published = 0;
        std::uint64_t reconnects = 0;
    };

    MqttAdapter(MqttConfig config, MqttCommandTarget& target);
    ~MqttAdapter();

    MqttAdapter(const MqttAdapter&) = delete;
    MqttAdapter& operator=(const MqttAdapter&) = delete;

    /// Connects asynchronously and starts worker/publisher threads. Returns
    /// immediately; a broker that is down does not fail start(): the client
    /// keeps reconnecting in the background.
    Status start();
    void stop();
    bool connected() const noexcept;
    Stats stats() const;

    /// Called by the controller's service thread for finished commands.
    void onCommandResult(const CommandResult& result);

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace ethercat_motion
