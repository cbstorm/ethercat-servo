// SPDX-License-Identifier: MIT
//
// Top-level facade wiring configuration, logging, the EtherCAT master
// (IgH or simulation backend), the motion controller, the safety manager,
// a non-real-time service thread and the optional MQTT adapter.
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ethercat_motion/axis.hpp"
#include "ethercat_motion/config.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/motion_controller.hpp"
#include "ethercat_motion/safety_manager.hpp"
#include "ethercat_motion/simulation.hpp"

namespace ethercat_motion {

class MqttAdapter;

enum class ControllerState : std::uint8_t { Created, Initialized, Running, Stopped, Error };

class Controller {
public:
    /// Loads the YAML file in initialize(); construction never throws on
    /// configuration errors.
    explicit Controller(std::string config_file);
    explicit Controller(Config config);
    ~Controller();

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    /// Loads/validates configuration, creates the logger and backend,
    /// validates slave identities, configures PDOs and activates the master.
    Status initialize();
    /// Starts the cyclic task, waits for all slaves to reach OP, then starts
    /// the service thread and (if enabled) MQTT.
    Status start();
    /// Controlled shutdown: stop all axes, disable drives, stop MQTT, stop
    /// the cyclic task and release the master. Idempotent.
    Status stop(std::chrono::milliseconds stop_timeout = std::chrono::milliseconds(5000));

    /// Simulation/test mode: runs `cycles` cycles synchronously in the
    /// caller's thread (only after initialize() and without start()).
    Status step(std::uint32_t cycles = 1);

    /// Throws std::out_of_range for unknown axis names.
    Axis& axis(const std::string& name);
    MotionController& motion() noexcept { return motion_; }
    SafetyManager& safety() noexcept { return *safety_; }
    EthercatMaster* master() noexcept { return master_.get(); }
    /// Non-null only with the simulation backend.
    SimulationControl* simulation() noexcept { return simulation_; }
    MqttAdapter* mqtt() noexcept { return mqtt_.get(); }
    const Config& config() const noexcept { return config_; }
    ControllerState state() const noexcept { return state_; }

    MasterStatus ethercatStatus() const;
    SafetyStatus safetyStatus() const { return safety_->status(); }

    /// Software emergency stop (supervisory, not safety rated).
    void emergencyStop();
    void releaseEmergencyStop();
    /// Clears latched safety faults and resets drive faults on all axes.
    Status resetFaults();
    /// Application heartbeat for SafetyConfig::heartbeat_timeout_ms.
    void heartbeat();

    /// Invoked from the non-real-time service thread for every finished
    /// command. Must not block for long.
    void setCommandResultCallback(std::function<void(const CommandResult&)> callback);

private:
    class MqttBridge;

    Status verifySupportedModes();
    void serviceThread();
    void drainQueues();
    void logEvent(const Event& event);

    std::string config_file_;
    Config config_;
    bool config_loaded_ = false;
    ControllerState state_ = ControllerState::Created;

    std::unique_ptr<SafetyManager> safety_;
    MotionController motion_;
    std::unique_ptr<EthercatMaster> master_;
    SimulationControl* simulation_ = nullptr;
    EventRing events_;

    std::unique_ptr<MqttBridge> mqtt_bridge_;
    std::unique_ptr<MqttAdapter> mqtt_;

    std::thread service_;
    std::atomic<bool> service_running_{false};
    std::mutex callback_mutex_;
    std::function<void(const CommandResult&)> result_callback_;
    std::atomic<bool> rt_stalled_{false};
};

}  // namespace ethercat_motion
