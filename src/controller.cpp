// SPDX-License-Identifier: MIT
#include "ethercat_motion/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>

#include <spdlog/spdlog.h>

#include "ethercat_motion/cia402_drive.hpp"
#include "ethercat_motion/mqtt_adapter.hpp"
#include "ethercat_motion/rt.hpp"

namespace ethercat_motion {

// Exposes the controller to the MQTT adapter without coupling MQTT to the
// EtherCAT / motion internals.
class Controller::MqttBridge final : public MqttCommandTarget {
public:
    explicit MqttBridge(Controller& c) : c_(c) {}

    std::vector<std::string> axisNames() const override { return c_.motion_.axisNames(); }
    const AxisConfig* axisConfig(const std::string& name) const override {
        Axis* a = c_.motion_.findAxis(name);
        return a != nullptr ? &a->config() : nullptr;
    }
    std::optional<std::uint16_t> axisIndex(const std::string& name) const override {
        Axis* a = c_.motion_.findAxis(name);
        if (a == nullptr) return std::nullopt;
        return static_cast<std::uint16_t>(a->index());
    }
    Result<std::uint64_t> submitCommand(const AxisCommand& command) override { return c_.motion_.submit(command); }
    std::optional<AxisStatus> axisStatus(const std::string& name) const override {
        Axis* a = c_.motion_.findAxis(name);
        if (a == nullptr) return std::nullopt;
        return c_.motion_.axisStatus(a->index());
    }
    MasterStatus masterStatus() const override { return c_.ethercatStatus(); }
    SafetyStatus safetyStatus() const override { return c_.safetyStatus(); }
    void emergencyStop() override { c_.emergencyStop(); }
    Status resetFaults() override { return c_.resetFaults(); }
    Status enableAll() override { return c_.motion_.enableAll(); }
    Status disableAll() override { return c_.motion_.disableAll(); }
    Status stopAll() override { return c_.motion_.stopAll(); }
    void applyDisconnectReaction(SafetyReaction reaction) override {
        switch (reaction) {
            case SafetyReaction::None: break;
            case SafetyReaction::Stop: (void)c_.motion_.stopAll(); break;
            case SafetyReaction::QuickStop: (void)c_.motion_.quickStopAll(); break;
            case SafetyReaction::DisableVoltage: (void)c_.motion_.disableAll(); break;
        }
    }

private:
    Controller& c_;
};

Controller::Controller(std::string config_file)
    : config_file_(std::move(config_file)), safety_(std::make_unique<SafetyManager>()) {}

Controller::Controller(Config config)
    : config_(std::move(config)), config_loaded_(true), safety_(std::make_unique<SafetyManager>()) {}

Controller::~Controller() { (void)stop(); }

namespace {

std::vector<SimulatedSlaveSpec> simulatedBus(const Config& config) {
    // Place axes addressed by absolute position at that ring position; fill
    // gaps with inert slaves; alias-addressed axes are appended.
    std::map<std::uint16_t, SimulatedSlaveSpec> by_position;
    std::vector<SimulatedSlaveSpec> aliased;
    for (const AxisConfig& a : config.axes) {
        SimulatedSlaveSpec s;
        s.name = "Sim-" + a.name;
        s.vendor_id = a.vendor_id;
        s.product_code = a.product_code;
        s.revision = a.revision.value_or(1);
        s.position_time_constant_s = a.simulation.position_time_constant_s;
        s.velocity_time_constant_s = a.simulation.velocity_time_constant_s;
        s.inertia = a.simulation.inertia;
        s.friction = a.simulation.friction;
        s.initial_position_counts = a.simulation.initial_position_counts;
        s.boot_cycles = a.simulation.boot_cycles;
        s.velocity_factor = a.velocity_factor;
        s.acceleration_factor = a.acceleration_factor;
        if (a.alias == 0) {
            by_position[a.position] = s;
        } else {
            s.alias = a.alias;
            aliased.push_back(s);
        }
    }
    std::vector<SimulatedSlaveSpec> bus;
    const std::size_t size = by_position.empty() ? 0 : static_cast<std::size_t>(by_position.rbegin()->first) + 1;
    for (std::size_t i = 0; i < size; ++i) {
        auto it = by_position.find(static_cast<std::uint16_t>(i));
        if (it != by_position.end()) {
            bus.push_back(it->second);
        } else {
            SimulatedSlaveSpec filler;
            filler.name = "Sim-Coupler";
            bus.push_back(filler);
        }
    }
    bus.insert(bus.end(), aliased.begin(), aliased.end());
    return bus;
}

}  // namespace

Status Controller::initialize() {
    if (state_ != ControllerState::Created) {
        return {ErrorCode::InvalidState, "initialize() may only be called once"};
    }
    auto fail = [this](Status s) {
        state_ = ControllerState::Error;
        logger()->error("initialization failed: {}", s.toString());
        return s;
    };

    if (!config_loaded_) {
        auto loaded = loadConfigFile(config_file_);
        if (!loaded) return fail(loaded.status());
        config_ = std::move(loaded.value());
        config_loaded_ = true;
    } else {
        Status s = finalizeConfig(config_);
        if (!s) return fail(s);
        s = validateConfig(config_);
        if (!s) return fail(s);
    }

    auto log = createLogger(config_.logging);
    if (!log) return fail(log.status());

    for (const std::string& w : config_.warnings) logger()->warn("configuration: {}", w);
    logger()->warn("SAFETY NOTICE: software supervision in this library is NOT safety rated. It does not replace "
                   "a certified safety PLC, STO circuit or hard-wired emergency stop.");
    if (!kernelIsPreemptRt()) {
        logger()->warn("kernel is not PREEMPT_RT: cyclic timing is best-effort, not hard real-time");
    }

    safety_ = std::make_unique<SafetyManager>(config_.safety);
    safety_->setEventRing(&events_);
    motion_.setSafetyManager(safety_.get());
    motion_.setEventRing(&events_);
    for (const AxisConfig& a : config_.axes) {
        Status s = motion_.addAxis(a.name, a);
        if (!s) return fail(s);
    }

    std::unique_ptr<EthercatBackend> backend;
    if (config_.ethercat.backend == BackendType::Simulation) {
        auto sim = std::make_unique<SimulatedBackend>(simulatedBus(config_));
        simulation_ = sim.get();
        backend = std::move(sim);
        logger()->warn("SIMULATION backend active: no EtherCAT hardware is driven");
    } else {
        auto igh = createIghBackend();
        if (!igh) return fail(igh.status());
        backend = std::move(igh.value());
    }

    master_ = std::make_unique<EthercatMaster>(config_.ethercat.master, std::move(backend));
    master_->setEventRing(&events_);
    Status s = master_->initialize();
    if (!s) return fail(s);
    s = motion_.attach(*master_);
    if (!s) return fail(s);
    s = master_->configure();
    if (!s) return fail(s);
    s = motion_.resolve(*master_);
    if (!s) return fail(s);
    if (config_.safety.verify_supported_modes) {
        s = verifySupportedModes();
        if (!s) return fail(s);
    }
    s = master_->activate();
    if (!s) return fail(s);

    state_ = ControllerState::Initialized;
    logger()->info("controller initialized: {} axis/axes, cycle {} us, backend {}", config_.axes.size(),
                   config_.ethercat.master.cycle_time_us, master_->backend().name());
    return Status::ok();
}

Status Controller::verifySupportedModes() {
    for (std::size_t i = 0; i < config_.axes.size(); ++i) {
        const AxisConfig& a = config_.axes[i];
        auto modes = master_->uploadSdo(static_cast<int>(i), cia402_obj::kSupportedDriveModes, 0, 4);
        if (!modes) {
            logger()->warn("[{}] could not read 0x6502 (supported drive modes): {} - verify the drive's modes manually",
                           a.name, modes.status().message());
            continue;
        }
        std::vector<OperationMode> wanted = a.allowed_modes;
        wanted.push_back(a.operation_mode);
        for (OperationMode mode : wanted) {
            if ((modes.value() & cia402::supportedModeBit(mode)) == 0) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "0x%llX", static_cast<unsigned long long>(modes.value()));
                return {ErrorCode::DeviceMismatch, "axis '" + a.name + "': drive reports 0x6502 = " + buf +
                                                       ", mode " + toString(mode) + " is not supported"};
            }
        }
        logger()->info("[{}] drive supports configured modes (0x6502 = 0x{:X})", a.name, modes.value());
    }
    return Status::ok();
}

Status Controller::start() {
    if (state_ != ControllerState::Initialized && state_ != ControllerState::Stopped) {
        return {ErrorCode::InvalidState, "start() requires an initialized, non-running controller"};
    }
    Status s = master_->start(&motion_);
    if (!s) return s;
    if (config_.safety.require_operational_at_start) {
        s = master_->waitForOperational(config_.ethercat.master.operational_timeout_ms);
        if (!s) {
            (void)master_->stop();
            logger()->error("start aborted: {}", s.toString());
            return s;
        }
    }
    rt_stalled_ = false;
    service_running_ = true;
    service_ = std::thread(&Controller::serviceThread, this);

    if (config_.mqtt.enabled) {
        mqtt_bridge_ = std::make_unique<MqttBridge>(*this);
        mqtt_ = std::make_unique<MqttAdapter>(config_.mqtt, *mqtt_bridge_);
        Status m = mqtt_->start();
        if (!m) {
            // MQTT is optional: motion control keeps running without it.
            logger()->error("MQTT disabled: {}", m.toString());
            mqtt_.reset();
        }
    }
    state_ = ControllerState::Running;
    logger()->info("controller running");
    return Status::ok();
}

Status Controller::stop(std::chrono::milliseconds stop_timeout) {
    if (state_ != ControllerState::Running) {
        return Status::ok();
    }
    logger()->info("controlled shutdown: stopping axes");
    (void)motion_.stopAll();
    if (Status s = motion_.waitAllIdle(stop_timeout); !s) logger()->warn("shutdown: {}", s.toString());
    (void)motion_.disableAll();
    const auto deadline = std::chrono::steady_clock::now() + stop_timeout;
    for (;;) {
        bool any_enabled = false;
        for (std::size_t i = 0; i < motion_.axisCount(); ++i) any_enabled = any_enabled || motion_.axisStatus(i).enabled;
        if (!any_enabled || std::chrono::steady_clock::now() >= deadline) {
            if (any_enabled) logger()->warn("shutdown: some drives did not report disabled in time");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (mqtt_) {
        mqtt_->stop();
        mqtt_.reset();
    }
    (void)master_->stop();
    service_running_ = false;
    if (service_.joinable()) service_.join();
    drainQueues();
    state_ = ControllerState::Stopped;
    logger()->info("controller stopped");
    return Status::ok();
}

Status Controller::step(std::uint32_t cycles) {
    if (state_ != ControllerState::Initialized && state_ != ControllerState::Stopped) {
        return {ErrorCode::InvalidState, "step() requires an initialized controller that is not running"};
    }
    for (std::uint32_t i = 0; i < cycles; ++i) {
        Status s = master_->step(&motion_);
        if (!s) return s;
        drainQueues();
    }
    return Status::ok();
}

Axis& Controller::axis(const std::string& name) { return motion_.getAxis(name); }

MasterStatus Controller::ethercatStatus() const { return master_ ? master_->getStatus() : MasterStatus{}; }

void Controller::emergencyStop() {
    logger()->warn("software emergency stop requested");
    safety_->requestEmergencyStop();
}

void Controller::releaseEmergencyStop() { safety_->releaseEmergencyStop(); }

Status Controller::resetFaults() {
    safety_->requestReset();
    return motion_.resetFaultAll();
}

void Controller::heartbeat() { safety_->heartbeat(monotonicNs()); }

void Controller::setCommandResultCallback(std::function<void(const CommandResult&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    result_callback_ = std::move(callback);
}

void Controller::serviceThread() {
    std::uint64_t last_heartbeat = master_->heartbeat();
    auto last_change = std::chrono::steady_clock::now();
    while (service_running_.load()) {
        drainQueues();
        const std::uint64_t hb = master_->heartbeat();
        const auto now = std::chrono::steady_clock::now();
        if (hb != last_heartbeat) {
            last_heartbeat = hb;
            last_change = now;
            if (rt_stalled_.exchange(false)) logger()->warn("cyclic task resumed");
        } else if (config_.safety.rt_watchdog_ms > 0 &&
                   now - last_change > std::chrono::milliseconds(config_.safety.rt_watchdog_ms) && !rt_stalled_) {
            rt_stalled_ = true;
            logger()->critical("cyclic task stalled for more than {} ms: drives rely on their own SM watchdog",
                               config_.safety.rt_watchdog_ms);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Controller::drainQueues() {
    Event event;
    while (events_.tryPop(event)) logEvent(event);

    CommandResult result;
    while (motion_.popResult(result)) {
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (result_callback_) result_callback_(result);
        }
        if (mqtt_) mqtt_->onCommandResult(result);
    }
}

void Controller::logEvent(const Event& e) {
    const std::string axis = e.axis >= 0 && static_cast<std::size_t>(e.axis) < motion_.axisCount()
                                 ? motion_.axis(static_cast<std::size_t>(e.axis)).name()
                                 : std::string("-");
    auto log = logger();
    switch (e.type) {
        case EventType::DriveStateChanged:
            log->info("[{}] drive state -> {} (statusword 0x{:04X})", axis, toString(static_cast<DriveState>(e.code)),
                      static_cast<unsigned>(e.value));
            break;
        case EventType::DriveFault:
            log->error("[{}] DRIVE FAULT, error code 0x{:04X}", axis, static_cast<unsigned>(e.code));
            break;
        case EventType::DriveFaultCleared:
            log->info("[{}] drive fault cleared", axis);
            break;
        case EventType::ModeChanged:
            log->info("[{}] mode of operation -> {}", axis, toString(static_cast<OperationMode>(e.code)));
            break;
        case EventType::CommandRejected:
            log->warn("[{}] command '{}' rejected: {}", axis, toString(static_cast<AxisCommandType>(e.code)),
                      toString(static_cast<ErrorCode>(e.value)));
            break;
        case EventType::CommandCompleted:
            log->debug("[{}] command '{}' completed", axis, toString(static_cast<AxisCommandType>(e.code)));
            break;
        case EventType::CommandFailed:
            log->warn("[{}] command '{}' failed: {}", axis, toString(static_cast<AxisCommandType>(e.code)),
                      toString(static_cast<ErrorCode>(e.value)));
            break;
        case EventType::MotionTimeout:
            log->error("[{}] motion timeout after {} ms (kind {})", axis, e.value, e.code);
            break;
        case EventType::FollowingError:
            log->error("[{}] following error {:.3f} exceeds limit", axis, static_cast<double>(e.value) / 1000.0);
            break;
        case EventType::PositionLimit:
            log->error("[{}] position limit ({})", axis,
                       e.code == 2 ? "stopped at +soft limit" : e.code == -2 ? "stopped at -soft limit"
                                                                             : "soft limit exceeded");
            break;
        case EventType::VelocityLimit:
            log->error("[{}] velocity limit exceeded ({:.3f})", axis, static_cast<double>(e.value) / 1000.0);
            break;
        case EventType::TransitionTimeout:
            log->error("[{}] CiA 402 transition timeout in state {}", axis, toString(static_cast<DriveState>(e.code)));
            break;
        case EventType::SdoError:
            log->error("[{}] SDO parameter write failed (parameter {})", axis, e.code);
            break;
        case EventType::HomingCompleted:
            log->info("[{}] homing completed at {} counts", axis, e.value);
            break;
        case EventType::HomingFailed:
            log->error("[{}] homing failed (reason {})", axis, e.code);
            break;
        case EventType::WorkingCounterChanged:
            log->info("working counter {} ({})", e.value,
                      e.code == 2 ? "complete" : e.code == 1 ? "incomplete" : "zero");
            break;
        case EventType::SlaveStateChanged:
            log->info("slave {} -> {}{}", e.code, alStateName(static_cast<std::uint8_t>(e.value & 0xFF)),
                      (e.value & 0x100) ? "" : " (offline)");
            break;
        case EventType::LinkStateChanged:
            if (e.code != 0) log->info("EtherCAT link up ({} slaves responding)", e.value);
            else log->error("EtherCAT link DOWN");
            break;
        case EventType::DeadlineOverrun:
            log->warn("cycle deadline overrun (consecutive {}, late by {} us)", e.code, e.value / 1000);
            break;
        case EventType::SafetyFault: {
            char names[256];
            describeSafetyFaults(static_cast<std::uint32_t>(e.code), names, sizeof(names));
            log->error("[{}] SAFETY FAULT latched: {}", axis, names);
            break;
        }
        case EventType::SafetyReset:
            log->info("safety faults reset");
            break;
        case EventType::EmergencyStop:
            log->error("EMERGENCY STOP active");
            break;
        case EventType::Info:
            log->info("[{}] event code {} value {}", axis, e.code, e.value);
            break;
    }
}

}  // namespace ethercat_motion
