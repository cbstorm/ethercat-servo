// SPDX-License-Identifier: MIT
//
// EtherCAT master manager: owns the backend, configures slaves, runs the
// deterministic cyclic task and monitors link, working counter and AL state.
//
// Cycle order (one iteration):
//   1. sleep until the absolute deadline (CLOCK_MONOTONIC)
//   2. receive frames + process domain           (backend.receive)
//   3. update communication status (WC, link, slave AL states)
//   4. CycleHandler::onCycle: read inputs, drive state machines, motion,
//      PID, write outputs                         (motion controller)
//   5. queue domain, DC sync, send frames         (backend.send)
//   6. record timing statistics (wait-free)
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ethercat_motion/diagnostics.hpp"
#include "ethercat_motion/ethercat_backend.hpp"
#include "ethercat_motion/ethercat_slave.hpp"
#include "ethercat_motion/lockfree.hpp"
#include "ethercat_motion/pdo_mapping.hpp"
#include "ethercat_motion/rt.hpp"

namespace ethercat_motion {

inline constexpr std::size_t kMaxSlaves = 32;

struct MasterConfig {
    std::string interface;               ///< Informational: IgH binds the NIC at module load.
    unsigned master_index = 0;
    std::uint32_t cycle_time_us = 1000;
    bool distributed_clocks = true;
    RtThreadConfig realtime;
    std::uint32_t operational_timeout_ms = 10000;  ///< Time allowed to reach OP after activation.
    std::uint32_t slave_state_poll_cycles = 10;    ///< Slave AL state polling interval.
};

/// Communication health computed every cycle.
struct CommStatus {
    bool link_up = false;
    bool wc_complete = false;
    std::uint32_t working_counter = 0;
    std::uint32_t expected_working_counter = 0;
    std::uint32_t consecutive_wc_errors = 0;
    std::uint32_t slaves_responding = 0;
    std::uint8_t al_states = 0;
    bool all_operational = false;
    bool reached_operational = false;  ///< All slaves reached OP at least once.
    std::uint32_t operational_mask_lo = 0;  ///< Bit i = slave i operational (first 32 slaves).
};

/// Data handed to the cyclic handler. Valid only for the duration of the call.
struct CycleContext {
    std::uint8_t* process_data = nullptr;
    std::int64_t now_ns = 0;
    double dt_s = 0.001;            ///< Nominal cycle time (deterministic).
    std::uint64_t cycle = 0;
    const CommStatus* comm = nullptr;
    const CycleStats* timing = nullptr;
    EthercatBackend* backend = nullptr;
};

/// Implemented by the motion controller. Called inside the cyclic task:
/// must not allocate, block, log, or perform I/O.
class CycleHandler {
public:
    virtual ~CycleHandler() = default;
    virtual void onCycle(const CycleContext& context) noexcept = 0;
};

enum class MasterState : std::uint8_t { Created, Initialized, Configured, Active, Running, Stopped, Error };
const char* toString(MasterState state) noexcept;

struct MasterStatus {
    MasterState state = MasterState::Created;
    CommStatus comm;
    std::array<SlaveStatus, kMaxSlaves> slaves{};
    std::uint16_t slave_count = 0;
    CycleStats timing;
    bool realtime_scheduling = false;
    bool memory_locked = false;
};

class EthercatMaster {
public:
    EthercatMaster(MasterConfig config, std::unique_ptr<EthercatBackend> backend);
    ~EthercatMaster();

    EthercatMaster(const EthercatMaster&) = delete;
    EthercatMaster& operator=(const EthercatMaster&) = delete;

    /// Opens (requests) the master and scans the bus.
    Status initialize();
    /// Registers a slave to be configured. Valid between initialize() and configure().
    Result<int> addSlave(const SlaveDeviceConfig& config);
    /// Validates identities against the scan and configures PDOs/SDOs/DC.
    /// Fails safely (no activation) when any slave does not match.
    Status configure();
    /// Blocking SDO upload from a configured slave (start-up checks).
    /// Valid after configure() and before activate().
    Result<std::uint64_t> uploadSdo(int slave, std::uint16_t index, std::uint8_t subindex, std::size_t size);
    /// Activates the master (process data exchange becomes possible).
    Status activate();
    /// Starts the cyclic real-time thread calling `handler` every cycle.
    Status start(CycleHandler* handler);
    /// Stops the cyclic thread. Idempotent.
    Status stop();
    /// Runs exactly one cycle in the caller's thread. Only valid while
    /// active and not running; intended for simulation and tests.
    Status step(CycleHandler* handler);
    /// Blocks (non-RT) until all slaves are operational or the timeout expires.
    Status waitForOperational(std::uint32_t timeout_ms) const;

    MasterStatus getStatus() const { return status_.load(); }
    MasterState state() const noexcept { return state_.load(); }
    const MasterConfig& config() const noexcept { return config_; }
    const std::vector<SlaveIdentity>& scannedSlaves() const noexcept { return scan_; }
    const PdoLayout& layout(int slave) const { return slaves_.at(static_cast<std::size_t>(slave)).layout; }
    const SlaveDeviceConfig& slaveConfig(int slave) const { return slaves_.at(static_cast<std::size_t>(slave)).config; }
    std::size_t slaveCount() const noexcept { return slaves_.size(); }
    /// Backend handle (for SDO requests) of a configured slave.
    int backendHandle(int slave) const { return slaves_.at(static_cast<std::size_t>(slave)).handle; }
    EthercatBackend& backend() noexcept { return *backend_; }
    /// Monotonically increasing cycle counter (watchdog heartbeat).
    std::uint64_t heartbeat() const noexcept { return heartbeat_.load(std::memory_order_relaxed); }
    /// Events produced by the cyclic task. Must be set before start().
    void setEventRing(EventRing* ring) noexcept { events_ = ring; }

private:
    struct SlaveEntry {
        SlaveDeviceConfig config;
        PdoLayout layout;
        int handle = -1;
        std::uint16_t ring_position = 0;
    };

    void cyclicThread(CycleHandler* handler);
    void runCycle(CycleHandler* handler, std::int64_t scheduled_ns, std::int64_t wake_ns);
    void updateComm() noexcept;
    void publishStatus() noexcept;
    void pushEvent(EventType type, std::int32_t code, std::int64_t value) noexcept;

    MasterConfig config_;
    std::unique_ptr<EthercatBackend> backend_;
    std::vector<SlaveIdentity> scan_;
    std::vector<SlaveEntry> slaves_;
    std::atomic<MasterState> state_{MasterState::Created};

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> heartbeat_{0};
    EventRing* events_ = nullptr;

    // Owned by the cyclic thread (or the step() caller).
    CommStatus comm_;
    CycleStats stats_;
    std::array<SlaveStatus, kMaxSlaves> slave_status_{};
    std::uint64_t cycle_ = 0;
    std::int64_t last_wake_ns_ = 0;
    std::uint32_t last_wc_ = 0;
    WorkingCounterState last_wc_state_ = WorkingCounterState::Zero;
    bool last_link_up_ = false;
    bool rt_fifo_ = false;
    bool rt_locked_ = false;

    SeqLock<MasterStatus> status_;
};

}  // namespace ethercat_motion
