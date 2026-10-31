// SPDX-License-Identifier: MIT
#include "ethercat_motion/ethercat_master.hpp"

#include <chrono>
#include <cstdio>
#include <set>
#include <thread>

#include <spdlog/spdlog.h>

namespace ethercat_motion {

const char* toString(MasterState state) noexcept {
    switch (state) {
        case MasterState::Created: return "created";
        case MasterState::Initialized: return "initialized";
        case MasterState::Configured: return "configured";
        case MasterState::Active: return "active";
        case MasterState::Running: return "running";
        case MasterState::Stopped: return "stopped";
        case MasterState::Error: return "error";
    }
    return "unknown";
}

EthercatMaster::EthercatMaster(MasterConfig config, std::unique_ptr<EthercatBackend> backend)
    : config_(std::move(config)), backend_(std::move(backend)) {}

EthercatMaster::~EthercatMaster() {
    (void)stop();
    if (backend_) {
        const MasterState s = state_.load();
        if (s == MasterState::Active || s == MasterState::Stopped) {
            backend_->deactivate();
        }
        backend_->close();
    }
}

Status EthercatMaster::initialize() {
    if (!backend_) {
        return {ErrorCode::InvalidState, "no EtherCAT backend"};
    }
    if (state_ != MasterState::Created) {
        return {ErrorCode::InvalidState, std::string("initialize() in state ") + toString(state_.load())};
    }
    if (config_.cycle_time_us < 100 || config_.cycle_time_us > 100000) {
        return {ErrorCode::InvalidConfig, "cycle_time_us must be within 100..100000"};
    }
    Status status = backend_->open(config_.master_index);
    if (!status) {
        state_ = MasterState::Error;
        return status;
    }
    auto scan = backend_->scanSlaves();
    if (!scan) {
        state_ = MasterState::Error;
        return scan.status();
    }
    scan_ = std::move(scan.value());
    logger()->info("EtherCAT master {} ({} backend): {} slave(s) found", config_.master_index, backend_->name(),
                   scan_.size());
    for (const auto& s : scan_) {
        logger()->info("  slave {}: alias {} vendor 0x{:08X} product 0x{:08X} rev 0x{:08X} '{}' [{}]",
                       s.ring_position, s.alias, s.vendor_id, s.product_code, s.revision, s.name,
                       alStateName(s.al_state));
    }
    state_ = MasterState::Initialized;
    return Status::ok();
}

Result<int> EthercatMaster::addSlave(const SlaveDeviceConfig& config) {
    if (state_ != MasterState::Initialized) {
        return Status(ErrorCode::InvalidState, "addSlave() is only valid after initialize() and before configure()");
    }
    if (slaves_.size() >= kMaxSlaves) {
        return Status(ErrorCode::LimitViolation, "too many slaves (max " + std::to_string(kMaxSlaves) + ")");
    }
    SlaveEntry entry;
    entry.config = config;
    slaves_.push_back(std::move(entry));
    return static_cast<int>(slaves_.size() - 1);
}

Status EthercatMaster::configure() {
    if (state_ != MasterState::Initialized) {
        return {ErrorCode::InvalidState, std::string("configure() in state ") + toString(state_.load())};
    }
    if (slaves_.empty()) {
        return {ErrorCode::InvalidConfig, "no slaves registered"};
    }

    // Validate everything first; configure only if every slave matches.
    std::set<std::uint16_t> used_positions;
    for (auto& slave : slaves_) {
        const std::string context = "slave '" + slave.config.name + "'";
        Status status = validateSyncConfig(slave.config.syncs, context);
        if (!status) {
            state_ = MasterState::Error;
            return status;
        }
        auto ring = resolveRingPosition(slave.config.alias, slave.config.position, scan_);
        if (!ring) {
            state_ = MasterState::Error;
            return {ring.status().code(), context + ": " + ring.status().message()};
        }
        slave.ring_position = ring.value();
        if (!used_positions.insert(slave.ring_position).second) {
            state_ = MasterState::Error;
            return {ErrorCode::InvalidConfig,
                    context + ": ring position " + std::to_string(slave.ring_position) + " used by two axes"};
        }
        status = validateIdentity(slave.config, scan_[slave.ring_position]);
        if (!status) {
            state_ = MasterState::Error;
            return status;
        }
    }

    const std::uint32_t cycle_ns = config_.cycle_time_us * 1000U;
    for (auto& slave : slaves_) {
        SlaveDeviceConfig effective = slave.config;
        effective.dc.enabled = effective.dc.enabled && config_.distributed_clocks;
        auto handle = backend_->configureSlave(effective, cycle_ns, slave.layout);
        if (!handle) {
            state_ = MasterState::Error;
            return {handle.status().code(), "slave '" + slave.config.name + "': " + handle.status().message()};
        }
        slave.handle = handle.value();
        logger()->info("configured slave '{}' at ring position {} ({} PDO entries, DC {})", slave.config.name,
                       slave.ring_position, slave.layout.entries().size(), effective.dc.enabled ? "on" : "off");
    }
    state_ = MasterState::Configured;
    return Status::ok();
}

Result<std::uint64_t> EthercatMaster::uploadSdo(int slave, std::uint16_t index, std::uint8_t subindex,
                                                std::size_t size) {
    if (state_ != MasterState::Configured) {
        return Status(ErrorCode::InvalidState, "uploadSdo() is only valid between configure() and activate()");
    }
    if (slave < 0 || static_cast<std::size_t>(slave) >= slaves_.size()) {
        return Status(ErrorCode::InvalidArgument, "invalid slave index");
    }
    return backend_->sdoUploadBlocking(slaves_[static_cast<std::size_t>(slave)].ring_position, index, subindex, size);
}

Status EthercatMaster::activate() {
    if (state_ != MasterState::Configured) {
        return {ErrorCode::InvalidState, std::string("activate() in state ") + toString(state_.load())};
    }
    Status status = backend_->activate();
    if (!status) {
        state_ = MasterState::Error;
        return status;
    }
    comm_ = CommStatus{};
    stats_.reset();
    cycle_ = 0;
    last_wake_ns_ = 0;
    state_ = MasterState::Active;
    publishStatus();
    logger()->info("EtherCAT master activated, process image {} bytes, expected WC {}", backend_->domainSize(),
                   backend_->expectedWorkingCounter());
    return Status::ok();
}

Status EthercatMaster::start(CycleHandler* handler) {
    const MasterState s = state_.load();
    if ((s != MasterState::Active && s != MasterState::Stopped) || running_.load()) {
        return {ErrorCode::InvalidState, std::string("start() in state ") + toString(s)};
    }
    running_ = true;
    state_ = MasterState::Running;
    try {
        thread_ = std::thread(&EthercatMaster::cyclicThread, this, handler);
    } catch (const std::system_error& ex) {
        running_ = false;
        state_ = MasterState::Error;
        return {ErrorCode::Internal, std::string("cannot start cyclic thread: ") + ex.what()};
    }
    return Status::ok();
}

Status EthercatMaster::stop() {
    if (!running_.exchange(false)) {
        return Status::ok();
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    state_ = MasterState::Stopped;
    publishStatus();
    return Status::ok();
}

Status EthercatMaster::step(CycleHandler* handler) {
    const MasterState s = state_.load();
    if ((s != MasterState::Active && s != MasterState::Stopped) || running_.load()) {
        return {ErrorCode::InvalidState, "step() requires an active master without running cyclic thread"};
    }
    const std::int64_t now = monotonicNs();
    runCycle(handler, now, now);
    publishStatus();
    return Status::ok();
}

Status EthercatMaster::waitForOperational(std::uint32_t timeout_ms) const {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (getStatus().comm.all_operational) {
            return Status::ok();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const MasterStatus status = getStatus();
    std::string detail;
    for (std::size_t i = 0; i < slaves_.size() && i < kMaxSlaves; ++i) {
        detail += " '" + slaves_[i].config.name + "'=" + alStateName(status.slaves[i].al_state) +
                  (status.slaves[i].online ? "" : "(offline)");
    }
    return {ErrorCode::Timeout, "slaves did not reach OP within " + std::to_string(timeout_ms) + " ms:" + detail +
                                    " (WC " + std::to_string(status.comm.working_counter) + "/" +
                                    std::to_string(status.comm.expected_working_counter) + ")"};
}

void EthercatMaster::cyclicThread(CycleHandler* handler) {
    const RtSetupReport report = applyRealtimeToCurrentThread(config_.realtime);
    rt_fifo_ = report.sched_fifo;
    rt_locked_ = report.memory_locked;
    // Logging is allowed here: we are not yet inside the cyclic loop.
    if (!report.warnings.empty()) {
        logger()->warn("cyclic task runs without full real-time setup: {}", report.warnings);
    }
    logger()->info("cyclic task started: {} us, SCHED_FIFO {}, mlockall {}, CPU {}", config_.cycle_time_us,
                   report.sched_fifo ? "yes" : "no", report.memory_locked ? "yes" : "no",
                   report.affinity ? std::to_string(config_.realtime.cpu_affinity) : std::string("any"));

    const std::int64_t period = static_cast<std::int64_t>(config_.cycle_time_us) * 1000;
    std::int64_t next = monotonicNs() + period;

    while (running_.load(std::memory_order_relaxed)) {
        sleepUntilNs(next);
        const std::int64_t wake = monotonicNs();
        runCycle(handler, next, wake);

        next += period;
        const std::int64_t done = monotonicNs();
        if (done > next) {
            // Deadline missed: skip the lost slots instead of bursting.
            ++stats_.overruns;
            ++stats_.consecutive_overruns;
            if (stats_.consecutive_overruns > stats_.max_consecutive_overruns) {
                stats_.max_consecutive_overruns = stats_.consecutive_overruns;
            }
            pushEvent(EventType::DeadlineOverrun, static_cast<std::int32_t>(stats_.consecutive_overruns),
                      done - next);
            while (next <= done) {
                next += period;
            }
        } else {
            stats_.consecutive_overruns = 0;
        }
        publishStatus();
    }
}

void EthercatMaster::runCycle(CycleHandler* handler, std::int64_t scheduled_ns, std::int64_t wake_ns) {
    const std::int64_t nominal = static_cast<std::int64_t>(config_.cycle_time_us) * 1000;
    if (last_wake_ns_ != 0) {
        const std::int64_t period = wake_ns - last_wake_ns_;
        stats_.period_last_ns = period;
        if (stats_.period_min_ns == 0 || period < stats_.period_min_ns) stats_.period_min_ns = period;
        if (period > stats_.period_max_ns) stats_.period_max_ns = period;
        const std::int64_t jitter = period > nominal ? period - nominal : nominal - period;
        if (jitter > stats_.jitter_max_ns) stats_.jitter_max_ns = jitter;
    }
    last_wake_ns_ = wake_ns;
    const std::int64_t latency = wake_ns - scheduled_ns;
    if (latency > stats_.latency_max_ns) stats_.latency_max_ns = latency;

    backend_->receive();
    updateComm();

    CycleContext context;
    context.process_data = backend_->domainData();
    context.now_ns = wake_ns;
    context.dt_s = static_cast<double>(nominal) * 1e-9;
    context.cycle = cycle_;
    context.comm = &comm_;
    context.timing = &stats_;
    context.backend = backend_.get();
    if (handler != nullptr) {
        handler->onCycle(context);
    }

    backend_->send(static_cast<std::uint64_t>(wake_ns), config_.distributed_clocks);

    const std::int64_t exec = monotonicNs() - wake_ns;
    stats_.exec_last_ns = exec;
    if (exec > stats_.exec_max_ns) stats_.exec_max_ns = exec;
    ++stats_.cycles;
    const double n = static_cast<double>(stats_.cycles < 1000 ? stats_.cycles : 1000);
    stats_.exec_avg_ns += (static_cast<double>(exec) - stats_.exec_avg_ns) / n;

    ++cycle_;
    heartbeat_.store(cycle_, std::memory_order_relaxed);
}

void EthercatMaster::updateComm() noexcept {
    const DomainState ds = backend_->domainState();
    comm_.working_counter = ds.working_counter;
    comm_.expected_working_counter = backend_->expectedWorkingCounter();
    comm_.wc_complete = ds.wc_state == WorkingCounterState::Complete;
    comm_.consecutive_wc_errors = comm_.wc_complete ? 0 : comm_.consecutive_wc_errors + 1;
    if (ds.working_counter != last_wc_ || ds.wc_state != last_wc_state_) {
        pushEvent(EventType::WorkingCounterChanged, static_cast<std::int32_t>(ds.wc_state), ds.working_counter);
        last_wc_ = ds.working_counter;
        last_wc_state_ = ds.wc_state;
    }

    const MasterLinkState ms = backend_->masterState();
    comm_.link_up = ms.link_up;
    comm_.slaves_responding = ms.slaves_responding;
    comm_.al_states = ms.al_states;
    if (ms.link_up != last_link_up_) {
        pushEvent(EventType::LinkStateChanged, ms.link_up ? 1 : 0, ms.slaves_responding);
        last_link_up_ = ms.link_up;
    }

    const std::uint32_t poll = config_.slave_state_poll_cycles == 0 ? 1 : config_.slave_state_poll_cycles;
    if (cycle_ % poll == 0 || !comm_.wc_complete) {
        bool all_op = !slaves_.empty();
        std::uint32_t mask = 0;
        for (std::size_t i = 0; i < slaves_.size(); ++i) {
            const SlaveRuntimeState s = backend_->slaveState(slaves_[i].handle);
            SlaveStatus& st = slave_status_[i];
            if (s.operational != st.operational || s.al_state != st.al_state || s.online != st.online) {
                pushEvent(EventType::SlaveStateChanged, static_cast<std::int32_t>(i),
                          (s.online ? 0x100 : 0) | s.al_state);
            }
            st.ring_position = slaves_[i].ring_position;
            st.online = s.online;
            st.operational = s.operational;
            st.al_state = s.al_state;
            all_op = all_op && s.operational;
            if (s.operational && i < 32) {
                mask |= 1U << i;
            }
        }
        comm_.all_operational = all_op && comm_.link_up;
        comm_.operational_mask_lo = comm_.link_up ? mask : 0;
        if (comm_.all_operational && comm_.wc_complete) {
            comm_.reached_operational = true;
        }
    }
}

void EthercatMaster::publishStatus() noexcept {
    MasterStatus status;
    status.state = state_.load(std::memory_order_relaxed);
    status.comm = comm_;
    status.slaves = slave_status_;
    status.slave_count = static_cast<std::uint16_t>(slaves_.size());
    status.timing = stats_;
    status.realtime_scheduling = rt_fifo_;
    status.memory_locked = rt_locked_;
    status_.store(status);
}

void EthercatMaster::pushEvent(EventType type, std::int32_t code, std::int64_t value) noexcept {
    if (events_ == nullptr) {
        return;
    }
    Event event;
    event.time_ns = last_wake_ns_;
    event.type = type;
    event.code = code;
    event.value = value;
    (void)events_->tryPush(event);
}

}  // namespace ethercat_motion
