// SPDX-License-Identifier: MIT
//
// Motion control manager / multi-axis coordinator.
//
// Owns the axes, the command queue (non-RT -> RT), the result queue
// (RT -> non-RT) and implements CycleHandler: inside the cyclic task it runs,
// for every axis, the CiA 402 state machine, trajectory generation, optional
// PID loops, limit supervision and the safety reactions.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ethercat_motion/axis.hpp"
#include "ethercat_motion/config.hpp"
#include "ethercat_motion/diagnostics.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/lockfree.hpp"
#include "ethercat_motion/safety_manager.hpp"

namespace ethercat_motion {

class AxisRuntime;

using CommandQueue = MpscRing<AxisCommand, 256>;
using ResultQueue = SpscRing<CommandResult, 512>;

class MotionController final : public CycleHandler {
public:
    MotionController();
    explicit MotionController(SafetyManager* safety);
    ~MotionController() override;

    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;

    /// Adds an axis. Only valid before attach().
    Status addAxis(const std::string& name, const AxisConfig& config);

    std::size_t axisCount() const noexcept { return axes_.size(); }
    /// Throws std::out_of_range for unknown names.
    Axis& getAxis(const std::string& name);
    const Axis& getAxis(const std::string& name) const;
    Axis* findAxis(const std::string& name) noexcept;
    Axis& axis(std::size_t index) { return *axes_.at(index); }
    std::vector<std::string> axisNames() const;

    // ---- Convenience multi-axis API -------------------------------------
    Status enableAll();
    Status disableAll();
    Status stopAll();
    Status quickStopAll();
    Status resetFaultAll();
    Status moveAbsolute(const std::string& axis, double position, const MoveParams& params = {});
    Status moveRelative(const std::string& axis, double distance, const MoveParams& params = {});
    Status moveVelocity(const std::string& axis, double velocity, double acceleration = 0.0);
    Status stop(const std::string& axis, double deceleration = 0.0);
    /// Waits until every axis is idle (no active motion) or the timeout expires.
    Status waitAllIdle(std::chrono::milliseconds timeout) const;

    /// Validates and enqueues a command. Returns the command id.
    Result<std::uint64_t> submit(AxisCommand command);
    /// Validates all commands, then enqueues them atomically; the cyclic task
    /// applies them in the same cycle (synchronized start). Returns the ids.
    Result<std::vector<std::uint64_t>> submitBatch(std::vector<AxisCommand> commands);
    AxisStatus axisStatus(std::size_t index) const;

    // ---- Integration with the EtherCAT master (non-RT) -------------------
    /// Registers one slave per axis with the master (before configure()).
    Status attach(EthercatMaster& master);
    /// Resolves PDO offsets and creates SDO requests (after configure(),
    /// before activate()). Fails if a configured mode lacks required objects.
    Status resolve(EthercatMaster& master);

    void setSafetyManager(SafetyManager* safety) noexcept { safety_ = safety; }
    void setEventRing(EventRing* ring) noexcept { events_ = ring; }

    /// Drains one command result (single non-RT consumer).
    bool popResult(CommandResult& result) noexcept { return results_.tryPop(result); }

    // ---- CycleHandler ----------------------------------------------------
    void onCycle(const CycleContext& context) noexcept override;

private:
    Status validate(const AxisCommand& command) const;

    std::vector<std::unique_ptr<AxisConfig>> configs_;
    std::vector<std::unique_ptr<Axis>> axes_;
    std::vector<std::unique_ptr<AxisRuntime>> runtimes_;
    std::vector<int> slave_handles_;

    CommandQueue commands_;
    ResultQueue results_;
    std::array<AxisCommand, kMaxAxes> batch_{};  // cyclic task: batch being collected
    std::size_t batch_fill_ = 0;
    std::atomic<std::uint64_t> next_id_{1};
    SafetyManager* safety_ = nullptr;
    SafetyManager own_safety_;
    EventRing* events_ = nullptr;
    bool attached_ = false;
    bool resolved_ = false;
};

}  // namespace ethercat_motion
