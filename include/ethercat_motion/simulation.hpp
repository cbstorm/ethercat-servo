// SPDX-License-Identifier: MIT
//
// Simulation backend: an in-process EtherCAT "bus" of mock CiA 402 drives.
//
// It implements the same EthercatBackend interface as the IgH backend and
// uses the same process image layout mechanics (PDO entries, byte offsets,
// working counter, AL states, SDO requests), so the complete stack above it
// runs unchanged. Faults can be injected from any thread.
//
// The mock drive model is a development aid. It is not a model of any
// particular drive's dynamics or firmware behavior.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ethercat_motion/ethercat_backend.hpp"
#include "ethercat_motion/lockfree.hpp"

namespace ethercat_motion {

struct SimulatedSlaveSpec {
    std::string name = "SimDrive";
    std::uint16_t alias = 0;
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::uint32_t revision = 0;
    std::uint32_t serial = 0;

    double position_time_constant_s = 0.002;
    double velocity_time_constant_s = 0.01;
    double inertia = 0.001;
    double friction = 0.0;
    std::int32_t initial_position_counts = 0;
    std::uint32_t boot_cycles = 5;
    double velocity_factor = 1.0;      ///< drive velocity units per count/s
    double acceleration_factor = 1.0;  ///< drive acceleration units per count/s^2
    std::int32_t home_switch_counts = 0;  ///< Where the simulated home switch sits.
    std::uint32_t supported_modes = 0x3A5;  ///< 0x6502: PP, PV, HM, CSP, CSV, CST
};

/// Read-only view of a simulated drive (for tests and examples).
struct SimulatedDriveSnapshot {
    std::uint16_t controlword = 0;
    std::uint16_t statusword = 0;
    std::int8_t mode = 0;
    std::int8_t mode_display = 0;
    std::int32_t target_position = 0;
    std::int32_t target_velocity = 0;
    std::int16_t target_torque = 0;
    double position = 0.0;     ///< counts
    double velocity = 0.0;     ///< counts/s
    std::uint16_t error_code = 0;
    bool online = true;
    std::uint8_t al_state = 0;
};

/// Thread-safe fault injection and inspection.
class SimulationControl {
public:
    virtual ~SimulationControl() = default;
    /// Drive enters Fault Reaction Active -> Fault with the given 0x603F code.
    virtual void injectDriveFault(int slave, std::uint16_t error_code) = 0;
    /// Slave stops answering (working counter drops, AL state lost).
    /// If the drive was enabled it faults with 0x8130 (lifeguard) like a
    /// real drive whose SM watchdog expires.
    virtual void setSlaveOnline(int slave, bool online) = 0;
    /// Whole link down/up.
    virtual void setLinkUp(bool up) = 0;
    /// Adds a constant offset (counts) between commanded and actual position
    /// in CSP, to provoke following errors.
    virtual void setPositionDisturbance(int slave, double counts) = 0;
    virtual SimulatedDriveSnapshot driveSnapshot(int slave) const = 0;
    virtual std::size_t slaveCount() const = 0;
};

class SimulatedBackend final : public EthercatBackend, public SimulationControl {
public:
    explicit SimulatedBackend(std::vector<SimulatedSlaveSpec> bus);
    ~SimulatedBackend() override;

    // EthercatBackend
    const char* name() const noexcept override { return "simulation"; }
    Status open(unsigned master_index) override;
    Result<std::vector<SlaveIdentity>> scanSlaves() override;
    Result<int> configureSlave(const SlaveDeviceConfig& config, std::uint32_t cycle_time_ns,
                               PdoLayout& layout) override;
    Result<int> createSdoRequest(int slave, std::uint16_t index, std::uint8_t subindex, std::size_t size,
                                 std::uint32_t timeout_ms) override;
    Result<std::uint64_t> sdoUploadBlocking(std::uint16_t ring_position, std::uint16_t index, std::uint8_t subindex,
                                            std::size_t size) override;
    Status activate() override;
    void deactivate() noexcept override;
    void close() noexcept override;

    std::uint8_t* domainData() noexcept override;
    std::size_t domainSize() const noexcept override;
    void receive() noexcept override;
    void send(std::uint64_t app_time_ns, bool sync_dc) noexcept override;
    DomainState domainState() noexcept override;
    std::uint32_t expectedWorkingCounter() const noexcept override;
    MasterLinkState masterState() noexcept override;
    SlaveRuntimeState slaveState(int slave) noexcept override;
    SdoRequestState sdoState(int request) noexcept override;
    std::uint8_t* sdoData(int request) noexcept override;
    void sdoWrite(int request) noexcept override;
    void sdoRead(int request) noexcept override;

    // SimulationControl
    void injectDriveFault(int slave, std::uint16_t error_code) override;
    void setSlaveOnline(int slave, bool online) override;
    void setLinkUp(bool up) override;
    void setPositionDisturbance(int slave, double counts) override;
    SimulatedDriveSnapshot driveSnapshot(int slave) const override;
    std::size_t slaveCount() const override;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace ethercat_motion
