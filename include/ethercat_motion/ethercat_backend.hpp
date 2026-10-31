// SPDX-License-Identifier: MIT
//
// Backend interface between the EtherCAT master manager and the actual
// transport. Two implementations exist:
//   * IghBackend       - IgH EtherCAT Master (ecrt API), hardware.
//   * SimulatedBackend - in-process simulation with mock CiA 402 drives.
//
// Raw IgH objects never leave the IgH backend implementation.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ethercat_motion/pdo_mapping.hpp"
#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

enum class WorkingCounterState : std::uint8_t { Zero, Incomplete, Complete };

struct DomainState {
    std::uint32_t working_counter = 0;
    WorkingCounterState wc_state = WorkingCounterState::Zero;
};

struct MasterLinkState {
    std::uint32_t slaves_responding = 0;
    std::uint8_t al_states = 0;   ///< Bitwise OR of AL states of all slaves.
    bool link_up = false;
};

struct SlaveRuntimeState {
    bool online = false;
    bool operational = false;
    std::uint8_t al_state = 0;    ///< 1 INIT, 2 PREOP, 3 BOOT, 4 SAFEOP, 8 OP
};

/// Identity of a slave found on the bus.
struct SlaveIdentity {
    std::uint16_t ring_position = 0;
    std::uint16_t alias = 0;
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::uint32_t revision = 0;
    std::uint32_t serial = 0;
    std::uint8_t al_state = 0;
    std::string name;
};

enum class SdoRequestState : std::uint8_t { Unused, Busy, Success, Error };

class EthercatBackend {
public:
    virtual ~EthercatBackend() = default;

    virtual const char* name() const noexcept = 0;

    // ---- Non-real-time configuration phase -------------------------------
    virtual Status open(unsigned master_index) = 0;
    virtual Result<std::vector<SlaveIdentity>> scanSlaves() = 0;
    /// Configures PDOs, startup SDOs and DC; registers every mapped entry
    /// and appends its location to `layout`. Returns a slave handle.
    virtual Result<int> configureSlave(const SlaveDeviceConfig& config, std::uint32_t cycle_time_ns,
                                       PdoLayout& layout) = 0;
    /// Creates a non-blocking SDO request usable from the cyclic task.
    virtual Result<int> createSdoRequest(int slave, std::uint16_t index, std::uint8_t subindex, std::size_t size,
                                         std::uint32_t timeout_ms) = 0;
    /// Blocking SDO upload for start-up checks. Non-RT, before activation only.
    virtual Result<std::uint64_t> sdoUploadBlocking(std::uint16_t ring_position, std::uint16_t index,
                                                    std::uint8_t subindex, std::size_t size) = 0;
    virtual Status activate() = 0;
    virtual void deactivate() noexcept = 0;
    virtual void close() noexcept = 0;

    // ---- Real-time cyclic phase (no allocation, no blocking) -------------
    virtual std::uint8_t* domainData() noexcept = 0;
    virtual std::size_t domainSize() const noexcept = 0;
    /// Receive frames and process the domain.
    virtual void receive() noexcept = 0;
    /// Queue the domain, synchronize distributed clocks, send frames.
    virtual void send(std::uint64_t app_time_ns, bool sync_dc) noexcept = 0;
    virtual DomainState domainState() noexcept = 0;
    virtual std::uint32_t expectedWorkingCounter() const noexcept = 0;
    virtual MasterLinkState masterState() noexcept = 0;
    virtual SlaveRuntimeState slaveState(int slave) noexcept = 0;

    virtual SdoRequestState sdoState(int request) noexcept = 0;
    virtual std::uint8_t* sdoData(int request) noexcept = 0;
    virtual void sdoWrite(int request) noexcept = 0;
    virtual void sdoRead(int request) noexcept = 0;
};

/// True when the library was compiled with the IgH EtherCAT master.
bool ighBackendAvailable() noexcept;

/// Creates the IgH backend, or an error when the library was built without it.
Result<std::unique_ptr<EthercatBackend>> createIghBackend();

}  // namespace ethercat_motion
