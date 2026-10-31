// SPDX-License-Identifier: MIT
//
// EtherCAT slave manager helpers: resolving alias/position addressing to a
// ring position and validating slave identity before configuration.
#pragma once

#include <cstdint>
#include <vector>

#include "ethercat_motion/ethercat_backend.hpp"
#include "ethercat_motion/pdo_mapping.hpp"

namespace ethercat_motion {

/// Resolves IgH-style (alias, position) addressing against a bus scan.
/// alias == 0: position is the absolute ring position.
/// alias != 0: position is relative to the first slave carrying that alias.
Result<std::uint16_t> resolveRingPosition(std::uint16_t alias, std::uint16_t position,
                                          const std::vector<SlaveIdentity>& scan);

/// Verifies vendor ID, product code and (when configured) revision.
/// Returns ErrorCode::DeviceMismatch with a descriptive message on mismatch.
Status validateIdentity(const SlaveDeviceConfig& expected, const SlaveIdentity& found);

/// Runtime view of one configured slave (published by the master).
struct SlaveStatus {
    std::uint16_t ring_position = 0;
    bool online = false;
    bool operational = false;
    std::uint8_t al_state = 0;
};

const char* alStateName(std::uint8_t al_state) noexcept;

}  // namespace ethercat_motion
