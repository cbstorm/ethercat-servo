// SPDX-License-Identifier: MIT
//
// PDO mapping manager: configurable sync manager / PDO / entry descriptions,
// startup SDOs, distributed-clock settings, and the resolved process-image
// layout produced after PDO entry registration.
//
// Nothing here is hardcoded for a particular drive. Mappings come from the
// YAML configuration or from an EtherCAT ESI file (see esi_parser.hpp).
#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

/// Direction seen from the master: Output = RxPDO (master -> slave),
/// Input = TxPDO (slave -> master).
enum class PdoDirection : std::uint8_t { Output, Input };

struct PdoEntryConfig {
    std::uint16_t index = 0;      ///< 0 = padding / gap entry.
    std::uint8_t subindex = 0;
    std::uint8_t bit_length = 0;
    std::string name;
};

struct PdoConfig {
    std::uint16_t index = 0;      ///< e.g. 0x1600 / 0x1A00
    std::vector<PdoEntryConfig> entries;
};

struct SyncManagerConfig {
    std::uint8_t index = 0;       ///< 2 = outputs, 3 = inputs for most CoE slaves.
    PdoDirection direction = PdoDirection::Output;
    std::vector<PdoConfig> pdos;
    bool watchdog = true;         ///< Enable the slave's SM watchdog for outputs.
};

/// SDO written by the master during slave configuration (PREOP -> SAFEOP).
struct SdoInitCommand {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t size = 0;        ///< 1, 2 or 4 bytes.
    std::int64_t value = 0;
    std::string comment;
};

struct DcConfig {
    bool enabled = false;
    std::uint16_t assign_activate = 0x0300;  ///< From ESI <Dc><OpMode><AssignActivate>.
    std::int32_t sync0_shift_ns = 0;
};

/// Everything needed to configure one slave on the bus.
struct SlaveDeviceConfig {
    std::string name;
    std::uint16_t alias = 0;
    std::uint16_t position = 0;
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::optional<std::uint32_t> revision;   ///< When set, must match exactly.
    std::vector<SyncManagerConfig> syncs;
    std::vector<SdoInitCommand> sdo_init;
    DcConfig dc;
};

/// Location of one registered PDO entry inside the domain process image.
struct PdoEntryLocation {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t bit_length = 0;
    PdoDirection direction = PdoDirection::Output;
    std::uint32_t byte_offset = 0;
    std::uint8_t bit_offset = 0;
};

/// Resolved process image layout of one slave.
class PdoLayout {
public:
    void add(const PdoEntryLocation& location) { entries_.push_back(location); }
    const PdoEntryLocation* find(std::uint16_t index, std::uint8_t subindex) const noexcept;
    const std::vector<PdoEntryLocation>& entries() const noexcept { return entries_; }
    bool empty() const noexcept { return entries_.empty(); }

private:
    std::vector<PdoEntryLocation> entries_;
};

/// Returns all non-padding entries of all PDOs of the sync managers.
std::vector<std::pair<PdoEntryConfig, PdoDirection>> collectEntries(const std::vector<SyncManagerConfig>& syncs);

/// Structural validation: entry sizes, duplicate objects, SM indices.
Status validateSyncConfig(const std::vector<SyncManagerConfig>& syncs, const std::string& context);

/// Little-endian process-image accessors (EtherCAT is little endian).
namespace pd {

inline std::uint8_t readU8(const std::uint8_t* base, std::uint32_t offset) noexcept { return base[offset]; }
inline std::int8_t readS8(const std::uint8_t* base, std::uint32_t offset) noexcept {
    return static_cast<std::int8_t>(base[offset]);
}
inline std::uint16_t readU16(const std::uint8_t* base, std::uint32_t offset) noexcept {
    return static_cast<std::uint16_t>(base[offset] | (base[offset + 1] << 8));
}
inline std::int16_t readS16(const std::uint8_t* base, std::uint32_t offset) noexcept {
    return static_cast<std::int16_t>(readU16(base, offset));
}
inline std::uint32_t readU32(const std::uint8_t* base, std::uint32_t offset) noexcept {
    return static_cast<std::uint32_t>(base[offset]) | (static_cast<std::uint32_t>(base[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(base[offset + 2]) << 16) | (static_cast<std::uint32_t>(base[offset + 3]) << 24);
}
inline std::int32_t readS32(const std::uint8_t* base, std::uint32_t offset) noexcept {
    return static_cast<std::int32_t>(readU32(base, offset));
}

inline void writeU8(std::uint8_t* base, std::uint32_t offset, std::uint8_t v) noexcept { base[offset] = v; }
inline void writeU16(std::uint8_t* base, std::uint32_t offset, std::uint16_t v) noexcept {
    base[offset] = static_cast<std::uint8_t>(v & 0xFF);
    base[offset + 1] = static_cast<std::uint8_t>(v >> 8);
}
inline void writeU32(std::uint8_t* base, std::uint32_t offset, std::uint32_t v) noexcept {
    base[offset] = static_cast<std::uint8_t>(v & 0xFF);
    base[offset + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    base[offset + 2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
    base[offset + 3] = static_cast<std::uint8_t>(v >> 24);
}

/// Reads an entry of 8/16/32 bits, sign-extending when `is_signed`.
std::int64_t readEntry(const std::uint8_t* base, const PdoEntryLocation& loc, bool is_signed) noexcept;
/// Writes an entry of 8/16/32 bits (value is truncated to the entry size).
void writeEntry(std::uint8_t* base, const PdoEntryLocation& loc, std::int64_t value) noexcept;

}  // namespace pd

/// Well-known CiA 402 object indices.
namespace cia402_obj {
inline constexpr std::uint16_t kErrorCode = 0x603F;
inline constexpr std::uint16_t kControlword = 0x6040;
inline constexpr std::uint16_t kStatusword = 0x6041;
inline constexpr std::uint16_t kQuickStopOptionCode = 0x605A;
inline constexpr std::uint16_t kModesOfOperation = 0x6060;
inline constexpr std::uint16_t kModesOfOperationDisplay = 0x6061;
inline constexpr std::uint16_t kPositionActual = 0x6064;
inline constexpr std::uint16_t kVelocityActual = 0x606C;
inline constexpr std::uint16_t kTargetTorque = 0x6071;
inline constexpr std::uint16_t kTorqueActual = 0x6077;
inline constexpr std::uint16_t kTargetPosition = 0x607A;
inline constexpr std::uint16_t kHomeOffset = 0x607C;
inline constexpr std::uint16_t kProfileVelocity = 0x6081;
inline constexpr std::uint16_t kProfileAcceleration = 0x6083;
inline constexpr std::uint16_t kProfileDeceleration = 0x6084;
inline constexpr std::uint16_t kQuickStopDeceleration = 0x6085;
inline constexpr std::uint16_t kTorqueSlope = 0x6087;
inline constexpr std::uint16_t kHomingMethod = 0x6098;
inline constexpr std::uint16_t kHomingSpeeds = 0x6099;      // :1 switch search, :2 zero search
inline constexpr std::uint16_t kHomingAcceleration = 0x609A;
inline constexpr std::uint16_t kFollowingErrorActual = 0x60F4;
inline constexpr std::uint16_t kDigitalInputs = 0x60FD;
inline constexpr std::uint16_t kTargetVelocity = 0x60FF;
inline constexpr std::uint16_t kSupportedDriveModes = 0x6502;
}  // namespace cia402_obj

}  // namespace ethercat_motion
