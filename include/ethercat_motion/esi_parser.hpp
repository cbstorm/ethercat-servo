// SPDX-License-Identifier: MIT
//
// EtherCAT Slave Information (ESI, ETG.2000) XML reader.
//
// Extracts identity, default PDO assignment (PDOs carrying an Sm attribute),
// all PDO definitions, CoE InitCmds and DC operation modes so they can be
// used as the device configuration. An ESI file describes what a device
// *claims* to support; the real firmware must still be verified (see
// docs/hardware_setup.md). A CANopen EDS file is NOT an ESI file.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ethercat_motion/pdo_mapping.hpp"
#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

struct EsiPdo {
    std::uint16_t index = 0;
    std::string name;
    PdoDirection direction = PdoDirection::Output;
    bool fixed = false;
    bool mandatory = false;
    std::optional<std::uint8_t> sm;      ///< Default assignment when present.
    std::vector<std::uint16_t> excludes;
    std::vector<PdoEntryConfig> entries;
};

struct EsiDcOpMode {
    std::string name;
    std::string description;
    std::uint16_t assign_activate = 0;
};

struct EsiObject {
    std::uint16_t index = 0;
    std::string name;
    std::string type;
    std::uint32_t bit_size = 0;
    std::string access;        ///< ro / rw / wo
    std::string pdo_mapping;   ///< "R", "T", "RT" or empty
    std::optional<std::uint64_t> default_value;  ///< <DefaultValue> or <DefaultData> (little endian)
};

struct EsiInitCmd {
    std::string transitions;   ///< e.g. "PS"
    SdoInitCommand sdo;
};

struct EsiSyncManager {
    std::uint8_t index = 0;
    std::string type;          ///< MBoxOut, MBoxIn, Outputs, Inputs
    std::uint16_t start_address = 0;
    std::uint16_t default_size = 0;
};

struct EsiDevice {
    std::string type_name;
    std::string name;
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::uint32_t revision = 0;
    std::uint32_t profile = 0;   ///< 402 for drives
    std::vector<EsiSyncManager> sync_managers;
    std::vector<EsiPdo> rx_pdos;
    std::vector<EsiPdo> tx_pdos;
    std::vector<EsiInitCmd> init_cmds;
    std::vector<EsiDcOpMode> dc_modes;
    std::vector<EsiObject> objects;

    /// Sync manager configuration of the *default* PDO assignment
    /// (PDOs with an Sm attribute).
    std::vector<SyncManagerConfig> defaultSyncConfig() const;
    /// CoE InitCmds executed during PREOP->SAFEOP (transition "PS").
    std::vector<SdoInitCommand> startupSdos() const;
    const EsiObject* findObject(std::uint16_t index) const noexcept;
    /// True when the ESI marks the object as mappable in the given direction.
    bool isMappable(std::uint16_t index, PdoDirection direction) const noexcept;
    std::optional<std::uint16_t> dcAssignActivate() const noexcept;
    /// Default value of 0x6502 "Supported drive modes", when declared.
    std::optional<std::uint32_t> supportedDriveModes() const noexcept;
};

struct EsiFile {
    std::uint32_t vendor_id = 0;
    std::string vendor_name;
    std::vector<EsiDevice> devices;

    /// Finds a device by product code and optional revision.
    const EsiDevice* find(std::uint32_t product_code, std::optional<std::uint32_t> revision = std::nullopt) const;
};

Result<EsiFile> parseEsiFile(const std::string& path);
Result<EsiFile> parseEsiString(const std::string& xml);

/// ESI numeric notation: "#x1A00", "0x1A00" or decimal.
std::optional<std::uint64_t> parseEsiNumber(const std::string& text) noexcept;

/// YAML snippet (axes[].pdo / sdo_init) reproducing a device's defaults.
std::string esiDeviceToYaml(const EsiDevice& device);

}  // namespace ethercat_motion
