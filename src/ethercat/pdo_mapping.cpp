// SPDX-License-Identifier: MIT
#include "ethercat_motion/pdo_mapping.hpp"

#include <cstdio>
#include <set>
#include <utility>

namespace ethercat_motion {

const PdoEntryLocation* PdoLayout::find(std::uint16_t index, std::uint8_t subindex) const noexcept {
    for (const auto& entry : entries_) {
        if (entry.index == index && entry.subindex == subindex) {
            return &entry;
        }
    }
    return nullptr;
}

std::vector<std::pair<PdoEntryConfig, PdoDirection>> collectEntries(const std::vector<SyncManagerConfig>& syncs) {
    std::vector<std::pair<PdoEntryConfig, PdoDirection>> out;
    for (const auto& sm : syncs) {
        for (const auto& pdo : sm.pdos) {
            for (const auto& entry : pdo.entries) {
                if (entry.index != 0) {
                    out.emplace_back(entry, sm.direction);
                }
            }
        }
    }
    return out;
}

Status validateSyncConfig(const std::vector<SyncManagerConfig>& syncs, const std::string& context) {
    if (syncs.empty()) {
        return {ErrorCode::InvalidConfig, context + ": no sync manager / PDO configuration"};
    }
    std::set<std::uint8_t> sm_indices;
    std::set<std::uint16_t> pdo_indices;
    std::set<std::uint32_t> objects;
    char buf[96];
    for (const auto& sm : syncs) {
        if (!sm_indices.insert(sm.index).second) {
            return {ErrorCode::InvalidConfig, context + ": duplicate sync manager " + std::to_string(sm.index)};
        }
        if (sm.index < 2) {
            return {ErrorCode::InvalidConfig,
                    context + ": sync managers 0/1 are mailbox SMs and cannot carry PDOs"};
        }
        for (const auto& pdo : sm.pdos) {
            if (!pdo_indices.insert(pdo.index).second) {
                std::snprintf(buf, sizeof(buf), ": PDO 0x%04X assigned twice", pdo.index);
                return {ErrorCode::InvalidConfig, context + buf};
            }
            const bool rx_range = pdo.index >= 0x1600 && pdo.index <= 0x17FF;
            const bool tx_range = pdo.index >= 0x1A00 && pdo.index <= 0x1BFF;
            if ((sm.direction == PdoDirection::Output && !rx_range) ||
                (sm.direction == PdoDirection::Input && !tx_range)) {
                std::snprintf(buf, sizeof(buf), ": PDO 0x%04X does not match SM%u direction", pdo.index, sm.index);
                return {ErrorCode::InvalidConfig, context + buf};
            }
            for (const auto& entry : pdo.entries) {
                if (entry.bit_length == 0) {
                    std::snprintf(buf, sizeof(buf), ": entry 0x%04X:%02X has zero bit length", entry.index,
                                  entry.subindex);
                    return {ErrorCode::InvalidConfig, context + buf};
                }
                if (entry.index == 0) {
                    continue;  // padding
                }
                const std::uint32_t key = (static_cast<std::uint32_t>(entry.index) << 8) | entry.subindex;
                if (!objects.insert(key).second) {
                    std::snprintf(buf, sizeof(buf), ": object 0x%04X:%02X mapped twice", entry.index,
                                  entry.subindex);
                    return {ErrorCode::InvalidConfig, context + buf};
                }
            }
        }
    }
    return Status::ok();
}

namespace pd {

std::int64_t readEntry(const std::uint8_t* base, const PdoEntryLocation& loc, bool is_signed) noexcept {
    switch (loc.bit_length) {
        case 8:
            return is_signed ? static_cast<std::int64_t>(readS8(base, loc.byte_offset)) : readU8(base, loc.byte_offset);
        case 16:
            return is_signed ? static_cast<std::int64_t>(readS16(base, loc.byte_offset))
                             : readU16(base, loc.byte_offset);
        case 32:
            return is_signed ? static_cast<std::int64_t>(readS32(base, loc.byte_offset))
                             : readU32(base, loc.byte_offset);
        default:
            return 0;
    }
}

void writeEntry(std::uint8_t* base, const PdoEntryLocation& loc, std::int64_t value) noexcept {
    switch (loc.bit_length) {
        case 8:
            writeU8(base, loc.byte_offset, static_cast<std::uint8_t>(value & 0xFF));
            break;
        case 16:
            writeU16(base, loc.byte_offset, static_cast<std::uint16_t>(value & 0xFFFF));
            break;
        case 32:
            writeU32(base, loc.byte_offset, static_cast<std::uint32_t>(value & 0xFFFFFFFF));
            break;
        default:
            break;
    }
}

}  // namespace pd

}  // namespace ethercat_motion
