// SPDX-License-Identifier: MIT
#include "ethercat_motion/ethercat_slave.hpp"

#include <cstdio>
#include <string>

namespace ethercat_motion {

Result<std::uint16_t> resolveRingPosition(std::uint16_t alias, std::uint16_t position,
                                          const std::vector<SlaveIdentity>& scan) {
    if (alias == 0) {
        if (position >= scan.size()) {
            return Status(ErrorCode::NotFound, "no slave at ring position " + std::to_string(position) + " (" +
                                                   std::to_string(scan.size()) + " slave(s) found)");
        }
        return position;
    }
    for (std::size_t i = 0; i < scan.size(); ++i) {
        if (scan[i].alias == alias) {
            const std::size_t target = i + position;
            if (target >= scan.size()) {
                break;
            }
            return static_cast<std::uint16_t>(target);
        }
    }
    return Status(ErrorCode::NotFound,
                  "no slave at alias " + std::to_string(alias) + " position " + std::to_string(position));
}

Status validateIdentity(const SlaveDeviceConfig& expected, const SlaveIdentity& found) {
    char buf[256];
    if (found.vendor_id != expected.vendor_id || found.product_code != expected.product_code) {
        std::snprintf(buf, sizeof(buf),
                      "slave '%s' at ring position %u: expected vendor 0x%08X product 0x%08X, found vendor 0x%08X "
                      "product 0x%08X ('%s')",
                      expected.name.c_str(), found.ring_position, expected.vendor_id, expected.product_code,
                      found.vendor_id, found.product_code, found.name.c_str());
        return {ErrorCode::DeviceMismatch, buf};
    }
    if (expected.revision && *expected.revision != found.revision) {
        std::snprintf(buf, sizeof(buf), "slave '%s' at ring position %u: expected revision 0x%08X, found 0x%08X",
                      expected.name.c_str(), found.ring_position, *expected.revision, found.revision);
        return {ErrorCode::DeviceMismatch, buf};
    }
    return Status::ok();
}

const char* alStateName(std::uint8_t al_state) noexcept {
    switch (al_state & 0x0F) {
        case 0x01: return "INIT";
        case 0x02: return "PREOP";
        case 0x03: return "BOOT";
        case 0x04: return "SAFEOP";
        case 0x08: return "OP";
        default: return "UNKNOWN";
    }
}

}  // namespace ethercat_motion
