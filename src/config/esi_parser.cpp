// SPDX-License-Identifier: MIT
#include "ethercat_motion/esi_parser.hpp"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

#include "xml_reader.hpp"

namespace ethercat_motion {

std::optional<std::uint64_t> parseEsiNumber(const std::string& raw) noexcept {
    std::string text;
    for (char c : raw) {
        if (!std::isspace(static_cast<unsigned char>(c))) text += c;
    }
    if (text.empty()) return std::nullopt;
    int base = 10;
    std::size_t start = 0;
    if (text.size() > 2 && text[0] == '#' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        start = 2;
    } else if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        start = 2;
    }
    std::uint64_t value = 0;
    for (std::size_t i = start; i < text.size(); ++i) {
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
        int digit = -1;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (base == 16 && c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        if (digit < 0) return std::nullopt;
        value = value * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(digit);
    }
    return value;
}

namespace {

std::uint64_t numberOr(const std::string& text, std::uint64_t fallback) {
    const auto v = parseEsiNumber(text);
    return v ? *v : fallback;
}

std::uint64_t attrNumber(const xml::Node& node, const char* name, std::uint64_t fallback) {
    const std::string* a = node.attribute(name);
    return a != nullptr ? numberOr(*a, fallback) : fallback;
}

bool attrBool(const xml::Node& node, const char* name) {
    const std::string* a = node.attribute(name);
    return a != nullptr && (*a == "1" || *a == "true");
}

/// "0100" -> 1: ESI <Data>/<DefaultData> are little-endian hex byte strings.
std::uint64_t littleEndianHex(const std::string& data) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i + 1 < data.size() && i / 2 < 8; i += 2) {
        const auto byte = parseEsiNumber("0x" + data.substr(i, 2));
        value |= static_cast<std::uint64_t>(byte.value_or(0)) << (4 * i);
    }
    return value;
}

EsiPdo parsePdo(const xml::Node& node, PdoDirection direction) {
    EsiPdo pdo;
    pdo.direction = direction;
    pdo.index = static_cast<std::uint16_t>(numberOr(node.childText("Index"), 0));
    pdo.name = node.childText("Name");
    pdo.fixed = attrBool(node, "Fixed");
    pdo.mandatory = attrBool(node, "Mandatory");
    if (const std::string* sm = node.attribute("Sm")) {
        if (const auto v = parseEsiNumber(*sm)) pdo.sm = static_cast<std::uint8_t>(*v);
    }
    for (const xml::Node* ex : node.childrenNamed("Exclude")) {
        pdo.excludes.push_back(static_cast<std::uint16_t>(numberOr(ex->text, 0)));
    }
    for (const xml::Node* e : node.childrenNamed("Entry")) {
        PdoEntryConfig entry;
        entry.index = static_cast<std::uint16_t>(numberOr(e->childText("Index"), 0));
        entry.subindex = static_cast<std::uint8_t>(numberOr(e->childText("SubIndex"), 0));
        entry.bit_length = static_cast<std::uint8_t>(numberOr(e->childText("BitLen"), 0));
        entry.name = e->childText("Name");
        pdo.entries.push_back(entry);
    }
    return pdo;
}

EsiDevice parseDevice(const xml::Node& dev, std::uint32_t vendor_id) {
    EsiDevice device;
    device.vendor_id = vendor_id;
    if (const xml::Node* type = dev.child("Type")) {
        device.type_name = type->text;
        device.product_code = static_cast<std::uint32_t>(attrNumber(*type, "ProductCode", 0));
        device.revision = static_cast<std::uint32_t>(attrNumber(*type, "RevisionNo", 0));
    }
    device.name = dev.childText("Name");
    if (const xml::Node* profile = dev.child("Profile")) {
        device.profile = static_cast<std::uint32_t>(numberOr(profile->childText("ProfileNo"), 0));
        if (const xml::Node* dict = profile->child("Dictionary")) {
            if (const xml::Node* objects = dict->child("Objects")) {
                for (const xml::Node* o : objects->childrenNamed("Object")) {
                    EsiObject obj;
                    obj.index = static_cast<std::uint16_t>(numberOr(o->childText("Index"), 0));
                    obj.name = o->childText("Name");
                    obj.type = o->childText("Type");
                    obj.bit_size = static_cast<std::uint32_t>(numberOr(o->childText("BitSize"), 0));
                    if (const xml::Node* flags = o->child("Flags")) {
                        obj.access = flags->childText("Access");
                        obj.pdo_mapping = flags->childText("PdoMapping");
                    }
                    if (const xml::Node* info = o->child("Info")) {
                        if (const auto v = parseEsiNumber(info->childText("DefaultValue"))) {
                            obj.default_value = *v;
                        } else if (const std::string data = info->childText("DefaultData"); !data.empty()) {
                            obj.default_value = littleEndianHex(data);
                        }
                    }
                    device.objects.push_back(obj);
                }
            }
        }
    }

    std::uint8_t sm_index = 0;
    for (const xml::Node* sm : dev.childrenNamed("Sm")) {
        EsiSyncManager s;
        s.index = sm_index++;
        s.type = sm->text;
        s.start_address = static_cast<std::uint16_t>(attrNumber(*sm, "StartAddress", 0));
        s.default_size = static_cast<std::uint16_t>(attrNumber(*sm, "DefaultSize", 0));
        device.sync_managers.push_back(s);
    }
    for (const xml::Node* p : dev.childrenNamed("RxPdo")) device.rx_pdos.push_back(parsePdo(*p, PdoDirection::Output));
    for (const xml::Node* p : dev.childrenNamed("TxPdo")) device.tx_pdos.push_back(parsePdo(*p, PdoDirection::Input));

    if (const xml::Node* mailbox = dev.child("Mailbox")) {
        if (const xml::Node* coe = mailbox->child("CoE")) {
            for (const xml::Node* cmd : coe->childrenNamed("InitCmd")) {
                EsiInitCmd init;
                for (const xml::Node* t : cmd->childrenNamed("Transition")) init.transitions += t->text;
                init.sdo.index = static_cast<std::uint16_t>(numberOr(cmd->childText("Index"), 0));
                init.sdo.subindex = static_cast<std::uint8_t>(numberOr(cmd->childText("SubIndex"), 0));
                init.sdo.comment = cmd->childText("Comment");
                // <Data> is a little-endian hex byte string, e.g. "0100" = 1 (UINT16).
                const std::string data = cmd->childText("Data");
                init.sdo.size = static_cast<std::uint8_t>(data.size() / 2);
                init.sdo.value = static_cast<std::int64_t>(littleEndianHex(data));
                device.init_cmds.push_back(init);
            }
        }
    }

    if (const xml::Node* dc = dev.child("Dc")) {
        for (const xml::Node* om : dc->childrenNamed("OpMode")) {
            EsiDcOpMode mode;
            mode.name = om->childText("Name");
            mode.description = om->childText("Desc");
            mode.assign_activate = static_cast<std::uint16_t>(numberOr(om->childText("AssignActivate"), 0));
            device.dc_modes.push_back(mode);
        }
    }
    return device;
}

}  // namespace

std::vector<SyncManagerConfig> EsiDevice::defaultSyncConfig() const {
    std::map<std::uint8_t, SyncManagerConfig> by_sm;
    auto add = [&](const EsiPdo& pdo) {
        if (!pdo.sm) return;
        auto& sm = by_sm[*pdo.sm];
        sm.index = *pdo.sm;
        sm.direction = pdo.direction;
        PdoConfig cfg;
        cfg.index = pdo.index;
        cfg.entries = pdo.entries;
        sm.pdos.push_back(cfg);
    };
    for (const auto& p : rx_pdos) add(p);
    for (const auto& p : tx_pdos) add(p);
    std::vector<SyncManagerConfig> out;
    for (auto& [index, sm] : by_sm) out.push_back(sm);
    return out;
}

std::vector<SdoInitCommand> EsiDevice::startupSdos() const {
    std::vector<SdoInitCommand> out;
    for (const auto& cmd : init_cmds) {
        const bool ps = cmd.transitions.find("PS") != std::string::npos;
        const bool sized = cmd.sdo.size == 1 || cmd.sdo.size == 2 || cmd.sdo.size == 4;
        if (ps && sized) out.push_back(cmd.sdo);
    }
    return out;
}

const EsiObject* EsiDevice::findObject(std::uint16_t index) const noexcept {
    for (const auto& o : objects) {
        if (o.index == index) return &o;
    }
    return nullptr;
}

bool EsiDevice::isMappable(std::uint16_t index, PdoDirection direction) const noexcept {
    const EsiObject* o = findObject(index);
    if (o == nullptr) return false;
    const char needed = direction == PdoDirection::Output ? 'R' : 'T';
    for (char c : o->pdo_mapping) {
        if (std::toupper(static_cast<unsigned char>(c)) == needed) return true;
    }
    return false;
}

std::optional<std::uint16_t> EsiDevice::dcAssignActivate() const noexcept {
    for (const auto& mode : dc_modes) {
        if (mode.assign_activate != 0) return mode.assign_activate;
    }
    return std::nullopt;
}

std::optional<std::uint32_t> EsiDevice::supportedDriveModes() const noexcept {
    const EsiObject* o = findObject(0x6502);
    if (o == nullptr || !o->default_value) return std::nullopt;
    return static_cast<std::uint32_t>(*o->default_value);
}

const EsiDevice* EsiFile::find(std::uint32_t product_code, std::optional<std::uint32_t> revision) const {
    for (const auto& d : devices) {
        if (d.product_code == product_code && (!revision || d.revision == *revision)) return &d;
    }
    return nullptr;
}

Result<EsiFile> parseEsiString(const std::string& text) {
    auto root = xml::parse(text);
    if (!root) return root.status();
    const xml::Node& info = *root.value();
    if (info.name != "EtherCATInfo") {
        return Status(ErrorCode::InvalidConfig,
                      "not an EtherCAT ESI file (root element <" + info.name +
                          ">, expected <EtherCATInfo>); note that CANopen EDS files are not ESI files");
    }
    EsiFile file;
    if (const xml::Node* vendor = info.child("Vendor")) {
        file.vendor_id = static_cast<std::uint32_t>(numberOr(vendor->childText("Id"), 0));
        file.vendor_name = vendor->childText("Name");
    }
    const xml::Node* descriptions = info.child("Descriptions");
    const xml::Node* devices = descriptions != nullptr ? descriptions->child("Devices") : nullptr;
    if (devices == nullptr) {
        return Status(ErrorCode::InvalidConfig, "ESI file has no <Descriptions><Devices>");
    }
    for (const xml::Node* dev : devices->childrenNamed("Device")) {
        file.devices.push_back(parseDevice(*dev, file.vendor_id));
    }
    return file;
}

Result<EsiFile> parseEsiFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return Status(ErrorCode::IoError, "cannot open ESI file '" + path + "'");
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    auto file = parseEsiString(buffer.str());
    if (!file) {
        return Status(file.status().code(), path + ": " + file.status().message());
    }
    return file;
}

std::string esiDeviceToYaml(const EsiDevice& device) {
    std::ostringstream y;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "# %s (%s)\nvendor_id: 0x%08X\nproduct_code: 0x%08X\nrevision: 0x%08X\n",
                  device.name.c_str(), device.type_name.c_str(), device.vendor_id, device.product_code,
                  device.revision);
    y << buf;
    y << "pdo:\n  source: config\n  sync_managers:\n";
    for (const auto& sm : device.defaultSyncConfig()) {
        y << "    - index: " << static_cast<unsigned>(sm.index) << "\n";
        y << "      direction: " << (sm.direction == PdoDirection::Output ? "output" : "input") << "\n";
        y << "      pdos:\n";
        for (const auto& pdo : sm.pdos) {
            std::snprintf(buf, sizeof(buf), "        - index: 0x%04X\n          entries:\n", pdo.index);
            y << buf;
            for (const auto& e : pdo.entries) {
                std::snprintf(buf, sizeof(buf), "            - [0x%04X, 0x%02X, %u]  # %s\n", e.index, e.subindex,
                              e.bit_length, e.name.c_str());
                y << buf;
            }
        }
    }
    const auto sdos = device.startupSdos();
    if (!sdos.empty()) {
        y << "sdo_init:\n";
        for (const auto& s : sdos) {
            std::snprintf(buf, sizeof(buf), "  - {index: 0x%04X, subindex: %u, size: %u, value: %lld}  # %s\n",
                          s.index, s.subindex, s.size, static_cast<long long>(s.value), s.comment.c_str());
            y << buf;
        }
    }
    if (const auto aa = device.dcAssignActivate()) {
        std::snprintf(buf, sizeof(buf), "dc:\n  enabled: true\n  assign_activate: 0x%04X\n", *aa);
        y << buf;
    }
    return y.str();
}

}  // namespace ethercat_motion
