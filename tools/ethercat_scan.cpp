// SPDX-License-Identifier: MIT
//
// ethercat_motion_scan - bus scan and ESI inspection.
//
//   ethercat_motion_scan [--master N] [--esi FILE]   scan the bus (IgH), match slaves to the ESI,
//                                                     read 0x6502 supported drive modes
//   ethercat_motion_scan --esi FILE --list            list devices described by an ESI file
//   ethercat_motion_scan --esi FILE --dump 0x5800[:0x1]
//                                                     print a YAML device configuration snippet
//
// Read-only: never activates the master and never writes to a drive.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <ethercat_motion/ethercat_motion.hpp>

using namespace ethercat_motion;

namespace {

std::string option(int argc, char** argv, const char* name) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
    }
    return {};
}

bool flag(int argc, char** argv, const char* name) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0) return true;
    }
    return false;
}

std::string modes(std::uint32_t mask) {
    std::string out;
    for (OperationMode m : {OperationMode::PP, OperationMode::PV, OperationMode::Homing, OperationMode::CSP,
                            OperationMode::CSV, OperationMode::CST}) {
        if (mask & cia402::supportedModeBit(m)) out += std::string(out.empty() ? "" : " ") + toString(m);
    }
    return out.empty() ? "-" : out;
}

void usage() {
    std::printf("usage: ethercat_motion_scan [--master N] [--esi FILE]\n"
                "       ethercat_motion_scan --esi FILE --list\n"
                "       ethercat_motion_scan --esi FILE --dump PRODUCT[:REVISION]\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (flag(argc, argv, "--help") || flag(argc, argv, "-h")) {
        usage();
        return 0;
    }
    const std::string esi_path = option(argc, argv, "--esi");
    std::optional<EsiFile> esi;
    if (!esi_path.empty()) {
        auto parsed = parseEsiFile(esi_path);
        if (!parsed) {
            std::fprintf(stderr, "%s\n", parsed.status().toString().c_str());
            return 1;
        }
        esi = std::move(parsed.value());
    }

    if (esi && flag(argc, argv, "--list")) {
        std::printf("vendor 0x%08X '%s'\n", esi->vendor_id, esi->vendor_name.c_str());
        for (const auto& d : esi->devices) {
            const auto m = d.supportedDriveModes();
            std::printf("  product 0x%08X rev 0x%08X  %-10s profile %u  modes (ESI 0x6502): %s\n", d.product_code,
                        d.revision, d.name.c_str(), d.profile, m ? modes(*m).c_str() : "not declared");
        }
        return 0;
    }
    if (const std::string dump = option(argc, argv, "--dump"); esi && !dump.empty()) {
        const auto colon = dump.find(':');
        const auto product = parseEsiNumber(dump.substr(0, colon));
        std::optional<std::uint32_t> revision;
        if (colon != std::string::npos) {
            if (auto r = parseEsiNumber(dump.substr(colon + 1))) revision = static_cast<std::uint32_t>(*r);
        }
        const EsiDevice* d = product ? esi->find(static_cast<std::uint32_t>(*product), revision) : nullptr;
        if (d == nullptr) {
            std::fprintf(stderr, "device %s not found in %s\n", dump.c_str(), esi_path.c_str());
            return 1;
        }
        std::printf("%s", esiDeviceToYaml(*d).c_str());
        return 0;
    }

    // Bus scan.
    auto backend = createIghBackend();
    if (!backend) {
        std::fprintf(stderr, "%s\n", backend.status().toString().c_str());
        return 2;
    }
    const unsigned master_index = static_cast<unsigned>(std::strtoul(option(argc, argv, "--master").c_str(), nullptr, 0));
    if (Status s = backend.value()->open(master_index); !s) {
        std::fprintf(stderr, "%s\n", s.toString().c_str());
        return 1;
    }
    auto slaves = backend.value()->scanSlaves();
    if (!slaves) {
        std::fprintf(stderr, "%s\n", slaves.status().toString().c_str());
        return 1;
    }
    std::printf("%zu slave(s) on master %u\n", slaves->size(), master_index);
    for (const auto& s : slaves.value()) {
        std::printf("  [%u] alias %u  vendor 0x%08X product 0x%08X rev 0x%08X serial %u  %-6s '%s'\n",
                    s.ring_position, s.alias, s.vendor_id, s.product_code, s.revision, s.serial,
                    alStateName(s.al_state), s.name.c_str());
        if (esi) {
            const EsiDevice* exact = esi->find(s.product_code, s.revision);
            const EsiDevice* any = esi->find(s.product_code);
            if (s.vendor_id != esi->vendor_id || any == nullptr) {
                std::printf("       not described by %s\n", esi_path.c_str());
            } else {
                std::printf("       ESI: %s%s\n", any->name.c_str(),
                            exact != nullptr ? "" : "  (revision differs from the ESI - verify compatibility!)");
            }
        }
        auto supported = backend.value()->sdoUploadBlocking(s.ring_position, cia402_obj::kSupportedDriveModes, 0, 4);
        if (supported) {
            std::printf("       0x6502 supported drive modes = 0x%llX: %s\n",
                        static_cast<unsigned long long>(supported.value()),
                        modes(static_cast<std::uint32_t>(supported.value())).c_str());
        } else {
            std::printf("       0x6502 not readable: %s\n", supported.status().message().c_str());
        }
    }
    backend.value()->close();
    return 0;
}
