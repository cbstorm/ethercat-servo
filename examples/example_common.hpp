// SPDX-License-Identifier: MIT
// Small helpers shared by the examples (argument parsing, Ctrl-C, printing).
#pragma once

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>

#include <ethercat_motion/ethercat_motion.hpp>

namespace example {

inline std::atomic<bool> g_stop{false};

inline void installSignalHandlers() {
    std::signal(SIGINT, [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
}

inline bool hasFlag(int argc, char** argv, const char* flag) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], flag) == 0) return true;
    }
    return false;
}

inline std::string option(int argc, char** argv, const char* name, const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
    }
    return fallback;
}

/// First argument that is not an option (or an option value).
inline std::string positional(int argc, char** argv, const std::string& fallback) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-') {
            if (i + 1 < argc && argv[i + 1][0] != '-' && std::strcmp(argv[i], "--allow-motion") != 0) ++i;
            continue;
        }
        return argv[i];
    }
    return fallback;
}

inline void printAxis(const ethercat_motion::AxisStatus& s, const std::string& units) {
    std::printf("  %-8s %-19s %-5s pos %10.4f %-3s vel %9.4f  sp %10.4f  sw 0x%04X err 0x%04X%s%s\n", s.name.c_str(),
                ethercat_motion::toString(s.drive_state), ethercat_motion::toString(s.mode_display), s.position,
                units.c_str(), s.velocity, s.setpoint_position, s.statusword, s.error_code, s.moving ? " MOVING" : "",
                s.safety_faults != 0 ? " SAFETY" : "");
}

/// Refuses motion on real hardware unless explicitly allowed.
inline bool motionAllowed(const ethercat_motion::Config& config, int argc, char** argv) {
    if (config.ethercat.backend == ethercat_motion::BackendType::Simulation) return true;
    if (hasFlag(argc, argv, "--allow-motion")) {
        std::printf("WARNING: commanding motion on REAL hardware. Keep the hardware E-stop within reach.\n");
        return true;
    }
    std::printf("Hardware backend: motion commands are disabled. Re-run with --allow-motion after completing the\n"
                "commissioning checklist in docs/hardware_setup.md.\n");
    return false;
}

}  // namespace example
