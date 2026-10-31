// SPDX-License-Identifier: MIT
//
// ethercat_motion_diagnostic - configuration check and live, read-only bus /
// drive / timing diagnostics. Never enables a drive or commands motion.
//
//   ethercat_motion_diagnostic CONFIG --check          validate config, print effective PDO mapping
//   ethercat_motion_diagnostic CONFIG [--seconds 10]   run the cyclic task and print status
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <ethercat_motion/ethercat_motion.hpp>

using namespace ethercat_motion;
using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};

void printConfig(const Config& c) {
    std::printf("backend %s, master %u, cycle %u us, DC %s, realtime prio %d cpu %d\n",
                c.ethercat.backend == BackendType::Igh ? "igh" : "simulation", c.ethercat.master.master_index,
                c.ethercat.master.cycle_time_us, c.ethercat.master.distributed_clocks ? "on" : "off",
                c.ethercat.master.realtime.priority, c.ethercat.master.realtime.cpu_affinity);
    for (const auto& w : c.warnings) std::printf("WARNING: %s\n", w.c_str());
    for (const auto& a : c.axes) {
        std::printf("\naxis '%s': alias %u position %u, vendor 0x%08X product 0x%08X rev %s, mode %s, loop %s\n",
                    a.name.c_str(), a.alias, a.position, a.vendor_id, a.product_code,
                    a.revision ? std::to_string(*a.revision).c_str() : "any", toString(a.operation_mode),
                    toString(a.control_loop));
        std::printf("  scaling: %.6g counts/%s, limits [%g, %g] %s, vmax %g, amax %g, PDO source '%s'\n",
                    a.counts_per_revolution * a.gear_ratio / a.units_per_revolution, a.units.c_str(),
                    a.limits.position_min, a.limits.position_max, a.units.c_str(), a.limits.velocity_max,
                    a.limits.acceleration_max, a.pdo_source.c_str());
        for (const auto& sm : a.syncs) {
            std::printf("  SM%u (%s)\n", sm.index, sm.direction == PdoDirection::Output ? "outputs/RxPDO" : "inputs/TxPDO");
            for (const auto& pdo : sm.pdos) {
                std::printf("    PDO 0x%04X\n", pdo.index);
                for (const auto& e : pdo.entries) {
                    std::printf("      0x%04X:%02X %2u bit  %s\n", e.index, e.subindex, e.bit_length, e.name.c_str());
                }
            }
        }
        for (const auto& s : a.sdo_init) {
            std::printf("  startup SDO 0x%04X:%02X = %lld (%u byte) %s\n", s.index, s.subindex,
                        static_cast<long long>(s.value), s.size, s.comment.c_str());
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argv[1][0] == '-') {
        std::printf("usage: ethercat_motion_diagnostic CONFIG [--check] [--seconds N]\n");
        return argc < 2 ? 1 : 0;
    }
    std::signal(SIGINT, [](int) { g_stop = true; });
    auto loaded = loadConfigFile(argv[1]);
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.status().toString().c_str());
        return 1;
    }
    int seconds = 10;
    bool check_only = false;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--check") == 0) check_only = true;
        if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) seconds = std::atoi(argv[++i]);
    }
    printConfig(loaded.value());
    if (check_only) {
        std::printf("\nconfiguration OK\n");
        return 0;
    }

    Config config = loaded.value();
    config.mqtt.enabled = false;  // diagnostics only
    Controller controller(config);
    if (Status s = controller.initialize(); !s) {
        std::fprintf(stderr, "initialize failed: %s\n", s.toString().c_str());
        return 1;
    }
    if (Status s = controller.start(); !s) {
        std::fprintf(stderr, "start failed: %s\n", s.toString().c_str());
        return 1;
    }
    std::printf("\nPREEMPT_RT kernel: %s\n", kernelIsPreemptRt() ? "yes" : "no");
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (!g_stop && std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(500ms);
        const MasterStatus m = controller.ethercatStatus();
        const SafetyStatus s = controller.safetyStatus();
        char faults[256];
        describeSafetyFaults(s.latched_faults, faults, sizeof(faults));
        std::printf("\nlink %s, WC %u/%u (%s), slaves responding %u, all OP %s, SCHED_FIFO %s, mlock %s\n",
                    m.comm.link_up ? "up" : "DOWN", m.comm.working_counter, m.comm.expected_working_counter,
                    m.comm.wc_complete ? "complete" : "INCOMPLETE", m.comm.slaves_responding,
                    m.comm.all_operational ? "yes" : "NO", m.realtime_scheduling ? "yes" : "no",
                    m.memory_locked ? "yes" : "no");
        std::printf("timing: cycles %llu overruns %llu period %.1f..%.1f us jitter max %.1f us latency max %.1f us "
                    "exec avg %.1f max %.1f us\n",
                    static_cast<unsigned long long>(m.timing.cycles), static_cast<unsigned long long>(m.timing.overruns),
                    static_cast<double>(m.timing.period_min_ns) / 1000.0,
                    static_cast<double>(m.timing.period_max_ns) / 1000.0,
                    static_cast<double>(m.timing.jitter_max_ns) / 1000.0,
                    static_cast<double>(m.timing.latency_max_ns) / 1000.0, m.timing.exec_avg_ns / 1000.0,
                    static_cast<double>(m.timing.exec_max_ns) / 1000.0);
        std::printf("safety: latched [%s]%s\n", faults, s.emergency_stop_active ? " E-STOP" : "");
        for (const auto& name : controller.motion().axisNames()) {
            const AxisStatus a = controller.axis(name).status();
            std::printf("  %-8s comm %-3s %-19s mode %-6s sw 0x%04X err 0x%04X pos %.4f %s vel %.4f\n", name.c_str(),
                        a.comm_ok ? "ok" : "BAD", toString(a.drive_state), toString(a.mode_display), a.statusword,
                        a.error_code, a.position, controller.axis(name).config().units.c_str(), a.velocity);
        }
    }
    (void)controller.stop();
    return 0;
}
