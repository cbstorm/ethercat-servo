// SPDX-License-Identifier: MIT
//
// Simulation walkthrough (no hardware): CSP move, CSV axis with an external
// PID position loop, PP mode, homing, an injected drive fault with recovery
// and a simulated cable break. Uses the real cyclic thread.
//
//   simulation [config.yaml]
#include <chrono>
#include <thread>

#include "example_common.hpp"

using namespace ethercat_motion;
using namespace std::chrono_literals;

namespace {

bool check(const Status& s, const char* what) {
    if (!s) std::fprintf(stderr, "%s: %s\n", what, s.toString().c_str());
    return s.isOk();
}

void show(Controller& c) {
    for (const auto& n : c.motion().axisNames()) example::printAxis(c.axis(n).status(), c.axis(n).config().units);
}

void waitReady(Controller& c) {
    for (int i = 0; i < 2000; ++i) {
        bool ready = true;
        for (const auto& n : c.motion().axisNames()) {
            const AxisStatus s = c.axis(n).status();
            ready = ready && s.comm_ok && s.drive_state == DriveState::SwitchOnDisabled && s.mode_display == s.mode;
        }
        if (ready) return;
        std::this_thread::sleep_for(1ms);
    }
}

}  // namespace

int main(int argc, char** argv) {
    auto loaded = loadConfigFile(example::positional(argc, argv, ETHERCAT_MOTION_CONFIG_DIR "/simulation.yaml"));
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.status().toString().c_str());
        return 1;
    }
    Config config = loaded.value();
    if (config.ethercat.backend != BackendType::Simulation) {
        std::fprintf(stderr, "this example only runs with ethercat.backend: simulation\n");
        return 1;
    }
    config.logging.level = "warn";
    Controller c(config);
    if (!check(c.initialize(), "initialize") || !check(c.start(), "start")) return 1;
    waitReady(c);

    Axis& x = c.axis("axis_x");
    Axis& y = c.axis("axis_y");

    std::printf("\n== 1. enable both axes (CiA 402: 0x06 -> 0x07 -> 0x0F)\n");
    if (!check(c.motion().enableAll(), "enable") || !check(x.wait(2s), "enable x") || !check(y.wait(2s), "enable y")) return 1;
    show(c);

    std::printf("\n== 2. CSP move axis_x to 25 mm, CSV+PID move axis_y to 90 deg\n");
    check(x.moveAbsolute(25.0), "move x");
    check(y.moveAbsolute(90.0), "move y");
    check(x.wait(5s), "wait x");
    check(y.wait(5s), "wait y");
    show(c);

    std::printf("\n== 3. switch axis_x to Profile Position (PP) and move to -10 mm (drive-generated profile)\n");
    check(x.setMode(OperationMode::PP), "set mode");
    check(x.wait(2s), "mode switch");
    check(x.moveAbsolute(-10.0), "PP move");
    check(x.wait(5s), "PP wait");
    check(x.setMode(OperationMode::CSP), "back to CSP");
    check(x.wait(2s), "mode switch");
    show(c);

    std::printf("\n== 4. homing axis_x (drive homing method %d)\n", x.config().homing.method);
    check(x.home(), "home");
    check(x.wait(20s), "homing");
    show(c);

    std::printf("\n== 5. inject drive fault 0x2310 on axis_x -> axis_y is quick-stopped (stop_all_on_axis_fault)\n");
    c.simulation()->injectDriveFault(0, 0x2310);
    std::this_thread::sleep_for(100ms);
    show(c);
    char faults[256];
    describeSafetyFaults(c.safetyStatus().latched_faults, faults, sizeof(faults));
    std::printf("  latched safety faults: %s\n", faults);

    std::printf("\n== 6. reset faults and re-enable\n");
    check(c.resetFaults(), "reset");
    std::this_thread::sleep_for(100ms);
    check(c.motion().enableAll(), "enable");
    check(x.wait(2s), "enable x");
    check(y.wait(2s), "enable y");
    show(c);

    std::printf("\n== 7. cable break at axis_y (slave 1 offline) for 200 ms\n");
    c.simulation()->setSlaveOnline(1, false);
    std::this_thread::sleep_for(200ms);
    c.simulation()->setSlaveOnline(1, true);
    std::this_thread::sleep_for(200ms);
    show(c);
    describeSafetyFaults(c.safetyStatus().latched_faults, faults, sizeof(faults));
    std::printf("  latched safety faults: %s\n", faults);
    check(c.resetFaults(), "reset");
    std::this_thread::sleep_for(100ms);

    const MasterStatus m = c.ethercatStatus();
    std::printf("\ncycles %llu, overruns %llu, period %.1f..%.1f us, jitter max %.1f us, exec avg %.1f us\n",
                static_cast<unsigned long long>(m.timing.cycles), static_cast<unsigned long long>(m.timing.overruns),
                static_cast<double>(m.timing.period_min_ns) / 1000.0, static_cast<double>(m.timing.period_max_ns) / 1000.0,
                static_cast<double>(m.timing.jitter_max_ns) / 1000.0, m.timing.exec_avg_ns / 1000.0);
    check(c.stop(), "stop");
    return 0;
}
