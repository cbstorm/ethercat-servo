// SPDX-License-Identifier: MIT
//
// Single axis: enable, move forward by a distance, move back, disable.
//
//   basic_axis [config.yaml] [--axis axis_x] [--distance 10] [--allow-motion]
//
// With the default simulation configuration no hardware is needed. With a
// hardware configuration (backend: igh) motion requires --allow-motion.
#include <chrono>
#include <thread>

#include "example_common.hpp"

using namespace ethercat_motion;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    example::installSignalHandlers();
    const std::string config_file = example::positional(argc, argv, ETHERCAT_MOTION_CONFIG_DIR "/simulation.yaml");
    const std::string axis_name = example::option(argc, argv, "--axis", "axis_x");
    const double distance = std::stod(example::option(argc, argv, "--distance", "10"));

    Controller controller(config_file);
    if (Status s = controller.initialize(); !s) {
        std::fprintf(stderr, "initialize failed: %s\n", s.toString().c_str());
        return 1;
    }
    if (Status s = controller.start(); !s) {
        std::fprintf(stderr, "start failed: %s\n", s.toString().c_str());
        return 1;
    }

    Axis* axis = controller.motion().findAxis(axis_name);
    if (axis == nullptr) {
        std::fprintf(stderr, "unknown axis '%s'\n", axis_name.c_str());
        return 1;
    }
    const std::string units = axis->config().units;
    std::printf("axis '%s' status after start:\n", axis_name.c_str());
    example::printAxis(axis->status(), units);

    int rc = 0;
    if (example::motionAllowed(controller.config(), argc, argv)) {
        // Wait for the drive to leave Not Ready To Switch On and the mode to be confirmed.
        for (int i = 0; i < 2000 && axis->status().mode_display != axis->status().mode; ++i) std::this_thread::sleep_for(1ms);

        Status s = axis->enable();
        if (s) s = axis->wait(3s);
        if (!s) {
            std::fprintf(stderr, "enable failed: %s\n", s.toString().c_str());
            rc = 1;
        }
        const double start = axis->getActualPosition();
        for (double target : {start + distance, start}) {
            if (rc != 0 || example::g_stop) break;
            std::printf("move to %.3f %s\n", target, units.c_str());
            s = axis->moveAbsolute(target);
            while (s && !example::g_stop) {
                const Status done = axis->wait(100ms);
                example::printAxis(axis->status(), units);
                if (done.code() != ErrorCode::Timeout) {
                    s = done;
                    break;
                }
            }
            if (!s) {
                std::fprintf(stderr, "move failed: %s\n", s.toString().c_str());
                rc = 1;
            }
        }
        if (example::g_stop) (void)axis->stop();
    }

    const MasterStatus m = controller.ethercatStatus();
    std::printf("cycles %llu, overruns %llu, period %.1f..%.1f us, exec max %.1f us\n",
                static_cast<unsigned long long>(m.timing.cycles), static_cast<unsigned long long>(m.timing.overruns),
                static_cast<double>(m.timing.period_min_ns) / 1000.0, static_cast<double>(m.timing.period_max_ns) / 1000.0,
                static_cast<double>(m.timing.exec_max_ns) / 1000.0);
    (void)controller.stop();
    return rc;
}
