// SPDX-License-Identifier: MIT
//
// Multi-axis: synchronized point-to-point moves through a list of joint-space
// waypoints using MultiAxisCoordinator (all axes start and finish together).
//
//   multi_axis [config.yaml] [--allow-motion]
#include <chrono>
#include <thread>
#include <vector>

#include "example_common.hpp"

using namespace ethercat_motion;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    example::installSignalHandlers();
    Controller controller(example::positional(argc, argv, ETHERCAT_MOTION_CONFIG_DIR "/simulation.yaml"));
    if (Status s = controller.initialize(); !s) {
        std::fprintf(stderr, "initialize failed: %s\n", s.toString().c_str());
        return 1;
    }
    if (Status s = controller.start(); !s) {
        std::fprintf(stderr, "start failed: %s\n", s.toString().c_str());
        return 1;
    }
    const auto names = controller.motion().axisNames();
    if (names.size() < 2 || !example::motionAllowed(controller.config(), argc, argv)) {
        if (names.size() < 2) std::printf("this example needs at least two axes\n");
        (void)controller.stop();
        return names.size() < 2 ? 1 : 0;
    }

    // Enable all axes and wait.
    for (int i = 0; i < 2000; ++i) {
        bool ready = true;
        for (const auto& n : names) ready = ready && controller.axis(n).status().mode_display == controller.axis(n).status().mode;
        if (ready) break;
        std::this_thread::sleep_for(1ms);
    }
    if (Status s = controller.motion().enableAll(); !s) {
        std::fprintf(stderr, "enable failed: %s\n", s.toString().c_str());
        return 1;
    }
    for (const auto& n : names) {
        if (Status s = controller.axis(n).wait(3s); !s) {
            std::fprintf(stderr, "enable %s failed: %s\n", n.c_str(), s.toString().c_str());
            (void)controller.stop();
            return 1;
        }
    }

    // Waypoints as a fraction of each axis' soft-limit range.
    const std::vector<std::pair<double, double>> waypoints = {{0.2, 0.0}, {0.2, 0.2}, {-0.2, 0.2}, {-0.2, -0.2}, {0.0, 0.0}};
    MultiAxisCoordinator coordinator(controller.motion());
    int rc = 0;
    for (const auto& [fx, fy] : waypoints) {
        if (example::g_stop) break;
        const auto& lx = controller.axis(names[0]).config().limits;
        const auto& ly = controller.axis(names[1]).config().limits;
        const std::vector<AxisTarget> targets = {{names[0], fx * (lx.position_max - lx.position_min) / 2.0},
                                                 {names[1], fy * (ly.position_max - ly.position_min) / 2.0}};
        auto plan = coordinator.moveSynchronized(targets);
        if (!plan) {
            std::fprintf(stderr, "move failed: %s\n", plan.status().toString().c_str());
            rc = 1;
            break;
        }
        std::printf("synchronized move -> (%.2f, %.2f), planned %.3f s\n", targets[0].position, targets[1].position,
                    plan->duration_s);
        Status s = coordinator.wait(std::chrono::milliseconds(static_cast<int>(plan->duration_s * 1000.0) + 3000));
        for (const auto& n : names) {
            example::printAxis(controller.axis(n).status(), controller.axis(n).config().units);
        }
        if (!s) {
            std::fprintf(stderr, "move did not complete: %s\n", s.toString().c_str());
            rc = 1;
            break;
        }
    }
    (void)controller.stop();
    return rc;
}
