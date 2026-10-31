// SPDX-License-Identifier: MIT
// Shared helpers for tests that drive the controller in simulation mode
// deterministically via Controller::step().
#pragma once

#include <functional>
#include <string>

#include <gtest/gtest.h>

#include "ethercat_motion/ethercat_motion.hpp"

namespace ethercat_motion::test {

inline std::string sourcePath(const std::string& relative) {
    return std::string(ETHERCAT_MOTION_SOURCE_DIR) + "/" + relative;
}

inline Config simulationConfig() {
    auto config = loadConfigFile(sourcePath("config/simulation.yaml"));
    if (!config.isOk()) {
        ADD_FAILURE() << config.status().toString();
        return Config{};
    }
    Config c = config.value();
    c.logging.level = "error";
    c.ethercat.master.realtime.enabled = false;
    return c;
}

/// Steps until `predicate` holds; returns false after `max_cycles`.
inline bool stepUntil(Controller& controller, const std::function<bool()>& predicate, int max_cycles = 5000) {
    for (int i = 0; i < max_cycles; ++i) {
        if (predicate()) return true;
        if (!controller.step(1)) return false;
    }
    return predicate();
}

inline CommandState motionState(Controller& c, const std::string& axis) {
    return c.axis(axis).status().motion_command.state;
}

/// Brings the simulated bus to OP and the drives to a known, disabled state.
inline void bringUp(Controller& controller) {
    ASSERT_TRUE(controller.initialize().isOk());
    ASSERT_TRUE(stepUntil(controller, [&] {
        bool ready = controller.ethercatStatus().comm.all_operational;
        for (const auto& name : controller.motion().axisNames()) {
            const AxisStatus s = controller.axis(name).status();
            ready = ready && s.comm_ok && s.drive_state == DriveState::SwitchOnDisabled &&
                    s.mode_display == s.mode && s.motion_state == MotionState::Idle;
        }
        return ready;
    }));
}

inline void enableAxis(Controller& controller, const std::string& axis) {
    ASSERT_TRUE(controller.axis(axis).enable().isOk());
    ASSERT_TRUE(stepUntil(controller, [&] { return controller.axis(axis).status().enabled; }, 200));
    EXPECT_TRUE(controller.step(2).isOk());
    EXPECT_EQ(controller.axis(axis).status().power_command.state, CommandState::Completed);
}

}  // namespace ethercat_motion::test
