// SPDX-License-Identifier: MIT
//
// Multi-axis coordinator: synchronized point-to-point moves.
//
// Every axis runs a trapezoidal profile that is a scaled copy of a common
// normalized profile, so all axes start in the same cycle, finish at the
// same time and move along a straight line in joint space. The common
// velocity / acceleration / deceleration are chosen so that no axis exceeds
// its own configured defaults (and therefore its limits).
//
// This is joint-space coordination for point-to-point moves; it is not a
// CNC path interpolator (no blending, no circular interpolation).
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "ethercat_motion/motion_controller.hpp"

namespace ethercat_motion {

struct AxisTarget {
    std::string axis;
    double position = 0.0;  ///< Absolute target in the axis' units.
};

struct SynchronizedMovePlan {
    struct Entry {
        std::string axis;
        double start = 0.0;
        double target = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;
        double deceleration = 0.0;
    };
    std::vector<Entry> entries;
    double duration_s = 0.0;   ///< Planned duration of the common profile.
};

class MultiAxisCoordinator {
public:
    explicit MultiAxisCoordinator(MotionController& motion) : motion_(motion) {}

    /// Computes the per-axis profile parameters without moving.
    /// `speed_scale` in (0, 1] scales the common velocity.
    Result<SynchronizedMovePlan> plan(const std::vector<AxisTarget>& targets, double speed_scale = 1.0) const;

    /// Plans and starts a synchronized move (all axes in the same cycle).
    Result<SynchronizedMovePlan> moveSynchronized(const std::vector<AxisTarget>& targets, double speed_scale = 1.0);

    /// Waits until every command of the last synchronized move finished.
    Status wait(std::chrono::milliseconds timeout) const;

private:
    MotionController& motion_;
    std::vector<std::pair<std::string, std::uint64_t>> last_ids_;
};

}  // namespace ethercat_motion
