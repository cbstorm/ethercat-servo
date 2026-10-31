// SPDX-License-Identifier: MIT
#include "ethercat_motion/coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "ethercat_motion/trajectory.hpp"

namespace ethercat_motion {

Result<SynchronizedMovePlan> MultiAxisCoordinator::plan(const std::vector<AxisTarget>& targets,
                                                        double speed_scale) const {
    if (targets.empty()) return Status(ErrorCode::InvalidArgument, "no targets");
    if (!(speed_scale > 0.0) || speed_scale > 1.0) {
        return Status(ErrorCode::InvalidArgument, "speed_scale must be in (0, 1]");
    }
    SynchronizedMovePlan out;
    std::set<std::string> seen;
    double longest = 0.0;
    for (const AxisTarget& t : targets) {
        Axis* axis = motion_.findAxis(t.axis);
        if (axis == nullptr) return Status(ErrorCode::NotFound, "unknown axis '" + t.axis + "'");
        if (!seen.insert(t.axis).second) return Status(ErrorCode::InvalidArgument, "axis '" + t.axis + "' listed twice");
        if (!std::isfinite(t.position)) return Status(ErrorCode::InvalidArgument, "non-finite target");
        const AxisStatus s = axis->status();
        if (s.moving) return Status(ErrorCode::InvalidState, "axis '" + t.axis + "' is moving");
        const AxisLimits& lim = axis->config().limits;
        if (lim.soft_limits_enabled && (t.position < lim.position_min || t.position > lim.position_max)) {
            return Status(ErrorCode::LimitViolation, "axis '" + t.axis + "': target outside soft limits");
        }
        SynchronizedMovePlan::Entry e;
        e.axis = t.axis;
        e.start = s.setpoint_position;
        e.target = t.position;
        out.entries.push_back(e);
        longest = std::max(longest, std::fabs(e.target - e.start));
    }
    if (longest <= 0.0) return out;  // nothing to move

    // Common normalized profile for the longest distance L: each axis i uses
    // v_i = v * d_i / L (same for a, d). Choose v, a, d so every axis stays
    // within its configured defaults: v <= v_i,max * L / d_i.
    double v = std::numeric_limits<double>::infinity();
    double a = v;
    double d = v;
    for (const auto& e : out.entries) {
        const double dist = std::fabs(e.target - e.start);
        if (dist <= 0.0) continue;
        const AxisConfig& cfg = motion_.findAxis(e.axis)->config();
        const double r = dist / longest;
        v = std::min(v, cfg.motion.velocity * speed_scale / r);
        a = std::min(a, cfg.motion.acceleration / r);
        d = std::min(d, cfg.motion.deceleration / r);
    }
    TrapezoidalProfile common;
    common.planPosition(0.0, 0.0, longest, v, a, d);
    out.duration_s = common.duration();
    for (auto& e : out.entries) {
        const double r = std::fabs(e.target - e.start) / longest;
        e.velocity = v * r;
        e.acceleration = a * r;
        e.deceleration = d * r;
    }
    return out;
}

Result<SynchronizedMovePlan> MultiAxisCoordinator::moveSynchronized(const std::vector<AxisTarget>& targets,
                                                                    double speed_scale) {
    auto planned = plan(targets, speed_scale);
    if (!planned) return planned;
    std::vector<AxisCommand> commands;
    std::vector<std::string> names;
    for (const auto& e : planned->entries) {
        if (e.velocity <= 0.0) continue;  // axis already at its target
        AxisCommand c;
        c.type = AxisCommandType::MoveAbsolute;
        c.axis = static_cast<std::uint16_t>(motion_.findAxis(e.axis)->index());
        c.value = e.target;
        c.velocity = e.velocity;
        c.acceleration = e.acceleration;
        c.deceleration = e.deceleration;
        commands.push_back(c);
        names.push_back(e.axis);
    }
    auto ids = motion_.submitBatch(commands);
    if (!ids) return ids.status();
    last_ids_.clear();
    for (std::size_t i = 0; i < names.size(); ++i) last_ids_.emplace_back(names[i], ids.value()[i]);
    return planned;
}

Status MultiAxisCoordinator::wait(std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (const auto& [name, id] : last_ids_) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        Status s = motion_.getAxis(name).waitForCommand(id, CommandSlot::Motion, std::max(left, std::chrono::milliseconds(0)));
        if (!s) return s;
    }
    return Status::ok();
}

}  // namespace ethercat_motion
