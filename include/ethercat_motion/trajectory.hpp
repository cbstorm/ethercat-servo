// SPDX-License-Identifier: MIT
//
// Trapezoidal velocity profile generator.
//
// Profiles are planned analytically (closed form) from the current set-point
// state (position, velocity) and sampled at arbitrary times, so they can be
// re-planned at any instant (e.g. new target while moving, stop request)
// without discontinuity in position or velocity. Planning and sampling are
// allocation free and therefore usable inside the cyclic task, but the
// generator itself knows nothing about EtherCAT.
#pragma once

#include <array>
#include <cstdint>

namespace ethercat_motion {

struct TrajectoryPoint {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class TrapezoidalProfile {
public:
    enum class Kind : std::uint8_t { Idle, Position, Velocity, Stop };

    /// Point-to-point move ending at rest at `target`. Handles a non-zero
    /// initial velocity, including moving away from the target or a speed
    /// above `vmax` (decelerates first). Returns false on invalid limits.
    bool planPosition(double p0, double v0, double target, double vmax, double acc, double dec) noexcept;

    /// Ramps from v0 to `v_target` and then keeps that velocity forever.
    bool planVelocity(double p0, double v0, double v_target, double acc, double dec) noexcept;

    /// Decelerates from v0 to rest.
    bool planStop(double p0, double v0, double dec) noexcept;

    /// Holds position p0 at rest.
    void hold(double p0) noexcept;

    TrajectoryPoint sample(double t) const noexcept;
    /// Total duration; for velocity profiles the ramp duration.
    double duration() const noexcept { return duration_; }
    bool finished(double t) const noexcept { return kind_ != Kind::Velocity && t >= duration_; }
    Kind kind() const noexcept { return kind_; }
    /// Final position (position/stop profiles).
    double finalPosition() const noexcept { return final_position_; }
    double finalVelocity() const noexcept { return final_velocity_; }

    /// Distance needed to stop from velocity v with deceleration dec.
    static double stoppingDistance(double v, double dec) noexcept;

private:
    struct Segment {
        double t0 = 0.0;
        double duration = 0.0;
        double p0 = 0.0;
        double v0 = 0.0;
        double a = 0.0;
    };

    void clear(double p0) noexcept;
    void append(double duration, double a) noexcept;
    void appendRestToTarget(double target, double vmax, double acc, double dec) noexcept;

    static constexpr std::size_t kMaxSegments = 6;
    std::array<Segment, kMaxSegments> segments_{};
    std::size_t count_ = 0;
    double duration_ = 0.0;
    double start_position_ = 0.0;
    double final_position_ = 0.0;
    double final_velocity_ = 0.0;
    Kind kind_ = Kind::Idle;
};

}  // namespace ethercat_motion
