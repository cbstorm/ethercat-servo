// SPDX-License-Identifier: MIT
#include "ethercat_motion/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace ethercat_motion {

namespace {

constexpr double kEps = 1e-12;

double signOf(double v) noexcept { return v < 0.0 ? -1.0 : 1.0; }

bool positiveFinite(double v) noexcept { return std::isfinite(v) && v > 0.0; }

}  // namespace

double TrapezoidalProfile::stoppingDistance(double v, double dec) noexcept {
    if (!(dec > 0.0)) {
        return 0.0;
    }
    return v * v / (2.0 * dec);
}

void TrapezoidalProfile::clear(double p0) noexcept {
    count_ = 0;
    duration_ = 0.0;
    start_position_ = p0;
    final_position_ = p0;
    final_velocity_ = 0.0;
}

void TrapezoidalProfile::append(double duration, double a) noexcept {
    if (!(duration > kEps) || count_ >= kMaxSegments) {
        return;
    }
    Segment& s = segments_[count_++];
    s.t0 = duration_;
    s.duration = duration;
    s.p0 = final_position_;
    s.v0 = final_velocity_;
    s.a = a;
    final_position_ += s.v0 * duration + 0.5 * a * duration * duration;
    final_velocity_ += a * duration;
    duration_ += duration;
}

void TrapezoidalProfile::appendRestToTarget(double target, double vmax, double acc, double dec) noexcept {
    const double dist = target - final_position_;
    const double s = signOf(dist);
    double u0 = std::max(0.0, final_velocity_ * s);  // speed toward the target

    if (u0 > vmax) {
        // Faster than allowed: bring the speed down to vmax first.
        append((u0 - vmax) / dec, -s * dec);
        u0 = vmax;
        final_velocity_ = s * vmax;
    }

    const double d = std::fabs(target - final_position_);
    if (d < kEps && u0 < kEps) {
        return;
    }
    const double vp2 = (2.0 * acc * dec * d + dec * u0 * u0) / (acc + dec);
    double vp = std::min(vmax, std::sqrt(std::max(0.0, vp2)));
    vp = std::max(vp, u0);

    const double t_acc = (vp - u0) / acc;
    const double d_acc = (vp * vp - u0 * u0) / (2.0 * acc);
    const double t_dec = vp / dec;
    const double d_dec = vp * vp / (2.0 * dec);
    const double d_cruise = std::max(0.0, d - d_acc - d_dec);
    const double t_cruise = vp > kEps ? d_cruise / vp : 0.0;

    append(t_acc, s * acc);
    append(t_cruise, 0.0);
    append(t_dec, -s * dec);
}

bool TrapezoidalProfile::planPosition(double p0, double v0, double target, double vmax, double acc,
                                      double dec) noexcept {
    if (!positiveFinite(vmax) || !positiveFinite(acc) || !positiveFinite(dec) || !std::isfinite(p0) ||
        !std::isfinite(v0) || !std::isfinite(target)) {
        return false;
    }
    clear(p0);
    kind_ = Kind::Position;
    final_velocity_ = v0;

    const double dist = target - p0;
    if (std::fabs(v0) > kEps) {
        const double s = signOf(dist);
        const double u0 = v0 * s;
        const bool moving_away = u0 < 0.0;
        const bool would_overshoot = u0 > 0.0 && stoppingDistance(u0, dec) > std::fabs(dist) + 1e-9;
        if (moving_away || would_overshoot) {
            // Stop first, then travel back to the target from rest.
            append(std::fabs(v0) / dec, -signOf(v0) * dec);
            final_velocity_ = 0.0;
        }
    }
    appendRestToTarget(target, vmax, acc, dec);
    final_position_ = target;
    final_velocity_ = 0.0;
    return true;
}

bool TrapezoidalProfile::planVelocity(double p0, double v0, double v_target, double acc, double dec) noexcept {
    if (!positiveFinite(acc) || !positiveFinite(dec) || !std::isfinite(p0) || !std::isfinite(v0) ||
        !std::isfinite(v_target)) {
        return false;
    }
    clear(p0);
    kind_ = Kind::Velocity;
    final_velocity_ = v0;

    if (v0 * v_target < 0.0) {
        append(std::fabs(v0) / dec, -signOf(v0) * dec);
        final_velocity_ = 0.0;
        append(std::fabs(v_target) / acc, signOf(v_target) * acc);
    } else if (std::fabs(v_target) > std::fabs(v0)) {
        append((std::fabs(v_target) - std::fabs(v0)) / acc, signOf(v_target - v0) * acc);
    } else {
        append((std::fabs(v0) - std::fabs(v_target)) / dec, signOf(v_target - v0) * dec);
    }
    final_velocity_ = v_target;
    return true;
}

bool TrapezoidalProfile::planStop(double p0, double v0, double dec) noexcept {
    if (!positiveFinite(dec) || !std::isfinite(p0) || !std::isfinite(v0)) {
        return false;
    }
    clear(p0);
    kind_ = Kind::Stop;
    final_velocity_ = v0;
    append(std::fabs(v0) / dec, -signOf(v0) * dec);
    final_velocity_ = 0.0;
    return true;
}

void TrapezoidalProfile::hold(double p0) noexcept {
    clear(p0);
    kind_ = Kind::Idle;
}

TrajectoryPoint TrapezoidalProfile::sample(double t) const noexcept {
    if (count_ > 0 && t <= 0.0) {
        return {segments_[0].p0, segments_[0].v0, segments_[0].a};
    }
    for (std::size_t i = 0; i < count_; ++i) {
        const Segment& s = segments_[i];
        if (t < s.t0 + s.duration) {
            const double tau = t - s.t0;
            return {s.p0 + s.v0 * tau + 0.5 * s.a * tau * tau, s.v0 + s.a * tau, s.a};
        }
    }
    if (kind_ == Kind::Velocity) {
        const double tau = std::max(0.0, t - duration_);
        return {final_position_ + final_velocity_ * tau, final_velocity_, 0.0};
    }
    return {final_position_, 0.0, 0.0};
}

}  // namespace ethercat_motion
