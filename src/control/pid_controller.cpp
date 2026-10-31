// SPDX-License-Identifier: MIT
#include "ethercat_motion/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ethercat_motion {

PIDController::PIDController() noexcept : PIDController(PidConfig{}) {}

PIDController::PIDController(const PidConfig& config) noexcept { setConfig(config); }

bool PIDController::isValid(const PidConfig& c) noexcept {
    const bool finite_gains = std::isfinite(c.kp) && std::isfinite(c.ki) && std::isfinite(c.kd);
    return finite_gains && std::isfinite(c.output_min) && std::isfinite(c.output_max) && c.output_min < c.output_max &&
           c.sample_time > 0.0 && std::isfinite(c.sample_time) && c.derivative_filter_time >= 0.0 &&
           c.setpoint_ramp_rate >= 0.0 && c.output_rate_limit >= 0.0 && c.deadband >= 0.0 &&
           c.anti_windup_gain >= 0.0 && !(c.integral_min > c.integral_max);
}

void PIDController::setConfig(const PidConfig& config) noexcept {
    if (!isValid(config)) {
        return;
    }
    config_ = config;
    integral_ = std::clamp(integral_, integralLow(), integralHigh());
}

double PIDController::integralLow() const noexcept { return std::max(config_.integral_min, config_.output_min); }
double PIDController::integralHigh() const noexcept { return std::min(config_.integral_max, config_.output_max); }

double PIDController::clampOutput(double value) const noexcept {
    return std::clamp(value, config_.output_min, config_.output_max);
}

void PIDController::setTunings(double kp, double ki, double kd) noexcept {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd)) {
        return;
    }
    // Integral holds Ki*e*dt, so a new Ki does not bump the output.
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
}

void PIDController::setOutputLimits(double min, double max) noexcept {
    if (!std::isfinite(min) || !std::isfinite(max) || !(min < max)) {
        return;
    }
    config_.output_min = min;
    config_.output_max = max;
    integral_ = std::clamp(integral_, integralLow(), integralHigh());
    output_ = clampOutput(output_);
}

void PIDController::setMode(Mode mode) noexcept {
    if (mode == mode_) {
        return;
    }
    if (mode == Mode::Automatic) {
        // Bumpless transfer: the next update initializes the integral so the
        // output continues from the last manual value, and the setpoint ramp
        // (if any) restarts from the measurement.
        bumpless_pending_ = true;
        derivative_ = 0.0;
        ramped_setpoint_ = previous_measured_;
    }
    mode_ = mode;
}

void PIDController::setManualOutput(double value) noexcept {
    if (!std::isfinite(value)) {
        return;
    }
    manual_output_ = value;
    if (mode_ == Mode::Manual) {
        output_ = clampOutput(value);
    }
}

void PIDController::setEnabled(bool enabled) noexcept {
    if (enabled == config_.enabled) {
        return;
    }
    config_.enabled = enabled;
    reset();
}

void PIDController::reset() noexcept {
    integral_ = 0.0;
    derivative_ = 0.0;
    previous_error_ = 0.0;
    last_error_ = 0.0;
    output_ = 0.0;
    initialized_ = false;
    saturated_ = false;
    input_fault_ = false;
    bumpless_pending_ = false;
}

double PIDController::update(double measured, double dt) noexcept {
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        dt = config_.sample_time;
    }
    if (!std::isfinite(measured) || !std::isfinite(target_setpoint_) || !std::isfinite(feed_forward_)) {
        input_fault_ = true;
        return output_;  // hold last output
    }
    if (!config_.enabled) {
        output_ = 0.0;
        return output_;
    }

    if (!initialized_) {
        ramped_setpoint_ = config_.setpoint_ramp_rate > 0.0 ? measured : target_setpoint_;
        previous_measured_ = measured;
        previous_error_ = ramped_setpoint_ - measured;
        derivative_ = 0.0;
        initialized_ = true;
    }

    // Setpoint ramp.
    if (config_.setpoint_ramp_rate > 0.0) {
        const double step = config_.setpoint_ramp_rate * dt;
        ramped_setpoint_ += std::clamp(target_setpoint_ - ramped_setpoint_, -step, step);
    } else {
        ramped_setpoint_ = target_setpoint_;
    }

    double error = ramped_setpoint_ - measured;
    if (std::fabs(error) < config_.deadband) {
        error = 0.0;
    }
    last_error_ = error;

    if (mode_ == Mode::Manual) {
        output_ = clampOutput(manual_output_);
        previous_measured_ = measured;
        previous_error_ = error;
        return output_;
    }

    // Derivative (on measurement avoids derivative kick on setpoint steps).
    const double raw_derivative = config_.derivative_on_measurement ? -(measured - previous_measured_) / dt
                                                                    : (error - previous_error_) / dt;
    if (config_.derivative_filter_time > 0.0) {
        const double alpha = dt / (config_.derivative_filter_time + dt);
        derivative_ += alpha * (raw_derivative - derivative_);
    } else {
        derivative_ = raw_derivative;
    }

    const double p_term = config_.kp * error;
    const double d_term = config_.kd * derivative_;
    const double ff = feed_forward_;

    if (bumpless_pending_) {
        integral_ = std::clamp(output_ - p_term - d_term - ff, integralLow(), integralHigh());
        bumpless_pending_ = false;
    }

    // Integral with anti-windup.
    const double i_candidate = std::clamp(integral_ + config_.ki * error * dt, integralLow(), integralHigh());
    const double unsat_candidate = ff + p_term + i_candidate + d_term;
    if (config_.anti_windup_gain > 0.0) {
        const double sat = clampOutput(unsat_candidate);
        integral_ = std::clamp(i_candidate + config_.anti_windup_gain * (sat - unsat_candidate) * dt, integralLow(),
                               integralHigh());
    } else {
        const double ki_e = config_.ki * error;
        const bool winding_up = (unsat_candidate > config_.output_max && ki_e > 0.0) ||
                                (unsat_candidate < config_.output_min && ki_e < 0.0);
        if (!winding_up) {
            integral_ = i_candidate;
        }
    }

    const double unsat = ff + p_term + integral_ + d_term;
    double u = clampOutput(unsat);
    saturated_ = (u != unsat);

    if (config_.output_rate_limit > 0.0) {
        const double max_step = config_.output_rate_limit * dt;
        u = std::clamp(u, output_ - max_step, output_ + max_step);
    }

    previous_measured_ = measured;
    previous_error_ = error;
    output_ = u;
    return u;
}

}  // namespace ethercat_motion
