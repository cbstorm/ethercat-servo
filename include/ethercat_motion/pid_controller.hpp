// SPDX-License-Identifier: MIT
//
// Industrial PID controller.
//
//   u = FF + Kp*e + I + D,   I += Ki*e*dt (conditional integration)
//
// Features: output limits, integral clamping + conditional integration and
// optional back-calculation anti-windup, first-order filtered derivative on
// measurement (no derivative kick), setpoint ramping, output rate limiting,
// manual/automatic mode with bumpless transfer, enable/disable, feed-forward,
// deadband and runtime re-tuning.
//
// Deterministic and allocation free; update() is safe inside the cyclic task.
// Because the integral state stores Ki*e*dt (not e*dt), changing Ki at
// runtime does not produce an output bump.
#pragma once

#include <cstdint>
#include <limits>

namespace ethercat_motion {

struct PidConfig {
    double kp = 0.0;
    double ki = 0.0;                    ///< 1/s
    double kd = 0.0;                    ///< s
    double sample_time = 0.001;         ///< s, used when update() gets dt <= 0
    double output_min = -1.0;
    double output_max = 1.0;
    double integral_min = -std::numeric_limits<double>::infinity();  ///< Clamped to output limits too.
    double integral_max = std::numeric_limits<double>::infinity();
    double anti_windup_gain = 0.0;      ///< Back-calculation gain Kt (0 = conditional integration only).
    double derivative_filter_time = 0.0;///< First order filter time constant (s); 0 = unfiltered.
    bool derivative_on_measurement = true;
    double setpoint_ramp_rate = 0.0;    ///< units/s; 0 = step changes allowed.
    double output_rate_limit = 0.0;     ///< output units/s; 0 = unlimited.
    double deadband = 0.0;              ///< |error| below this is treated as 0.
    bool enabled = true;
};

class PIDController {
public:
    enum class Mode : std::uint8_t { Automatic, Manual };

    PIDController() noexcept;
    explicit PIDController(const PidConfig& config) noexcept;

    /// Validates the configuration (limits ordered, gains finite, ...).
    static bool isValid(const PidConfig& config) noexcept;

    void setSetpoint(double setpoint) noexcept { target_setpoint_ = setpoint; }
    double setpoint() const noexcept { return target_setpoint_; }
    /// Effective (ramped) setpoint used in the last update.
    double rampedSetpoint() const noexcept { return ramped_setpoint_; }
    void setFeedForward(double value) noexcept { feed_forward_ = value; }

    /// Computes a new output. `dt` <= 0 uses the configured sample time.
    double update(double measured, double dt = 0.0) noexcept;

    void setTunings(double kp, double ki, double kd) noexcept;
    void setOutputLimits(double min, double max) noexcept;
    void setConfig(const PidConfig& config) noexcept;
    const PidConfig& config() const noexcept { return config_; }

    void setMode(Mode mode) noexcept;
    Mode mode() const noexcept { return mode_; }
    /// Output used in manual mode.
    void setManualOutput(double value) noexcept;

    void setEnabled(bool enabled) noexcept;
    bool enabled() const noexcept { return config_.enabled; }

    /// Clears integral, derivative filter, ramp and rate-limiter state.
    void reset() noexcept;
    /// Clears only the integral term.
    void resetIntegral() noexcept { integral_ = 0.0; }

    double output() const noexcept { return output_; }
    double integral() const noexcept { return integral_; }
    double lastError() const noexcept { return last_error_; }
    bool saturated() const noexcept { return saturated_; }
    /// Set when a non-finite input was seen; the output was held.
    bool inputFault() const noexcept { return input_fault_; }

private:
    double clampOutput(double value) const noexcept;
    double integralLow() const noexcept;
    double integralHigh() const noexcept;

    PidConfig config_;
    Mode mode_ = Mode::Automatic;
    double target_setpoint_ = 0.0;
    double ramped_setpoint_ = 0.0;
    double feed_forward_ = 0.0;
    double manual_output_ = 0.0;

    double integral_ = 0.0;
    double derivative_ = 0.0;
    double previous_measured_ = 0.0;
    double previous_error_ = 0.0;
    double last_error_ = 0.0;
    double output_ = 0.0;
    bool initialized_ = false;
    bool saturated_ = false;
    bool input_fault_ = false;
    bool bumpless_pending_ = false;
};

}  // namespace ethercat_motion
