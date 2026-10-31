// SPDX-License-Identifier: MIT
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ethercat_motion/pid_controller.hpp"

using namespace ethercat_motion;

namespace {

PidConfig base() {
    PidConfig c;
    c.kp = 2.0;
    c.ki = 0.0;
    c.kd = 0.0;
    c.output_min = -100.0;
    c.output_max = 100.0;
    c.sample_time = 0.001;
    return c;
}

}  // namespace

TEST(Pid, ProportionalOnly) {
    PIDController pid(base());
    pid.setSetpoint(10.0);
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.001), 12.0);
}

TEST(Pid, OutputIsClamped) {
    PIDController pid(base());
    pid.setSetpoint(1000.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 100.0);
    EXPECT_TRUE(pid.saturated());
}

TEST(Pid, IntegralAccumulates) {
    PidConfig c = base();
    c.kp = 0.0;
    c.ki = 10.0;
    PIDController pid(c);
    pid.setSetpoint(1.0);
    double out = 0.0;
    for (int i = 0; i < 100; ++i) out = pid.update(0.0, 0.01);
    // I = ki * e * t = 10 * 1 * 1.0 s
    EXPECT_NEAR(out, 10.0, 1e-9);
}

TEST(Pid, AntiWindupConditionalIntegration) {
    PidConfig c = base();
    c.kp = 1.0;
    c.ki = 100.0;
    c.output_max = 5.0;
    c.output_min = -5.0;
    PIDController pid(c);
    pid.setSetpoint(10.0);
    for (int i = 0; i < 1000; ++i) pid.update(0.0, 0.01);
    EXPECT_LE(pid.integral(), 5.0);
    // After the error reverses the output must leave saturation quickly.
    pid.setSetpoint(-10.0);
    const double out = pid.update(0.0, 0.01);
    EXPECT_LT(out, 0.0);
}

TEST(Pid, BackCalculationAntiWindup) {
    PidConfig c = base();
    c.kp = 1.0;
    c.ki = 50.0;
    c.output_max = 5.0;
    c.output_min = -5.0;
    c.anti_windup_gain = 20.0;
    PIDController pid(c);
    pid.setSetpoint(100.0);
    for (int i = 0; i < 2000; ++i) pid.update(0.0, 0.001);
    EXPECT_LE(pid.integral(), 5.0 + 1e-9);
}

TEST(Pid, DerivativeOnMeasurementHasNoSetpointKick) {
    PidConfig c = base();
    c.kp = 0.0;
    c.kd = 1.0;
    PIDController pid(c);
    pid.setSetpoint(0.0);
    pid.update(0.0, 0.001);
    pid.setSetpoint(50.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 0.0);
    // A moving measurement produces a derivative response.
    EXPECT_LT(pid.update(0.01, 0.001), 0.0);
}

TEST(Pid, DerivativeFilterSmoothsStep) {
    PidConfig c = base();
    c.kp = 0.0;
    c.kd = 1.0;
    c.derivative_filter_time = 0.01;
    PIDController filtered(c);
    c.derivative_filter_time = 0.0;
    PIDController raw(c);
    filtered.update(0.0, 0.001);
    raw.update(0.0, 0.001);
    EXPECT_LT(std::fabs(filtered.update(0.01, 0.001)), std::fabs(raw.update(0.01, 0.001)));
}

TEST(Pid, SetpointRamp) {
    PidConfig c = base();
    c.kp = 1.0;
    c.setpoint_ramp_rate = 10.0;  // units/s
    PIDController pid(c);
    pid.setSetpoint(100.0);
    pid.update(0.0, 0.1);  // initializes ramp at measurement, then ramps 1 unit
    EXPECT_NEAR(pid.rampedSetpoint(), 1.0, 1e-12);
    pid.update(0.0, 0.1);
    EXPECT_NEAR(pid.rampedSetpoint(), 2.0, 1e-12);
}

TEST(Pid, OutputRateLimit) {
    PidConfig c = base();
    c.kp = 100.0;
    c.output_rate_limit = 10.0;  // per second
    PIDController pid(c);
    pid.setSetpoint(1.0);
    EXPECT_NEAR(pid.update(0.0, 0.1), 1.0, 1e-12);
    EXPECT_NEAR(pid.update(0.0, 0.1), 2.0, 1e-12);
}

TEST(Pid, Deadband) {
    PidConfig c = base();
    c.deadband = 0.5;
    PIDController pid(c);
    pid.setSetpoint(0.3);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 0.0);
    pid.setSetpoint(1.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 2.0);
}

TEST(Pid, FeedForward) {
    PIDController pid(base());
    pid.setSetpoint(0.0);
    pid.setFeedForward(7.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 7.0);
}

TEST(Pid, ManualModeAndBumplessTransfer) {
    PidConfig c = base();
    c.kp = 1.0;
    c.ki = 1.0;
    PIDController pid(c);
    pid.setMode(PIDController::Mode::Manual);
    pid.setManualOutput(30.0);
    pid.setSetpoint(5.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.001), 30.0);
    pid.setMode(PIDController::Mode::Automatic);
    // First automatic output continues from the manual value (no bump).
    const double first = pid.update(0.0, 0.001);
    EXPECT_NEAR(first, 30.0, 0.01);
}

TEST(Pid, RetuningIsBumpless) {
    PidConfig c = base();
    c.kp = 0.0;
    c.ki = 10.0;
    PIDController pid(c);
    pid.setSetpoint(1.0);
    for (int i = 0; i < 50; ++i) pid.update(1.0 - 0.0, 0.01);  // zero error: nothing accumulates
    pid.setSetpoint(2.0);
    for (int i = 0; i < 50; ++i) pid.update(1.0, 0.01);
    const double before = pid.output();
    pid.setTunings(0.0, 100.0, 0.0);
    pid.setSetpoint(1.0);  // zero error
    EXPECT_NEAR(pid.update(1.0, 0.01), before, 1e-9);
}

TEST(Pid, DisableAndReset) {
    PidConfig c = base();
    c.ki = 5.0;
    PIDController pid(c);
    pid.setSetpoint(1.0);
    pid.update(0.0, 0.1);
    pid.setEnabled(false);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 0.1), 0.0);
    pid.setEnabled(true);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(Pid, NonFiniteInputHoldsOutput) {
    PIDController pid(base());
    pid.setSetpoint(1.0);
    const double out = pid.update(0.0, 0.001);
    EXPECT_DOUBLE_EQ(pid.update(std::numeric_limits<double>::quiet_NaN(), 0.001), out);
    EXPECT_TRUE(pid.inputFault());
}

TEST(Pid, ConfigValidation) {
    PidConfig c = base();
    EXPECT_TRUE(PIDController::isValid(c));
    c.output_min = 10.0;
    c.output_max = -10.0;
    EXPECT_FALSE(PIDController::isValid(c));
    c = base();
    c.kp = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(PIDController::isValid(c));
    c = base();
    c.sample_time = 0.0;
    EXPECT_FALSE(PIDController::isValid(c));
}

TEST(Pid, ClosedLoopFirstOrderPlantConverges) {
    // Plant: x' = (u - x) / tau. PI controller must settle without overshoot runaway.
    PidConfig c = base();
    c.kp = 2.0;
    c.ki = 5.0;
    PIDController pid(c);
    pid.setSetpoint(10.0);
    double x = 0.0;
    const double dt = 0.001;
    for (int i = 0; i < 10000; ++i) {
        const double u = pid.update(x, dt);
        x += (u - x) / 0.1 * dt;
    }
    EXPECT_NEAR(x, 10.0, 1e-3);
}
