# PID control and control architecture

## Who owns which loop

A servo drive already closes current, velocity and usually position loops at tens of kHz. An external PID at 1 kHz on top of an identical internal loop adds delay and can destabilize the system. The library therefore makes the architecture explicit with `control_loop`, and validates it:

| Drive mode | `control_loop` | Drive closes | Library does | PID output |
|---|---|---|---|---|
| CSP | `none` (only option) | position, velocity, current | Trajectory → target position each cycle | – |
| CSV | `none` | velocity, current | Velocity ramp → target velocity (jog only; position moves are rejected) | – |
| CSV | `position` | velocity, current | Trajectory; PID on position error | velocity correction added to the trajectory velocity (feed-forward) |
| CST | `none` | current | `setTorque()` commands | – |
| CST | `velocity` | current | Velocity ramp; PID on velocity error | torque [% rated] |
| CST | `position` | current | Trajectory; PID on position error | torque [% rated] |
| PP / PV / Homing | (inactive) | everything incl. profile | Handshakes only | – |

Configuration validation rejects:

- a `position` loop without CSV or CST among the axis' modes;
- a `velocity` loop without CST.

The loop is only active while the axis is in a compatible mode.

> The ISL-EC ESI does not declare CST in 0x6502, so CST loops are probably unavailable on that drive (see hardware_setup.md).

## PID features (`PIDController`)

```
e = ramp(setpoint) - measured                (deadband -> 0)
u = FF + Kp*e + I + Kd*D
I += Ki*e*dt                                 (conditional integration or back-calculation)
D = lowpass(-(d measured / dt))              (derivative on measurement: no kick)
u = clamp(u, out_min, out_max), rate-limited
```

| Feature | Config |
|---|---|
| Gains | `kp`, `ki` [1/s], `kd` [s]; `setTunings()` at runtime (MQTT `set_pid`). Bumpless, because the integral stores Ki·e·dt. |
| Output limits | `output_min`, `output_max` (must be finite and ordered) |
| Anti-windup | Integral clamped to `[integral_min, integral_max]` ∩ output range. Conditional integration (no integration further into saturation), or back-calculation when `anti_windup_gain` > 0. |
| Derivative | On measurement by default (`derivative_on_measurement`), first-order filter `derivative_filter_time` |
| Setpoint ramp | `setpoint_ramp_rate` (units/s), starts from the measurement |
| Rate limit | `output_rate_limit` (output units/s) |
| Deadband | `deadband` |
| Feed-forward | `setFeedForward()` (the CSV position loop does not use it; the trajectory velocity is added outside) |
| Manual/auto | `setMode(Manual)`, `setManualOutput()`. Switching back to automatic initializes the integral so the output continues from the manual value. |
| Enable/disable/reset | `setEnabled()`, `reset()`, `resetIntegral()` |
| Robustness | A non-finite input holds the last output and sets `inputFault()`. Invalid configurations are refused (`isValid`). |

`update()` is deterministic, allocation free and noexcept. In the cyclic task, dt is the nominal cycle time (not measured jitter), and the loop is reset whenever the drive is not Operation Enabled.

## Safe defaults and tuning

- The example gains are placeholders. **They are not tuned for any machine.**
- Start with `kp` only and small output limits (e.g. velocity correction ≤ 10 % of `velocity_max`). Keep `ki = 0` and `kd = 0`.
- Increase `kp` until the response is fast without overshoot. Then add a small `ki` to remove the steady-state error, with output limits in place.
- Use `kd` only with `derivative_filter_time` ≥ 2–5 cycles.
- Watch `following_error`, which is in telemetry and `AxisStatus`. Keep `following_error_max` as a guard.
- Stability: the external loop sees at least one cycle of transport delay plus the drive's internal loop delay. Keep the external bandwidth well below the drive's (rule of thumb ≤ 1/5), and well below 1/(10·cycle time).
- The CSV velocity command is always clamped to `velocity_max·(1+velocity_tolerance)`. The CST torque is clamped to `torque_max`.
