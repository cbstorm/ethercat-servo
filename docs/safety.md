# Safety supervision

> **Limitation.** Everything described here is software running on a general-purpose OS. It is **not** a safety function per ISO 13849, IEC 62061 or IEC 61800-5-2. It does **not** replace a certified safety PLC, a safety-rated STO (Safe Torque Off) circuit, hard-wired emergency-stop hardware, guards, or any other measure your risk assessment requires. Design the machine so it is safe when this software misbehaves or stops running.

## Supervised conditions

| Fault | Detected when | Scope | Default reaction |
|---|---|---|---|
| `emergency_stop` | `Controller::emergencyStop()`, MQTT `emergency_stop` | all axes | `quick_stop` |
| `communication_lost` | link down (after the bus reached OP once) | all | `disable_voltage` |
| `working_counter` | WC incomplete for `working_counter_error_cycles` consecutive cycles | all | `disable_voltage` |
| `slave_not_operational` | a slave left OP | all | `disable_voltage` |
| `deadline_overrun` | `max_consecutive_overruns` consecutive missed deadlines | all | `quick_stop` |
| `heartbeat_timeout` | `heartbeat_timeout_ms` without `Controller::heartbeat()` (after the first heartbeat) | all | `stop` |
| `drive_fault` | drive in Fault / Fault Reaction Active | own axis → disable voltage; others → `drive_fault_reaction` if `stop_all_on_axis_fault` | |
| `position_limit` | actual position crosses a soft limit (+ `position_tolerance`) while enabled | own axis → `limit_reaction`; others as above | |
| `velocity_limit` | \|velocity\| > `velocity_max·(1+velocity_tolerance)` for N cycles | as above | |
| `following_error` | \|setpoint − actual\| > `following_error_max` for N cycles (CSP, external position loops) | as above | |
| `transition_timeout` | CiA 402 state did not change within `state_transition_ms` | own axis → disable voltage | |

Reactions, in increasing severity:

- `stop`: controlled ramp with `quick_stop_deceleration`; the drive stays enabled.
- `quick_stop`: CiA 402 quick stop using the drive's 0x6085 / 0x605A.
- `disable_voltage`: controlword 0x0000. The drive is torque-free, so the axis coasts or the brake engages.

## Additional preventive checks (not latched)

- Commands are validated against limits at submission: soft limits, velocity, acceleration, torque.
- The cyclic task validates the state again (enabled, mode confirmed, not busy).
- Jogs decelerate to rest exactly at the soft limit. Moves further out of the range are rejected; moves back into the range are allowed.
- Velocity and torque commands can require refreshing (`motion.command_watchdog_ms`). The axis stops when they are not refreshed.
- Motion timeout: planned duration × 1.5 + 2 s, or `motion.motion_timeout_s`. On timeout the command fails and the axis stops.
- At start-up:
  - configuration validation;
  - vendor, product and revision check of **every** slave before activation;
  - 0x6502 supported-modes check;
  - all slaves must reach OP within `operational_timeout_ms`.

  Any failure aborts start-up without activating the master.
- RT watchdog: the service thread logs a critical error if the cyclic task stops advancing for `rt_watchdog_ms`. In that situation the drive's own SM watchdog has to stop the motor.

## Latching and reset

- Faults latch on the first occurrence and stay latched after the condition disappears.
- While anything is latched, the affected axes refuse enable and motion commands (`safety fault`). Disable, quick stop, stop and fault reset are always accepted.
- `Controller::resetFaults()` (MQTT `reset_faults`, or `reset_fault` on an axis) requests a reset and pulses the CiA 402 fault reset on faulted drives. The reset is retried for 2 s and only succeeds once **no** condition is active. Otherwise `reset_blocked` is set.
- An E-stop must be released (`releaseEmergencyStop()`) before it can be reset.
- After a reset, drives stay disabled. Re-enabling is always an explicit command.
- The first fault, its axis and its time are recorded (`SafetyStatus::first_fault*`) and logged.

## Behavior in failure situations

| Situation | Behavior |
|---|---|
| EtherCAT cable to one slave breaks | WC incomplete → `working_counter` / `slave_not_operational` latched → `disable_voltage` on all axes (controlword 0 reaches the remaining slaves). The cut-off drive is stopped by its own SM watchdog. Its axis is marked `comm_ok = false` and its request is set to Disable, so it is never re-enabled automatically. When the slave returns, IgH reconfigures it; the drive typically reports a fault (e.g. 0x8130), which needs a reset. |
| Link down | Same, for all axes |
| Cyclic task misses deadlines | Overruns are counted (skipped slots, no burst). After `max_consecutive_overruns`, the `overrun_reaction` applies. |
| Cyclic task stalls completely | Nothing is sent. The drives' SM watchdogs must stop the motors. The service thread logs critically. **Configure the drive's communication-loss reaction.** |
| Drive fault | Faulted axis: disable voltage. Other axes: `drive_fault_reaction` when `stop_all_on_axis_fault: true` (default). |
| Position / velocity limit | `limit_reaction` (quick stop by default) |
| MQTT broker disconnects | `mqtt.on_disconnect` (default `stop`). The cyclic task is unaffected. |
| Process crash | Same as a stalled cyclic task: the drive's watchdog stops the motor |
