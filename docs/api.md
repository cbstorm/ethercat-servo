# API overview

Full reference: run `doxygen Doxyfile` in the project root (output in `build/docs/html`). The headers are documented.

## Error handling

- Non-real-time calls return `Status` (code + message) or `Result<T>`. Both are `[[nodiscard]]`.
- Exceptions are only thrown by `MotionController::getAxis()` / `Controller::axis()` for unknown names (`std::out_of_range`). Use `findAxis()` for a non-throwing lookup.
- `ErrorCode` values map 1:1 to the C API's `EC_ERR_*` (negated).

## `Controller` (`controller.hpp`)

| Method | |
|---|---|
| `Controller(std::string config_file)`, `Controller(Config)` | Construction never fails; errors surface in `initialize()` |
| `initialize()` | Config, logger, backend, identity validation, PDO/SDO/DC configuration, 0x6502 check, activation |
| `start()` | Cyclic thread, wait for OP, service thread, MQTT |
| `stop(timeout)` | Controlled shutdown: stop axes → disable drives → stop MQTT → stop cyclic thread |
| `step(n)` | Run n cycles synchronously (simulation/tests; only when not started) |
| `axis(name)` | `Axis&` |
| `motion()`, `safety()`, `master()`, `simulation()`, `mqtt()` | Subsystems (`simulation()` is non-null only with the simulation backend) |
| `ethercatStatus()`, `safetyStatus()` | Snapshots |
| `emergencyStop()`, `releaseEmergencyStop()`, `resetFaults()`, `heartbeat()` | Safety |
| `setCommandResultCallback(fn)` | Called from the service thread for every finished command |

## `Axis` (`axis.hpp`)

All methods are thread-safe and non-blocking. Each one validates the request against the axis configuration and enqueues it.

| Method | Notes |
|---|---|
| `enable()`, `disable()`, `quickStop()`, `resetFault()` | Power commands. `disable()` stops first if the axis is moving. |
| `setMode(OperationMode)` | Only while idle; must be in `allowed_modes` |
| `moveAbsolute(pos, MoveParams)`, `moveTo(...)`, `moveRelative(dist, ...)` | `MoveParams{velocity, acceleration, deceleration}`; 0 = configured default |
| `moveVelocity(v, acc)`, `setVelocity(v)` | Signed velocity; soft-limit guarded |
| `setTorque(percent)` | CST without external loop |
| `stop(dec)`, `home()`, `setPosition(p)`, `setPid(kp, ki, kd)` | |
| `status()` / `getStatus()` | `AxisStatus` snapshot: drive state, modes, flags, statusword, error code, positions, velocities, command tracking |
| `getActualPosition()`, `getActualVelocity()` | units, units/s |
| `wait(timeout)` | Waits for completion of the last command submitted through this handle |
| `waitForCommand(id, slot, timeout)` | Waits for a specific command |

Command completion semantics:

| Command | Completes when | Fails with |
|---|---|---|
| `enable` | Operation Enabled | `Timeout`, `DriveFault`, `SafetyFault` |
| `disable` / `quickStop` | Request satisfied | |
| `resetFault` | Fault cleared | `DriveFault` if it persists |
| `moveAbsolute` / `moveRelative` | In position for `in_position_time_ms` | `Timeout`, `Aborted` (superseded), `SafetyFault`, `LimitViolation`, ... |
| `moveVelocity` | Target velocity reached | |
| `stop` | Standstill | |
| `home` | Homing attained and mode restored | |

## `MotionController` and `MultiAxisCoordinator`

- `addAxis(name, AxisConfig)`, `getAxis(name)`, `findAxis(name)`
- `enableAll()`, `disableAll()`, `stopAll()`, `quickStopAll()`, `resetFaultAll()`
- `moveAbsolute(name, ...)`, `moveRelative(name, ...)`, `moveVelocity(name, ...)`, `stop(name)`, `waitAllIdle(timeout)`
- `submit(AxisCommand)`: low level. `submitBatch(commands)` applies all commands in the same cycle.
- `MultiAxisCoordinator::plan(targets, speed_scale)` and `moveSynchronized(...)`: joint-space synchronized point-to-point moves within every axis' limits. `wait(timeout)` waits for them.

## Lower-level modules (usable on their own)

| Header | Use |
|---|---|
| `ethercat_master.hpp` | `EthercatMaster(MasterConfig, backend)`: `initialize`, `addSlave`, `configure`, `activate`, `start(CycleHandler*)`, `step`, `stop`, `getStatus`, `uploadSdo` |
| `simulation.hpp` | `SimulatedBackend(std::vector<SimulatedSlaveSpec>)` + `SimulationControl` fault injection |
| `cia402_drive.hpp` | `Cia402Drive` state machine, `cia402::decodeState` |
| `trajectory.hpp` | `TrapezoidalProfile` |
| `pid_controller.hpp` | `PIDController` |
| `safety_manager.hpp` | `SafetyManager` |
| `esi_parser.hpp` | `parseEsiFile`, `EsiDevice::defaultSyncConfig()`, `startupSdos()`, `supportedDriveModes()`, `esiDeviceToYaml()` |
| `config.hpp` | `loadConfigFile`, `loadConfigString`, `validateConfig`, `toSlaveDeviceConfig` |
| `mqtt_codec.hpp` | JSON parsing/serialization and topics, without an MQTT client |
| `lockfree.hpp` | `SpscRing`, `MpscRing`, `SeqLock` |

## C API (`ethercat_motion.h`)

```
ec_controller_create / _initialize / _start / _stop / _step / _destroy
ec_controller_last_error, ec_error_string
ec_controller_emergency_stop / _reset_faults / _heartbeat
ec_controller_axis_count / _axis_name
ec_axis_enable / _disable / _quick_stop / _reset_fault / _set_mode
ec_axis_move_absolute / _move_absolute_ex / _move_relative / _set_velocity / _set_torque / _stop / _home / _set_pid
ec_axis_get_status / _get_position / _wait
```

Every function returns `EC_OK` (0) or a negative `EC_ERR_*`. `ec_controller_last_error()` returns the message of the last failure on that controller. See `examples/c_api_example.c`.
