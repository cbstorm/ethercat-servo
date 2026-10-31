# Known limitations

## Verification status

| Part | Status |
|---|---|
| Core (CiA 402, trajectory, PID, safety, config, ESI, motion, coordinator, C API, simulation backend) | Built warning-free with GCC 15 (`-Wall -Wextra -Wpedantic -Wshadow ...`). 114 unit/integration tests pass. Also clean under ASan+UBSan, and the threaded tests under TSan. |
| IgH backend (`src/ethercat/igh_backend.cpp`) | Written against the `ecrt` API common to IgH 1.5.2 and 1.6. **Not compiled or tested in the development environment** (IgH was not installed). It needs a build against your IgH installation and the commissioning procedure in hardware_setup.md. |
| MQTT adapter (`src/mqtt/mqtt_adapter.cpp`) | Written against Paho MQTT C++ 1.5 (signatures checked against the installed 1.5.3 headers). **Not compiled in the development environment**, because `libpaho-mqtt-dev`/`libssl-dev` were missing. The JSON codec is fully tested. Build with the packages installed and test against a broker. |
| Real hardware | Not tested. Hardware-dependent assumptions are listed in hardware_setup.md. |

## Design limitations

- **Single process-data domain.** One missing slave makes the whole domain's working counter incomplete. Feedback of *all* axes is then treated as stale for that cycle, and the communication-loss reaction applies to all axes.
- The **blocking SDO upload** (0x6502 check, scan tool) is only used before activation. During operation, parameters are written with non-blocking SDO requests that are created before activation. Arbitrary SDO access at runtime is not exposed.
- Trajectories are **trapezoidal (no jerk limitation)**. S-curve profiles are not implemented.
- The multi-axis coordinator does **joint-space synchronized point-to-point moves**: no path blending, no circular interpolation, no kinematics.
- Torque scaling assumes the CiA 402 default (0.1 % of rated torque). Velocity and acceleration scaling must be configured (`velocity_factor`, `acceleration_factor`).
- Touch probe, digital I/O (0x60FD/0x60FE) and other vendor objects are mapped if configured, but no API is exposed for them.
- Homing uses the drive's homing mode (0x6098). Homing in the library itself, from a switch input, is not implemented. `setPosition()` provides software homing.
- At most 16 axes and 32 slaves (compile-time constants `kMaxAxes`, `kMaxSlaves`).
- DC uses the master as time reference (`ecrt_master_sync_reference_clock`). The reference-clock-to-master mode (`sync_reference_clock_to`) is not used.
- The simulation's drive model (first-order lag, simplified PP/PV/homing) is meant for logic tests. It does not model real dynamics or firmware quirks.
- Timing on a non-PREEMPT_RT kernel is best-effort.

## Open hardware questions

See the list at the end of [hardware_setup.md](hardware_setup.md). The most important ones:

1. The supplied manual describes the **ISL60-PR (pulse/RS485)**, not an EtherCAT drive.
2. ISL-EC 0x6502 = 0x1DD declares **no CST and no Homing** (bit 4 set instead of bit 5).
3. The default PDO does not map 0x6060; whether the firmware accepts remapping is unknown.
4. Units of the velocity and acceleration objects.
