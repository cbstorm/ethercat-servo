# Architecture

## Modules

```
Application
 ├── C++ API (Controller, Axis, MotionController, MultiAxisCoordinator)  /  C API (ethercat_motion.h)
 │
 ├── Configuration Manager ........ config.hpp      YAML -> validated Config (+ ESI cross-checks)
 ├── Diagnostics and Logging ...... diagnostics.hpp spdlog; RT events via wait-free ring
 ├── MQTT Adapter (optional) ...... mqtt_adapter.hpp / mqtt_codec.hpp
 │
 ├── Motion Control Manager ....... motion_controller.hpp     (implements CycleHandler)
 │    ├── Axis (public handle) .... axis.hpp                  non-RT, thread safe, non-blocking
 │    ├── AxisRuntime (RT) ........ src/motion/axis_runtime   one per axis
 │    │    ├── CiA 402 Drive ...... cia402_drive.hpp          power state machine
 │    │    ├── Trajectory ......... trajectory.hpp            trapezoidal profiles
 │    │    └── PID ................ pid_controller.hpp        optional external loop
 │    └── Multi-axis Coordinator .. coordinator.hpp           synchronized moves
 │
 ├── Safety Manager ............... safety_manager.hpp        latching, reactions
 │
 └── EtherCAT Master Manager ...... ethercat_master.hpp       cyclic task, comm supervision
      ├── Slave Manager ........... ethercat_slave.hpp        addressing, identity validation
      ├── PDO Mapping Manager ..... pdo_mapping.hpp / esi_parser.hpp
      └── Backend ................. ethercat_backend.hpp
           ├── IgH (ecrt) ......... src/ethercat/igh_backend.cpp     (hardware)
           └── Simulation ......... simulation.hpp                   (mock CiA 402 drives)
```

Dependency direction is strictly downward:

- The EtherCAT layer knows nothing about motion, PID or MQTT.
- The motion layer knows nothing about MQTT.
- The MQTT adapter talks to the application only through the `MqttCommandTarget` interface. `Controller` implements that interface.
- Raw IgH types (`ec_master_t*`, ...) exist only inside `igh_backend.cpp`.

## Threads

| Thread | Priority | Does | Never does |
|---|---|---|---|
| Cyclic task (`EthercatMaster`) | SCHED_FIFO (configurable), pinned CPU | Receives frames, CiA 402 state machines, trajectories, PID, safety evaluation, writes outputs, sends frames, timing statistics | Allocate, lock a mutex, log, touch files, sockets or MQTT |
| Service thread (`Controller`) | normal | Drains RT events into spdlog, dispatches command results (callbacks, MQTT acknowledgements), RT watchdog | Touch process data |
| MQTT worker | normal | Parses and validates JSON commands, submits them, sends acknowledgements | Touch process data, block the RT thread |
| MQTT publisher | normal | Telemetry and status publishing, reconnect with backoff, applies the `on_disconnect` reaction | Touch process data |
| Paho callback threads | normal | Push received messages into a bounded queue | Anything else |
| Application threads | any | `Axis::*` calls (validate and enqueue), `status()` snapshots | — |

## Data exchange with the cyclic task

| Direction | Mechanism | Properties |
|---|---|---|
| App/MQTT → RT commands | `MpscRing<AxisCommand,256>` | Producers serialize on a mutex. The RT consumer is lock-free and pops at most 32 per cycle. Batches (synchronized moves) are pushed contiguously and applied in the same cycle. |
| RT → app command results | `SpscRing<CommandResult,512>` | One result per command (completed, failed, aborted or rejected) |
| RT → app events | `SpscRing<Event,1024>` | Fixed-size records, logged by the service thread |
| RT → app status | `SeqLock<AxisStatus>`, `SeqLock<MasterStatus>`, `SeqLock<SafetyStatus>` | The writer never waits. Readers retry while a write is in progress. Payload is stored as relaxed atomics, so there are no data races. |
| App → RT safety inputs | `std::atomic` flags | E-stop, reset request, heartbeat |

If a queue is full the request fails immediately with `QueueFull`. Nothing blocks, and the RT side drops events rather than waiting.

## Cycle order

Each iteration of `EthercatMaster::cyclicThread`:

1. `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` until the next absolute deadline.
2. `ecrt_master_receive` + `ecrt_domain_process`.
3. Communication status: working counter (consecutive errors), link, slave AL states (polled every N cycles, and every cycle while the WC is incomplete).
4. `MotionController::onCycle`:
   1. read inputs of every axis (statusword, positions, ...) and update the CiA 402 state machines; local supervision (limits, following error, overspeed);
   2. apply up to 32 queued commands (state-aware validation in RT, rejection → result);
   3. `SafetyManager::evaluate` (latch faults, compute per-axis reactions);
   4. per axis: apply reaction, update command completion, trajectory/handshakes/PID, step async SDO parameter writes, recompute controlword, write outputs, publish status snapshot.
5. `ecrt_domain_queue`, DC sync (`application_time`, `sync_reference_clock`, `sync_slave_clocks`), `ecrt_master_send`.
6. Timing statistics. If the next deadline was already missed, the cycle counts an overrun, skips the lost slots (no burst catch-up) and emits an event.

`Controller::step()` and `EthercatMaster::step()` run exactly one such cycle in the caller's thread. Simulation and the tests use this for deterministic execution.

## Start-up sequence (`Controller::initialize` / `start`)

1. Load YAML, finalize defaults, resolve `pdo.source: esi`, and cross-check the mapping against the ESI. Validate everything with path-qualified error messages.
2. Create the logger, safety manager and axes.
3. Create the backend (IgH or simulation). `EthercatMaster::initialize` requests the master and scans the bus.
4. `MotionController::attach` registers one slave per axis.
5. `EthercatMaster::configure`:
   1. validate every PDO configuration;
   2. resolve alias/position to a ring position;
   3. validate vendor, product and revision for **all** slaves;
   4. only then configure PDOs, startup SDOs and DC. Any mismatch aborts before activation.
6. `MotionController::resolve` locates the PDO entries and creates non-blocking SDO requests for parameters that are not mapped.
7. Verify supported modes: read 0x6502 from each drive (blocking SDO, before activation) and refuse to start if a configured mode is missing.
8. `EthercatMaster::activate`.
9. `start()` launches the cyclic thread, waits until all slaves are OP (timeout), then starts the service thread and MQTT. An MQTT failure is logged, and motion control continues without it.

## Real-time considerations

- `mlockall`, stack prefaulting, `SCHED_FIFO` and CPU pinning are applied inside the cyclic thread before the loop. Missing privileges produce a warning, not a failure.
- All buffers, layouts, SDO requests and trajectories are created before activation. The cycle performs no heap allocation.
- Hard real-time behavior requires a PREEMPT_RT kernel, an isolated CPU, IRQ affinity, and a NIC driven by an IgH native driver or `generic` with low latency. Standard Ubuntu gives best-effort timing only. `ethercat_motion_diagnostic` reports jitter, latency and overruns.
