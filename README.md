# ethercat_motion

A C++17 EtherCAT motion-control library for Linux, built on the IgH EtherCAT Master (EtherLab). It provides:

- CiA 402 servo-drive control. The state machine is driven by the decoded drive state, and every transition is supervised by a timeout.
- Multi-axis motion: trapezoidal trajectories and synchronized moves.
- An optional external PID controller.
- Software safety supervision.
- An optional MQTT interface.
- A deterministic simulation backend.
- A C API.

> **SAFETY:** The software supervision in this library is **not safety rated**. It does not replace a certified safety PLC, a safety-rated STO circuit, hard-wired emergency-stop hardware or any other machinery safety function required by your risk assessment. MQTT commands are supervisory commands, not a safety-rated stop mechanism. See [docs/safety.md](docs/safety.md).

## Features

| Area | What you get |
|---|---|
| EtherCAT | IgH `ecrt` backend with RAII. It configures PDO assignment and mapping from YAML or an ESI file, sends startup SDOs, sets up distributed clocks, and monitors the working counter, AL states and link state. Slave identity is validated before the master is activated. Non-blocking SDO requests are available inside the cycle. |
| CiA 402 | Statusword decoding for all 8 states. Controlword transitions are chosen from the decoded state and supervised by timeouts. Fault reset uses a rising edge and never re-enables the drive. Supported modes are PP, PV, Homing, CSP, CSV and CST, plus mode switching through PDO or SDO with confirmation from 0x6061. |
| Motion | Absolute and relative moves, velocity jog, stop, quick stop, homing and torque commands. Trapezoidal profiles are planned analytically, can be replanned at any time and never jump. Soft limits are enforced and a jog stops exactly at the limit. Following-error and overspeed supervision, motion timeouts, engineering units with gear ratio and direction, and synchronized multi-axis moves are included. |
| Control | Industrial PID: anti-windup (clamping and back-calculation), filtered derivative on measurement, setpoint ramp, output rate limit, manual/automatic mode with bumpless transfer, feed-forward and deadband. The control architecture is explicit: the PID only closes loops the drive does not already own. |
| Real time | One cyclic thread with absolute-time `clock_nanosleep`, `SCHED_FIFO`, CPU affinity and `mlockall`. The cycle does no allocation, takes no locks, does no I/O and does no logging. It records jitter, latency, execution time and overruns. |
| Safety | Software E-stop, quick stop, fault latching, and supervision of communication loss, working counter, slave AL state, deadline overruns, drive faults, position and velocity limits, following error, heartbeat and the RT watchdog. Reactions are configurable, and a controlled shutdown is performed on stop. |
| MQTT | Optional Paho MQTT C++ adapter (3.1.1 or 5) with TLS, username/password, Last Will and Testament, reconnect with backoff, JSON commands with validation, acknowledgements and completion messages, and telemetry/status published at a configurable rate. It never touches EtherCAT memory. |
| Simulation | In-process bus of mock CiA 402 drives behind the same backend interface. It includes process image, working counter, AL states, SDO and fault injection: drive fault, cable break, link down and position disturbance. |
| Integration | Shared and static libraries, CMake package export, pkg-config, and a C API (`ethercat_motion.h`). |

## Quick start (simulation, no hardware)

```bash
sudo apt install -y cmake build-essential libspdlog-dev libyaml-cpp-dev libgtest-dev nlohmann-json3-dev \
                    libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev     # MQTT packages are optional
mkdir build && cd build
cmake ..
make -j$(nproc)
ctest --output-on-failure
./examples/simulation            # walkthrough: moves, PP, homing, fault injection, cable break
./examples/multi_axis            # synchronized two-axis moves
```

Or use the Makefile wrapper: `make check-deps`, `make deps`, `make build`, `make test`, `make run-sim` (`make help` lists every target and variable).

The hardware backend needs the IgH EtherCAT Master; see [docs/installation.md](docs/installation.md). Then follow the commissioning checklist in [docs/hardware_setup.md](docs/hardware_setup.md) **before** commanding any motion. On hardware, the examples refuse to command motion unless you pass `--allow-motion`.

## Usage

```cpp
#include <ethercat_motion/ethercat_motion.hpp>
using namespace std::chrono_literals;

int main() {
    ethercat_motion::Controller controller("config.yaml");
    if (auto s = controller.initialize(); !s) { std::puts(s.toString().c_str()); return 1; }
    if (auto s = controller.start(); !s) { std::puts(s.toString().c_str()); return 1; }

    auto& x = controller.axis("axis_x");
    if (!x.enable() || !x.wait(2s)) return 1;            // CiA 402 enable sequence completed
    if (!x.moveAbsolute(100.0) || !x.wait(10s)) return 1; // engineering units (e.g. mm), in position
    auto status = x.status();                            // wait-free snapshot
    std::printf("%s at %.3f\n", status.name.c_str(), status.position);

    (void)controller.stop();         // controlled stop, disable, release
}
```

C:

```c
ec_controller_t* c = ec_controller_create("config.yaml");
ec_controller_initialize(c);
ec_controller_start(c);
ec_axis_enable(c, "axis_x");
ec_axis_wait(c, "axis_x", 2000);
ec_axis_move_absolute(c, "axis_x", 100.0);
ec_axis_wait(c, "axis_x", 10000);
ec_controller_stop(c);
ec_controller_destroy(c);
```

All motion calls are non-blocking. Each one validates the request against the configured limits, then queues it for the cyclic task. Completion is reported through `wait()`, `status()`, or a command-result callback.

## Documentation

| Document | Contents |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Modules, threads, cycle order, data flow, real-time rules |
| [docs/installation.md](docs/installation.md) | Dependencies, IgH master, build options, permissions, PREEMPT_RT |
| [docs/hardware_setup.md](docs/hardware_setup.md) | Commissioning guide, **hardware verification checklist**, ISL-EC findings |
| [docs/cia402.md](docs/cia402.md) | State machine, controlwords, modes, PDO/SDO object access, homing |
| [docs/pid.md](docs/pid.md) | Control architecture (who owns which loop), PID features, tuning |
| [docs/mqtt.md](docs/mqtt.md) | Topics, JSON schema, acknowledgements, failure behavior, TLS |
| [docs/safety.md](docs/safety.md) | Supervised conditions, reactions, latching, limitations |
| [docs/api.md](docs/api.md) | C++ and C API reference overview (`doxygen` generates full docs) |
| [docs/limitations.md](docs/limitations.md) | Known limitations and open hardware questions |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common problems and diagnostics |
| [docs/test_results.md](docs/test_results.md) | Test inventory and latest results |

## Project layout

```
include/ethercat_motion/   public headers (C++ and C API)
src/common                 types, logging/diagnostics, RT helpers
src/ethercat               master manager, slave helpers, PDO layout, IgH + simulation backends
src/cia402                 CiA 402 state machine
src/motion                 axis runtime (RT), axis API, motion controller, trajectory, coordinator
src/control                PID controller
src/safety                 safety manager
src/config                 YAML configuration, ESI (XML) parser
src/mqtt                   MQTT codec (JSON) and Paho adapter
config/                    example.yaml (hardware), simulation.yaml
esi/                       vendor ESI file used by the configs (LC-EH_V1.5.xml)
examples/ tools/ tests/ docs/ cmake/ scripts/
```

## License

MIT, see [LICENSE](LICENSE). The vendor ESI file in `esi/` belongs to its manufacturer.
