# MQTT interface

The MQTT adapter is optional and only *supervisory*. The library builds and runs without it. A broker outage never affects the cyclic task, and an MQTT command is **not** a safety-rated stop.

## Configuration

```yaml
mqtt:
  enabled: true
  broker: "tcp://127.0.0.1:1883"     # ssl://host:8883 for TLS
  client_id: "ethercat_controller"
  username: ""
  password: ""
  version: "3.1.1"                   # or "5"
  keepalive_s: 20
  clean_session: true
  qos: 1                             # commands, responses, status (telemetry uses QoS 0)
  topic_prefix: "machine"
  ethercat_topic: "ethercat"
  telemetry_interval_ms: 100
  status_interval_ms: 1000
  reconnect_min_s: 1                 # exponential backoff
  reconnect_max_s: 30
  command_queue_size: 64
  on_disconnect: stop                # none | stop | quick_stop | disable_voltage
  tls:
    enabled: false
    ca_file: "/etc/ssl/certs/ca.pem"
    cert_file: ""                    # client certificate (mutual TLS)
    key_file: ""
    verify_server: true
```

## Topics

| Topic | Direction | Retained | Content |
|---|---|---|---|
| `<prefix>/ethercat/status` | out | yes | `online`/`offline` (the offline message is also the Last Will and Testament), bus and safety summary |
| `<prefix>/ethercat/diagnostics` | out | no | cycle timing, slave AL states, safety details |
| `<prefix>/ethercat/command` | in | – | machine commands |
| `<prefix>/ethercat/response` | out | no | responses to machine commands |
| `<prefix>/<axis>/command` | in | – | axis commands |
| `<prefix>/<axis>/response` | out | no | `accepted` / `rejected` / `completed` / `failed` |
| `<prefix>/<axis>/status` | out | no | on change and every `status_interval_ms` |
| `<prefix>/<axis>/telemetry` | out | no | every `telemetry_interval_ms` |

## Axis commands

All commands are JSON objects with `"command"`. `"request_id"` (string or number) is optional and echoed back. Unknown fields are ignored. Wrong types, missing fields, non-finite numbers and values outside the axis limits are rejected *before* anything reaches the motion layer. The motion layer validates again, and the cyclic task checks the state, mode and safety permission.

| command | fields |
|---|---|
| `enable`, `disable`, `quick_stop`, `reset_fault`, `home`, `get_status` | – |
| `move_absolute` | `position`; optional `velocity`, `acceleration`, `deceleration` |
| `move_relative` | `distance` (or `position`); optional profile fields |
| `set_velocity` | `velocity` (signed); optional `acceleration` |
| `stop` | optional `deceleration` |
| `set_torque` | `torque` [% rated] (CST without external loop) |
| `set_mode` | `mode`: `PP`, `PV`, `CSP`, `CSV`, `CST` |
| `set_pid` | `kp`, `ki`, `kd` (≥ 0) |
| `set_position` | `position` (redefine the current position) |

Machine commands on `<prefix>/ethercat/command`: `get_status`, `emergency_stop`, `reset_faults`, `enable_all`, `disable_all`, `stop_all`.

Example:

```json
{"command": "move_absolute", "position": 100.0, "velocity": 20.0, "request_id": "12345"}
```

Responses on `machine/axis_x/response`:

```json
{"request_id":"12345","command":"move_absolute","status":"accepted","ok":true,"command_id":17,"timestamp":1720000000,"timestamp_ms":1720000000123}
{"request_id":"12345","command":"move_absolute","status":"completed","ok":true,"command_id":17,"timestamp":1720000002,"timestamp_ms":1720000002410}
```

Errors:

```json
{"request_id":"12346","command":"","status":"rejected","ok":false,"error":"limit violation","message":"field 'position' = 1000 outside [-100, 100]"}
{"request_id":"12347","command":"enable","status":"failed","ok":false,"error":"safety fault","command_id":18}
```

A move that is superseded by a newer command finishes with `"error":"aborted"`.

Telemetry (`machine/axis_x/telemetry`):

```json
{"axis":"axis_x","position":99.8,"velocity":20.0,"torque":0.0,"setpoint_position":99.85,"following_error":0.05,"enabled":true,"fault":false,"moving":true,"timestamp":1720000000,"timestamp_ms":1720000000123}
```

## Threading and failure behavior

- Paho callback threads only push the message into a bounded queue. When the queue is full, the message is dropped and answered with `rejected` / `queue full`.
- A worker thread parses and validates, then submits to the motion command queue. That queue is lock-free for the cyclic task and never waits.
- A publisher thread reads wait-free status snapshots, publishes, and reconnects with exponential backoff. The library's own reconnect loop also covers a broker that is down at start-up.
- When the connection is lost, the `on_disconnect` reaction is applied once from the publisher thread. The default `stop` performs a controlled stop of all axes. Motion that was not started via MQTT is affected too, so choose `none` if MQTT is only used for monitoring.
- MQTT can be disabled (`enabled: false`) or compiled out (`-DETHERCAT_MOTION_WITH_MQTT=OFF`). EtherCAT motion control is unaffected either way.

## Trying it

```bash
sudo apt install mosquitto mosquitto-clients
./examples/mqtt_control                              # simulation config, MQTT forced on
mosquitto_sub -v -t 'machine/#'
mosquitto_pub -t machine/axis_x/command -m '{"command":"enable","request_id":"1"}'
mosquitto_pub -t machine/axis_x/command -m '{"command":"move_absolute","position":20,"velocity":10,"request_id":"2"}'
mosquitto_pub -t machine/ethercat/command -m '{"command":"emergency_stop"}'
mosquitto_pub -t machine/ethercat/command -m '{"command":"reset_faults"}'
```
