# Troubleshooting

Start with `ethercat_motion_diagnostic config.yaml --check` (configuration), then `ethercat_motion_diagnostic config.yaml --seconds 30` (live, read-only). The IgH tools (`ethercat master`, `ethercat slaves -v`, `ethercat pdos`, `dmesg`) show the master's view.

| Symptom | Likely cause | What to do |
|---|---|---|
| `library built without IgH EtherCAT Master support` | `ecrt.h` was not found by CMake | Install IgH, rebuild with `-DEtherCAT_ROOT=/opt/etherlab`. Check that the configure summary says `IgH EtherCAT backend : ON`. |
| `ecrt_request_master(0) failed` | Service not running, NIC not bound, or no permission on `/dev/EtherCAT0` | `sudo ethercatctl start`, check `MASTER0_DEVICE` in `/etc/ethercat.conf`, the udev rule, and the group membership |
| `EtherCAT link is down` | Cable or wrong interface | Check the cable and LEDs, the MAC address in `ethercat.conf`, and `ethercat master` |
| `expected vendor ... found ...` / `expected revision ...` | A different device or firmware than configured | Run `ethercat_motion_scan`. Update the config only after confirming compatibility. |
| `drive reports 0x6502 = ..., mode X is not supported` | The configured mode is not implemented by the firmware | Remove the mode from `operation_mode`/`allowed_modes`. Setting `safety.verify_supported_modes: false` is only appropriate after confirming with the vendor. |
| `slaves did not reach OP within ... ms` | PDO mapping rejected, DC/sync error, or a wrong cycle time | `dmesg` / `ethercat slaves -v`. Try `pdo: {source: esi}`, `dc.enabled: false`, then a 2 ms cycle. Check 0x1C32 sync errors. |
| WC incomplete, `working_counter` fault | Slave lost or bus errors | `ethercat slaves`, cable, EMI, port errors (`ethercat reg_read`) |
| Enable fails with `timeout` (`transition_timeout`) | No DC-bus power, STO active, drive interlock, or the drive needs a different sequence | Check the drive display, STO input, error code 0x603F and the `[axis] drive state ->` log lines |
| Enable rejected with `safety fault` | A latched fault | `Controller::safetyStatus()` / MQTT status shows `latched`. Fix the cause, then `resetFaults()`. |
| Enable rejected with `drive fault` | Drive in Fault | `resetFault()`, then look up 0x603F in the drive manual |
| Motion rejected with `invalid state` | Not enabled, mode switch or homing in progress, or mode not confirmed | Wait for `enable()` (`axis.wait()`) and check `status().mode_display` |
| Motion rejected with `not supported` | Command not valid in the current mode (e.g. position move in CSV without `control_loop: position`, torque outside CST) | See docs/pid.md |
| `following_error` fault during fast moves | Threshold below the real servo lag, poor drive tuning, or wrong scaling | Check `counts_per_revolution` first, then tune the drive, then raise `following_error_max` |
| `velocity_limit` fault | Wrong `velocity_factor` (e.g. drive reports rpm), or a real overspeed | Compare `status().velocity` with the real speed |
| Mode never confirmed | 0x6060 not mapped and the SDO write fails, or the firmware rejects the mode | Look for `SDO parameter write failed` in the log. Check 0x6502. |
| `cyclic task runs without full real-time setup` | No `CAP_SYS_NICE`/`CAP_IPC_LOCK` or rtprio limits | See installation.md §4 |
| Overruns / high jitter | Non-RT kernel, CPU not isolated, power management, slow NIC driver | See installation.md §5. Run the diagnostic for a long time. |
| `cyclic task stalled` | The process was suspended or starved | Fix the scheduling. Make sure the drive's communication-loss reaction stops the motor. |
| MQTT `this build has no MQTT support` | Paho C/C++ or OpenSSL dev packages were missing at configure time | `apt install libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev`, then reconfigure |
| MQTT commands get no answer | Wrong topic prefix or axis name, or the broker is unreachable | `mosquitto_sub -v -t '#'`. The adapter logs `MQTT connected`. |
| `configuration invalid: ... unknown key` | Typo in YAML | The message contains the full path, e.g. `axes[0].limits.velocty_max` |
