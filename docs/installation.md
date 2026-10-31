# Installation

Nothing in this project changes network configuration, system services or kernel settings for you. Every system change below is a command you run yourself after reading it.

## 1. Packages (Ubuntu 22.04 / 24.04, Debian 12)

```bash
sudo apt install -y build-essential cmake pkg-config \
    libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev libgtest-dev
# Optional MQTT adapter (all three are needed; Ubuntu's libpaho-mqttpp-dev does not pull in the other two):
sudo apt install -y libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev
# Optional, for trying MQTT locally:
sudo apt install -y mosquitto mosquitto-clients
```

`scripts/check_dependencies.sh` reports what is present without changing anything.

## 2. IgH EtherCAT Master (hardware only)

The IgH master is not packaged by Ubuntu. Build it from source (<https://gitlab.com/etherlab.org/ethercat>):

```bash
git clone https://gitlab.com/etherlab.org/ethercat.git && cd ethercat
git checkout stable-1.6            # or a 1.5.x release; both API versions are supported
./bootstrap
./configure --prefix=/opt/etherlab --enable-generic --disable-8139too --disable-eoe
make -j$(nproc) all modules
sudo make modules_install install
sudo depmod
```

Configure the master and the dedicated NIC (example; use **your** interface MAC address):

```bash
sudo cp /opt/etherlab/etc/ethercat.conf /etc/ethercat.conf
sudoedit /etc/ethercat.conf        # MASTER0_DEVICE="aa:bb:cc:dd:ee:ff"  DEVICE_MODULES="generic"
sudo /opt/etherlab/sbin/ethercatctl start      # or the provided systemd unit
/opt/etherlab/bin/ethercat slaves              # the drive(s) should be listed
```

Use a **dedicated** Ethernet interface for EtherCAT. Do not configure an IP address on it, and keep NetworkManager away from it. With the `generic` driver the interface must be up but unconfigured. Native IgH drivers (e1000e, igb, r8169, ...) give lower latency than `generic`.

## 3. Build this library

```bash
mkdir build && cd build
cmake .. -DEtherCAT_ROOT=/opt/etherlab        # omit EtherCAT_ROOT for a simulation-only build
make -j$(nproc)
ctest --output-on-failure
sudo make install                              # headers, libethercat_motion.so/.a, CMake package, pkg-config
```

CMake options:

| Option | Default | Meaning |
|---|---|---|
| `ETHERCAT_MOTION_WITH_IGH` | `AUTO` | `ON` fails if IgH is missing; `OFF` builds simulation only |
| `ETHERCAT_MOTION_WITH_MQTT` | `AUTO` | `ON` fails if Paho is missing |
| `ETHERCAT_MOTION_BUILD_STATIC` | `ON` | Also build `libethercat_motion.a` |
| `ETHERCAT_MOTION_BUILD_TESTS` / `_EXAMPLES` / `_TOOLS` | `ON` | |
| `ETHERCAT_MOTION_WARNINGS_AS_ERRORS` | `OFF` | `-Werror` |
| `EtherCAT_ROOT` | – | IgH install prefix (`/opt/etherlab`) |

The configure summary prints whether the IgH backend and the MQTT adapter are included.

### Using the installed library

```cmake
find_package(ethercat_motion REQUIRED)
target_link_libraries(app PRIVATE ethercat_motion::ethercat_motion)          # shared
# or ethercat_motion::ethercat_motion_static
```

```bash
g++ -std=c++17 app.cpp $(pkg-config --cflags --libs ethercat_motion)
```

If you install to `/usr/local`, run `sudo ldconfig` once so the shared library is found.

## 4. Permissions

The application needs:

1. **Access to `/dev/EtherCAT0`.** Example udev rule (`/etc/udev/rules.d/99-ethercat.rules`):

   ```
   KERNEL=="EtherCAT[0-9]*", MODE="0660", GROUP="ethercat"
   ```

   Then `sudo groupadd ethercat; sudo usermod -aG ethercat $USER` and log in again.

2. **Real-time scheduling and memory locking.** Either grant capabilities to the binary:

   ```bash
   sudo setcap cap_sys_nice,cap_ipc_lock+ep ./your_app
   ```

   or allow them for a group in `/etc/security/limits.d/99-realtime.conf`:

   ```
   @ethercat  -  rtprio   95
   @ethercat  -  memlock  unlimited
   ```

Without these privileges the controller still runs. It logs `cyclic task runs without full real-time setup`, and timing is best-effort.

## 5. Real-time kernel (recommended for production)

- Ubuntu: `sudo pro enable realtime-kernel` (Ubuntu Pro), or build a PREEMPT_RT kernel. Debian ships `linux-image-rt-amd64`.
- Isolate a CPU for the cyclic task: kernel parameters `isolcpus=2 nohz_full=2 rcu_nocbs=2`, then set `ethercat.realtime.cpu_affinity: 2`.
- Move the EtherCAT NIC interrupt to the same isolated CPU (or another dedicated one) through `/proc/irq/<n>/smp_affinity_list`.
- Disable deep C-states and CPU frequency scaling in the BIOS, or with `cpupower`.
- Verify with `ethercat_motion_diagnostic config.yaml --seconds 600`: look at `jitter max`, `latency max` and `overruns`.

The library is PREEMPT_RT compatible. It does **not** make a standard kernel hard real-time.
