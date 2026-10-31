#!/usr/bin/env bash
# Reports which build/runtime dependencies are present. Read-only: changes nothing.
set -u

ok=0
check() {  # name, test command
    if eval "$2" >/dev/null 2>&1; then
        printf '  [ok]      %s\n' "$1"
    else
        printf '  [missing] %s\n' "$1"
        ok=1
    fi
}
optional() {
    if eval "$2" >/dev/null 2>&1; then
        printf '  [ok]      %s\n' "$1"
    else
        printf '  [absent]  %s (optional)\n' "$1"
    fi
}

echo "Required:"
check "cmake >= 3.16"            "cmake --version"
check "C++17 compiler (g++)"     "g++ --version"
check "spdlog (libspdlog-dev)"   "pkg-config --exists spdlog || test -d /usr/include/spdlog"
check "yaml-cpp (libyaml-cpp-dev)" "pkg-config --exists yaml-cpp"
check "nlohmann-json3-dev"       "test -f /usr/include/nlohmann/json.hpp"
echo "Optional:"
optional "GoogleTest (libgtest-dev)"            "test -d /usr/include/gtest"
optional "Paho MQTT C++ (libpaho-mqttpp-dev)"   "test -f /usr/include/mqtt/async_client.h"
optional "Paho MQTT C (libpaho-mqtt-dev)"       "test -f /usr/include/MQTTAsync.h"
optional "OpenSSL (libssl-dev)"                 "test -f /usr/include/openssl/ssl.h"
optional "IgH EtherCAT Master (ecrt.h)"         "test -f /opt/etherlab/include/ecrt.h || test -f /usr/local/include/ecrt.h || test -f /usr/include/ecrt.h"
optional "IgH 'ethercat' tool"                  "command -v ethercat || test -x /opt/etherlab/bin/ethercat"
optional "EtherCAT device /dev/EtherCAT0"       "test -e /dev/EtherCAT0"
optional "mosquitto broker"                     "command -v mosquitto"
echo "System:"
if [ -f /sys/kernel/realtime ] && [ "$(cat /sys/kernel/realtime)" = "1" ] || uname -v | grep -q PREEMPT_RT; then
    echo "  [ok]      PREEMPT_RT kernel"
else
    echo "  [info]    kernel is not PREEMPT_RT (timing is best-effort)"
fi
printf '  [info]    rtprio limit: %s, memlock limit: %s\n' "$(ulimit -r)" "$(ulimit -l)"
exit $ok
