#!/usr/bin/env bash
# Installs the apt packages needed to build ethercat_motion (Ubuntu/Debian).
# Run it yourself: it uses sudo and changes only the package set.
# It does NOT install the IgH EtherCAT Master and does NOT touch network
# configuration, services, udev rules or limits - see docs/installation.md.
set -euo pipefail

packages=(build-essential cmake pkg-config libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev libgtest-dev)
if [[ "${1:-}" != "--no-mqtt" ]]; then
    packages+=(libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev mosquitto-clients)
fi

echo "Will run: sudo apt-get install -y ${packages[*]}"
read -r -p "Continue? [y/N] " answer
[[ "$answer" == [yY] ]] || { echo "aborted"; exit 1; }
sudo apt-get install -y "${packages[@]}"
