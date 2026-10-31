// SPDX-License-Identifier: MIT
//
// MQTT control: runs the controller with the MQTT adapter enabled until Ctrl-C.
//
//   mqtt_control [config.yaml] [--broker tcp://127.0.0.1:1883] [--prefix machine]
//
// Try (with mosquitto running):
//   mosquitto_sub -v -t 'machine/#'
//   mosquitto_pub -t machine/axis_x/command -m '{"command":"enable","request_id":"1"}'
//   mosquitto_pub -t machine/axis_x/command -m '{"command":"move_absolute","position":20,"velocity":10,"request_id":"2"}'
//   mosquitto_pub -t machine/ethercat/command -m '{"command":"emergency_stop"}'
//
// MQTT is supervisory only and not a safety-rated stop mechanism.
#include <chrono>
#include <thread>

#include "example_common.hpp"

using namespace ethercat_motion;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    example::installSignalHandlers();
    if (!mqttAvailable()) {
        std::fprintf(stderr, "this build has no MQTT support (install libpaho-mqttpp-dev libpaho-mqtt-dev libssl-dev "
                             "and rebuild)\n");
        return 2;
    }
    auto loaded = loadConfigFile(example::positional(argc, argv, ETHERCAT_MOTION_CONFIG_DIR "/simulation.yaml"));
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.status().toString().c_str());
        return 1;
    }
    Config config = loaded.value();
    config.mqtt.enabled = true;
    config.mqtt.broker = example::option(argc, argv, "--broker", config.mqtt.broker);
    config.mqtt.topic_prefix = example::option(argc, argv, "--prefix", config.mqtt.topic_prefix);
    if (config.ethercat.backend != BackendType::Simulation && !example::hasFlag(argc, argv, "--allow-motion")) {
        std::fprintf(stderr, "refusing to accept MQTT motion commands on real hardware without --allow-motion\n");
        return 1;
    }

    Controller controller(config);
    if (Status s = controller.initialize(); !s) {
        std::fprintf(stderr, "initialize failed: %s\n", s.toString().c_str());
        return 1;
    }
    if (Status s = controller.start(); !s) {
        std::fprintf(stderr, "start failed: %s\n", s.toString().c_str());
        return 1;
    }
    std::printf("MQTT control running (broker %s, prefix '%s'). Ctrl-C to stop.\n", config.mqtt.broker.c_str(),
                config.mqtt.topic_prefix.c_str());
    while (!example::g_stop) {
        std::this_thread::sleep_for(200ms);
        controller.heartbeat();
    }
    std::printf("stopping...\n");
    (void)controller.stop();
    return 0;
}
