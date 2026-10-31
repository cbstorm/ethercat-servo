// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "ethercat_motion/mqtt_codec.hpp"

using namespace ethercat_motion;
using json = nlohmann::json;

namespace {

AxisConfig axis() {
    AxisConfig a;
    a.name = "axis_x";
    a.limits.position_min = -100;
    a.limits.position_max = 100;
    a.limits.velocity_max = 50;
    a.limits.acceleration_max = 100;
    a.limits.deceleration_max = 100;
    a.limits.quick_stop_deceleration = 400;
    a.limits.torque_max = 80;
    return a;
}

}  // namespace

TEST(MqttCodec, TopicScheme) {
    TopicScheme t("machine/");
    EXPECT_EQ(t.axis("axis_x", "command"), "machine/axis_x/command");
    EXPECT_EQ(t.machine("status"), "machine/ethercat/status");
    EXPECT_EQ(t.commandWildcard(), "machine/+/command");
    EXPECT_EQ(t.nameFromCommandTopic("machine/axis_x/command").value_or(""), "axis_x");
    EXPECT_FALSE(t.nameFromCommandTopic("machine/a/b/command").has_value());
    EXPECT_FALSE(t.nameFromCommandTopic("other/axis_x/command").has_value());
    EXPECT_FALSE(t.nameFromCommandTopic("machine/axis_x/status").has_value());
}

TEST(MqttCodec, ParsesSpecExampleCommand) {
    auto r = parseAxisCommand(R"({"command":"move_absolute","position":100.0,"velocity":20.0,"request_id":"12345"})",
                              axis(), 3);
    ASSERT_TRUE(r.isOk()) << r.status().toString();
    EXPECT_EQ(r->request_id, "12345");
    EXPECT_EQ(r->command.type, AxisCommandType::MoveAbsolute);
    EXPECT_EQ(r->command.axis, 3);
    EXPECT_DOUBLE_EQ(r->command.value, 100.0);
    EXPECT_DOUBLE_EQ(r->command.velocity, 20.0);
}

TEST(MqttCodec, AllCommandsParse) {
    const char* ok[] = {
        R"({"command":"enable"})",
        R"({"command":"disable"})",
        R"({"command":"reset_fault"})",
        R"({"command":"quick_stop"})",
        R"({"command":"move_relative","distance":-5})",
        R"({"command":"set_velocity","velocity":-12.5})",
        R"({"command":"home"})",
        R"({"command":"set_pid","kp":1,"ki":0.1,"kd":0})",
        R"({"command":"get_status","request_id":7})",
        R"({"command":"stop","deceleration":200})",
        R"({"command":"set_mode","mode":"csv"})",
        R"({"command":"set_torque","torque":10})",
    };
    for (const char* p : ok) {
        auto r = parseAxisCommand(p, axis(), 0);
        EXPECT_TRUE(r.isOk()) << p << ": " << r.status().toString();
    }
    auto status = parseAxisCommand(R"({"command":"get_status","request_id":7})", axis(), 0);
    EXPECT_TRUE(status->get_status);
    EXPECT_EQ(status->request_id, "7");
}

TEST(MqttCodec, RejectsInvalidInput) {
    struct Case {
        const char* payload;
        ErrorCode code;
    };
    const Case cases[] = {
        {"not json", ErrorCode::InvalidArgument},
        {"[1,2]", ErrorCode::InvalidArgument},
        {R"({"position":1})", ErrorCode::InvalidArgument},
        {R"({"command":"fly"})", ErrorCode::InvalidArgument},
        {R"({"command":"move_absolute"})", ErrorCode::InvalidArgument},
        {R"({"command":"move_absolute","position":"10"})", ErrorCode::InvalidArgument},
        {R"({"command":"move_absolute","position":1000})", ErrorCode::LimitViolation},
        {R"({"command":"move_absolute","position":1,"velocity":999})", ErrorCode::LimitViolation},
        {R"({"command":"move_absolute","position":1,"velocity":-1})", ErrorCode::LimitViolation},
        {R"({"command":"set_velocity","velocity":51})", ErrorCode::LimitViolation},
        {R"({"command":"set_torque","torque":81})", ErrorCode::LimitViolation},
        {R"({"command":"set_pid","kp":-1,"ki":0,"kd":0})", ErrorCode::InvalidArgument},
        {R"({"command":"set_mode","mode":"warp"})", ErrorCode::InvalidArgument},
        {R"({"command":"stop","deceleration":1e9})", ErrorCode::LimitViolation},
    };
    for (const auto& c : cases) {
        auto r = parseAxisCommand(c.payload, axis(), 0);
        ASSERT_FALSE(r.isOk()) << c.payload;
        EXPECT_EQ(r.status().code(), c.code) << c.payload << ": " << r.status().toString();
    }
}

TEST(MqttCodec, MachineCommands) {
    EXPECT_EQ(parseMachineCommand(R"({"command":"emergency_stop"})")->command, MachineCommand::EmergencyStop);
    EXPECT_EQ(parseMachineCommand(R"({"command":"reset_faults"})")->command, MachineCommand::ResetFaults);
    EXPECT_FALSE(parseMachineCommand(R"({"command":"self_destruct"})").isOk());
    EXPECT_EQ(extractRequestId(R"({"request_id":"abc"})"), "abc");
    EXPECT_EQ(extractRequestId("garbage"), "");
}

TEST(MqttCodec, TelemetryMatchesSpec) {
    AxisStatus s;
    s.name.assign("axis_x");
    s.position = 99.8;
    s.velocity = 20.0;
    s.enabled = true;
    const json j = json::parse(serializeTelemetry(s, 1720000000123));
    EXPECT_EQ(j["axis"], "axis_x");
    EXPECT_DOUBLE_EQ(j["position"].get<double>(), 99.8);
    EXPECT_DOUBLE_EQ(j["velocity"].get<double>(), 20.0);
    EXPECT_EQ(j["enabled"], true);
    EXPECT_EQ(j["fault"], false);
    EXPECT_EQ(j["timestamp"], 1720000000);
}

TEST(MqttCodec, StatusAndResponses) {
    AxisStatus s;
    s.name.assign("axis_y");
    s.drive_state = DriveState::Fault;
    s.error_code = 0x2310;
    s.safety_faults = safety_fault::kDriveFault;
    json j = json::parse(serializeAxisStatus(s, 0));
    EXPECT_EQ(j["drive_state"], "Fault");
    EXPECT_EQ(j["error_code"], "0x2310");
    EXPECT_EQ(j["safety_faults"][0], "drive_fault");

    j = json::parse(serializeResponse("r1", "enable", "rejected", ErrorCode::SafetyFault, "latched", 0, 0));
    EXPECT_EQ(j["ok"], false);
    EXPECT_EQ(j["status"], "rejected");
    EXPECT_EQ(j["error"], "safety fault");
    EXPECT_FALSE(j.contains("command_id"));

    MasterStatus m;
    m.comm.link_up = true;
    SafetyStatus sf;
    sf.latched_faults = safety_fault::kEmergencyStop;
    j = json::parse(serializeMachineStatus(m, sf, true, 0));
    EXPECT_EQ(j["state"], "online");
    EXPECT_EQ(j["safety"]["latched"][0], "emergency_stop");
    j = json::parse(serializeDiagnostics(m, sf, 0));
    EXPECT_TRUE(j.contains("timing"));
}
