// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "ethercat_motion/config.hpp"
#include "test_helpers.hpp"

using namespace ethercat_motion;
using ethercat_motion::test::sourcePath;

namespace {

// Minimal valid single-axis document; tests patch fragments of it.
std::string axisDoc(const std::string& axis_extra, const std::string& top_extra = "") {
    return R"(
ethercat:
  backend: simulation
  cycle_time_us: 1000
axes:
  - name: x
    vendor_id: 0x00000A79
    product_code: 0x5800
    counts_per_revolution: 10000
    units_per_revolution: 10
    limits:
      position_min: -10
      position_max: 10
      velocity_max: 5
      acceleration_max: 20
    pdo:
      sync_managers:
        - index: 2
          direction: output
          pdos:
            - index: 0x1600
              entries:
                - [0x6040, 0, 16]
                - [0x607A, 0, 32]
        - index: 3
          direction: input
          pdos:
            - index: 0x1A00
              entries:
                - [0x6041, 0, 16]
                - [0x6064, 0, 32]
)" + axis_extra + top_extra;
}

std::string errorOf(const std::string& yaml) {
    auto c = loadConfigString(yaml, sourcePath("config"));
    return c.isOk() ? std::string() : c.status().message();
}

}  // namespace

TEST(Config, ShippedConfigurationsAreValid) {
    auto sim = loadConfigFile(sourcePath("config/simulation.yaml"));
    ASSERT_TRUE(sim.isOk()) << sim.status().toString();
    EXPECT_EQ(sim->axes.size(), 2U);
    EXPECT_EQ(sim->ethercat.backend, BackendType::Simulation);

    auto hw = loadConfigFile(sourcePath("config/example.yaml"));
    ASSERT_TRUE(hw.isOk()) << hw.status().toString();
    EXPECT_EQ(hw->ethercat.backend, BackendType::Igh);
    const AxisConfig& x = hw->axes[0];
    EXPECT_EQ(x.vendor_id, 0x00000A79U);
    EXPECT_EQ(x.product_code, 0x5800U);
    ASSERT_TRUE(x.revision.has_value());
    EXPECT_EQ(*x.revision, 1U);
    EXPECT_EQ(x.operation_mode, OperationMode::CSP);
    EXPECT_EQ(x.dc_assign_activate.value_or(0), 0x0300);
}

TEST(Config, MinimalDocumentAndDefaults) {
    auto c = loadConfigString(axisDoc(""));
    ASSERT_TRUE(c.isOk()) << c.status().toString();
    const AxisConfig& a = c->axes[0];
    EXPECT_DOUBLE_EQ(a.limits.deceleration_max, 20.0);         // = acceleration_max
    EXPECT_DOUBLE_EQ(a.limits.quick_stop_deceleration, 40.0);  // = 2 x deceleration
    EXPECT_DOUBLE_EQ(a.motion.velocity, 2.5);                  // = velocity_max / 2
    EXPECT_NEAR(a.motion.in_position_window, 10.0 / 1000.0, 1e-12);
    EXPECT_DOUBLE_EQ(a.pid.sample_time, 0.001);
}

TEST(Config, ReportsUnknownKeysWithPath) {
    const std::string e = errorOf(axisDoc("    velocty_max: 3\n"));
    EXPECT_NE(e.find("axes[0].velocty_max: unknown key"), std::string::npos) << e;
}

TEST(Config, ReportsTypeErrors) {
    const std::string e = errorOf(axisDoc("    gear_ratio: fast\n"));
    EXPECT_NE(e.find("axes[0].gear_ratio: expected number"), std::string::npos) << e;
}

TEST(Config, YamlSyntaxError) {
    EXPECT_NE(errorOf("axes: [\n").find("YAML syntax error"), std::string::npos);
}

TEST(Config, MissingRequiredObjects) {
    std::string doc = axisDoc("    operation_mode: CSV\n");
    const std::string e = errorOf(doc);
    EXPECT_NE(e.find("CSV requires 0x60FF"), std::string::npos) << e;
}

TEST(Config, CstNeedsTargetTorque) {
    const std::string e = errorOf(axisDoc("    allowed_modes: [CST]\n"));
    EXPECT_NE(e.find("CST requires 0x6071"), std::string::npos) << e;
}

TEST(Config, ExternalLoopRules) {
    std::string e = errorOf(axisDoc("    control_loop: position\n"));
    EXPECT_NE(e.find("'position' loop requires CSV or CST"), std::string::npos) << e;
    e = errorOf(axisDoc("    control_loop: velocity\n"));
    EXPECT_NE(e.find("'velocity' loop requires CST"), std::string::npos) << e;
}

TEST(Config, LimitValidation) {
    std::string doc = axisDoc("");
    doc.replace(doc.find("position_max: 10"), 16, "position_max: -20");
    const std::string e = errorOf(doc);
    EXPECT_NE(e.find("position_min must be < position_max"), std::string::npos) << e;
}

TEST(Config, HomingRequiresParameters) {
    const std::string e = errorOf(axisDoc("    homing:\n      method: 1\n"));
    EXPECT_NE(e.find("homing.speed_search"), std::string::npos) << e;
}

TEST(Config, DuplicateNames) {
    auto c = loadConfigFile(sourcePath("config/simulation.yaml"));
    ASSERT_TRUE(c.isOk());
    Config cfg = c.value();
    cfg.axes[1].name = cfg.axes[0].name;
    const Status s = validateConfig(cfg);
    EXPECT_FALSE(s.isOk());
    EXPECT_NE(s.message().find("duplicate axis name"), std::string::npos);
}

TEST(Config, EsiSourceUsesDefaultMapping) {
    auto c = loadConfigFile(sourcePath("config/simulation.yaml"));
    ASSERT_TRUE(c.isOk()) << c.status().toString();
    const AxisConfig& y = c->axes[1];
    EXPECT_EQ(y.pdo_source, "esi");
    ASSERT_EQ(y.syncs.size(), 2U);
    EXPECT_EQ(y.syncs[0].pdos[0].index, 0x1600);
    ASSERT_FALSE(y.sdo_init.empty());
    EXPECT_EQ(y.sdo_init[0].index, 0x6060);
    EXPECT_EQ(y.dc_assign_activate.value_or(0), 0x0300);
}

TEST(Config, EsiCrossCheckRejectsNonMappableObject) {
    std::string doc = axisDoc("", "");
    doc.replace(doc.find("[0x607A, 0, 32]"), 15, "[0x6064, 0, 32]");  // actual position in an RxPDO
    doc.replace(doc.find("  cycle_time_us: 1000"), 21, "  cycle_time_us: 1000\n  esi_file: ../esi/LC-EH_V1.5.xml");
    const std::string e = errorOf(doc);
    EXPECT_NE(e.find("0x6064 is not declared RxPDO-mappable"), std::string::npos) << e;
}

TEST(Config, EsiVendorMismatch) {
    std::string doc = axisDoc("");
    doc.replace(doc.find("vendor_id: 0x00000A79"), 21, "vendor_id: 0x00000002");
    doc.replace(doc.find("  cycle_time_us: 1000"), 21, "  cycle_time_us: 1000\n  esi_file: ../esi/LC-EH_V1.5.xml");
    EXPECT_NE(errorOf(doc).find("does not match ESI vendor"), std::string::npos);
}

TEST(Config, MqttValidation) {
    const std::string e =
        errorOf(axisDoc("", "mqtt:\n  enabled: true\n  qos: 3\n  topic_prefix: \"a/#\"\n"));
    EXPECT_NE(e.find("mqtt.qos"), std::string::npos) << e;
    EXPECT_NE(e.find("mqtt.topic_prefix"), std::string::npos) << e;
}

TEST(Config, SlaveDeviceConversion) {
    auto c = loadConfigFile(sourcePath("config/example.yaml"));
    ASSERT_TRUE(c.isOk());
    const SlaveDeviceConfig s = toSlaveDeviceConfig(c->axes[0]);
    EXPECT_EQ(s.vendor_id, 0x0A79U);
    EXPECT_EQ(s.syncs.size(), 2U);
    EXPECT_TRUE(s.dc.enabled);
    EXPECT_EQ(s.dc.assign_activate, 0x0300);
    EXPECT_TRUE(validateSyncConfig(s.syncs, "x").isOk());
}

TEST(Config, SyncValidationCatchesDuplicates) {
    std::vector<SyncManagerConfig> syncs(1);
    syncs[0].index = 2;
    syncs[0].pdos.push_back({0x1600, {{0x6040, 0, 16, ""}, {0x6040, 0, 16, ""}}});
    EXPECT_FALSE(validateSyncConfig(syncs, "x").isOk());
    syncs[0].pdos[0].entries.pop_back();
    syncs[0].direction = PdoDirection::Input;  // 0x1600 is an RxPDO
    EXPECT_FALSE(validateSyncConfig(syncs, "x").isOk());
}

TEST(Config, EsiSupportedModesRejectCst) {
    std::string doc = axisDoc("    allowed_modes: [CST]\n");
    doc.replace(doc.find("[0x607A, 0, 32]"), 15, "[0x607A, 0, 32]\n                - [0x6071, 0, 16]");
    doc.replace(doc.find("  cycle_time_us: 1000"), 21, "  cycle_time_us: 1000\n  esi_file: ../esi/LC-EH_V1.5.xml");
    auto c = loadConfigString(doc, sourcePath("config"));
    ASSERT_TRUE(c.isOk()) << c.status().toString();
    ASSERT_EQ(c->warnings.size(), 1U);
    EXPECT_NE(c->warnings[0].find("mode CST is not supported according to the ESI (0x6502 = 0x1DD"),
              std::string::npos)
        << c->warnings[0];
}
