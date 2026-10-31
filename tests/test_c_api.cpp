// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstring>

#include <gtest/gtest.h>

#include "ethercat_motion/ethercat_motion.h"
#include "test_helpers.hpp"

using ethercat_motion::test::sourcePath;

namespace {

bool stepUntilStatus(ec_controller_t* c, const char* axis, bool (*pred)(const ec_axis_status_t&), int max = 5000) {
    ec_axis_status_t s{};
    for (int i = 0; i < max; ++i) {
        if (ec_axis_get_status(c, axis, &s) == EC_OK && pred(s)) return true;
        if (ec_controller_step(c, 1) != EC_OK) return false;
    }
    return false;
}

}  // namespace

TEST(CApi, ErrorsForBadArguments) {
    EXPECT_EQ(ec_controller_create(nullptr), nullptr);
    EXPECT_EQ(ec_controller_initialize(nullptr), EC_ERR_INVALID_ARGUMENT);
    EXPECT_STREQ(ec_error_string(EC_ERR_TIMEOUT), "timeout");
    EXPECT_STREQ(ec_error_string(EC_OK), "ok");

    ec_controller_t* c = ec_controller_create("/nonexistent.yaml");
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(ec_controller_initialize(c), EC_ERR_IO);
    EXPECT_NE(std::strstr(ec_controller_last_error(c), "nonexistent"), nullptr);
    ec_controller_destroy(c);
}

TEST(CApi, SimulatedMove) {
    ec_controller_t* c = ec_controller_create(sourcePath("config/simulation.yaml").c_str());
    ASSERT_NE(c, nullptr);
    ASSERT_EQ(ec_controller_initialize(c), EC_OK) << ec_controller_last_error(c);
    EXPECT_EQ(ec_controller_axis_count(c), 2);
    char name[32];
    ASSERT_EQ(ec_controller_axis_name(c, 0, name, sizeof(name)), EC_OK);
    EXPECT_STREQ(name, "axis_x");
    EXPECT_EQ(ec_axis_enable(c, "nope"), EC_ERR_NOT_FOUND);

    ASSERT_TRUE(stepUntilStatus(c, "axis_x", [](const ec_axis_status_t& s) {
        return s.comm_ok && s.drive_state == EC_DRIVE_SWITCH_ON_DISABLED && s.mode_display == EC_MODE_CSP;
    }));
    ASSERT_EQ(ec_axis_enable(c, "axis_x"), EC_OK);
    ASSERT_TRUE(stepUntilStatus(c, "axis_x", [](const ec_axis_status_t& s) { return s.enabled != 0; }));
    EXPECT_EQ(ec_axis_move_absolute(c, "axis_x", 1000.0), EC_ERR_LIMIT);
    ASSERT_EQ(ec_axis_move_absolute(c, "axis_x", 3.0), EC_OK);
    ASSERT_TRUE(stepUntilStatus(c, "axis_x", [](const ec_axis_status_t& s) {
        return s.motion_done && std::fabs(s.position - 3.0) < 0.01;
    }));
    double pos = 0.0;
    ASSERT_EQ(ec_axis_get_position(c, "axis_x", &pos), EC_OK);
    EXPECT_NEAR(pos, 3.0, 0.01);
    ec_controller_destroy(c);
}
