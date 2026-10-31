/* SPDX-License-Identifier: MIT
 *
 * Plain C use of the library through the C API (simulation configuration,
 * deterministic stepping without the cyclic thread).
 *
 *   c_api_example [config.yaml]
 */
#include <math.h>
#include <stdio.h>

#include <ethercat_motion/ethercat_motion.h>

static int step_until_enabled(ec_controller_t* c, const char* axis) {
    ec_axis_status_t s;
    int i;
    for (i = 0; i < 2000; ++i) {
        if (ec_axis_get_status(c, axis, &s) == EC_OK && s.enabled) return EC_OK;
        if (ec_controller_step(c, 1) != EC_OK) break;
    }
    return EC_ERR_TIMEOUT;
}

int main(int argc, char** argv) {
    const char* config = argc > 1 ? argv[1] : ETHERCAT_MOTION_CONFIG_DIR "/simulation.yaml";
    ec_controller_t* c = ec_controller_create(config);
    ec_axis_status_t s;
    int rc;
    int i;

    if (c == NULL) return 1;
    rc = ec_controller_initialize(c);
    if (rc != EC_OK) {
        fprintf(stderr, "initialize: %s (%s)\n", ec_error_string(rc), ec_controller_last_error(c));
        ec_controller_destroy(c);
        return 1;
    }

    /* Let the simulated bus reach OP and the drive finish booting. */
    for (i = 0; i < 200; ++i) {
        ec_controller_step(c, 1);
        if (ec_axis_get_status(c, "axis_x", &s) == EC_OK && s.drive_state == EC_DRIVE_SWITCH_ON_DISABLED &&
            s.mode_display == s.mode) {
            break;
        }
    }

    rc = ec_axis_enable(c, "axis_x");
    if (rc == EC_OK) rc = step_until_enabled(c, "axis_x");
    if (rc == EC_OK) rc = ec_axis_move_absolute(c, "axis_x", 12.5);
    for (i = 0; rc == EC_OK && i < 5000; ++i) {
        ec_controller_step(c, 1);
        ec_axis_get_status(c, "axis_x", &s);
        if (s.motion_done && fabs(s.position - 12.5) < 0.01) break;
    }
    if (rc != EC_OK) {
        fprintf(stderr, "error: %s (%s)\n", ec_error_string(rc), ec_controller_last_error(c));
    } else {
        ec_axis_get_status(c, "axis_x", &s);
        printf("axis_x reached %.4f (statusword 0x%04X, drive state %d)\n", s.position, s.statusword, s.drive_state);
    }
    ec_controller_destroy(c);
    return rc == EC_OK ? 0 : 1;
}
