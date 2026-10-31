/* SPDX-License-Identifier: MIT
 *
 * C API for the EtherCAT Motion Control Library.
 *
 * All functions return EC_OK (0) on success or a negative EC_ERR_* code.
 * The last error message of a controller is available through
 * ec_controller_last_error(); it is stored per controller and the returned
 * pointer is valid until the next failing call on that controller.
 * Motion functions are non-blocking: they validate and enqueue a command.
 * Use ec_axis_wait() to wait for completion.
 */
#ifndef ETHERCAT_MOTION_H
#define ETHERCAT_MOTION_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ec_controller ec_controller_t;

typedef enum {
    EC_OK = 0,
    EC_ERR_INVALID_ARGUMENT = -1,
    EC_ERR_INVALID_CONFIG = -2,
    EC_ERR_NOT_FOUND = -3,
    EC_ERR_INVALID_STATE = -4,
    EC_ERR_TIMEOUT = -5,
    EC_ERR_QUEUE_FULL = -6,
    EC_ERR_LIMIT = -7,
    EC_ERR_NOT_SUPPORTED = -8,
    EC_ERR_ETHERCAT = -9,
    EC_ERR_DEVICE_MISMATCH = -10,
    EC_ERR_DRIVE_FAULT = -11,
    EC_ERR_SAFETY_FAULT = -12,
    EC_ERR_IO = -13,
    EC_ERR_ABORTED = -14,
    EC_ERR_INTERNAL = -15
} ec_error_t;

typedef enum {
    EC_MODE_NONE = 0,
    EC_MODE_PP = 1,
    EC_MODE_PV = 3,
    EC_MODE_HOMING = 6,
    EC_MODE_CSP = 8,
    EC_MODE_CSV = 9,
    EC_MODE_CST = 10
} ec_operation_mode_t;

typedef enum {
    EC_DRIVE_UNKNOWN = 0,
    EC_DRIVE_NOT_READY_TO_SWITCH_ON,
    EC_DRIVE_SWITCH_ON_DISABLED,
    EC_DRIVE_READY_TO_SWITCH_ON,
    EC_DRIVE_SWITCHED_ON,
    EC_DRIVE_OPERATION_ENABLED,
    EC_DRIVE_QUICK_STOP_ACTIVE,
    EC_DRIVE_FAULT_REACTION_ACTIVE,
    EC_DRIVE_FAULT
} ec_drive_state_t;

typedef struct {
    ec_drive_state_t drive_state;
    int32_t mode;               /* ec_operation_mode_t requested */
    int32_t mode_display;       /* reported by the drive */
    int32_t enabled;
    int32_t fault;
    int32_t moving;
    int32_t motion_done;
    int32_t homed;
    int32_t comm_ok;
    uint16_t statusword;
    uint16_t error_code;        /* 0x603F */
    uint32_t safety_faults;
    double position;            /* user units */
    double velocity;            /* user units / s */
    double torque;              /* % rated */
    double following_error;
} ec_axis_status_t;

/* Lifecycle ------------------------------------------------------------ */
ec_controller_t* ec_controller_create(const char* config_file);
int ec_controller_initialize(ec_controller_t* controller);
int ec_controller_start(ec_controller_t* controller);
int ec_controller_stop(ec_controller_t* controller);
/* Simulation/test: run cycles synchronously (initialized, not started). */
int ec_controller_step(ec_controller_t* controller, uint32_t cycles);
void ec_controller_destroy(ec_controller_t* controller);

/* Errors --------------------------------------------------------------- */
const char* ec_controller_last_error(const ec_controller_t* controller);
const char* ec_error_string(int error);

/* Safety --------------------------------------------------------------- */
int ec_controller_emergency_stop(ec_controller_t* controller);
int ec_controller_reset_faults(ec_controller_t* controller);
int ec_controller_heartbeat(ec_controller_t* controller);

/* Axes ----------------------------------------------------------------- */
int ec_controller_axis_count(ec_controller_t* controller);
/* Copies the axis name into `buffer`; returns EC_OK or an error. */
int ec_controller_axis_name(ec_controller_t* controller, int index, char* buffer, size_t size);

int ec_axis_enable(ec_controller_t* controller, const char* axis);
int ec_axis_disable(ec_controller_t* controller, const char* axis);
int ec_axis_quick_stop(ec_controller_t* controller, const char* axis);
int ec_axis_reset_fault(ec_controller_t* controller, const char* axis);
int ec_axis_set_mode(ec_controller_t* controller, const char* axis, ec_operation_mode_t mode);
/* velocity/acceleration/deceleration: 0 = configured default */
int ec_axis_move_absolute(ec_controller_t* controller, const char* axis, double position);
int ec_axis_move_absolute_ex(ec_controller_t* controller, const char* axis, double position, double velocity,
                             double acceleration, double deceleration);
int ec_axis_move_relative(ec_controller_t* controller, const char* axis, double distance);
int ec_axis_set_velocity(ec_controller_t* controller, const char* axis, double velocity);
int ec_axis_set_torque(ec_controller_t* controller, const char* axis, double torque_percent);
int ec_axis_stop(ec_controller_t* controller, const char* axis);
int ec_axis_home(ec_controller_t* controller, const char* axis);
int ec_axis_set_pid(ec_controller_t* controller, const char* axis, double kp, double ki, double kd);
int ec_axis_get_status(ec_controller_t* controller, const char* axis, ec_axis_status_t* status);
int ec_axis_get_position(ec_controller_t* controller, const char* axis, double* position);
/* Waits for completion of the last command submitted to the axis. */
int ec_axis_wait(ec_controller_t* controller, const char* axis, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ETHERCAT_MOTION_H */
