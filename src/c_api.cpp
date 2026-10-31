// SPDX-License-Identifier: MIT
#include "ethercat_motion/ethercat_motion.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>

#include "ethercat_motion/controller.hpp"

using namespace ethercat_motion;

struct ec_controller {
    std::unique_ptr<Controller> controller;
    mutable std::mutex error_mutex;
    std::string last_error;
};

namespace {

int toCode(const Status& status) { return -static_cast<int>(status.code()); }

int report(ec_controller_t* c, const Status& status) {
    if (!status) {
        std::lock_guard<std::mutex> lock(c->error_mutex);
        c->last_error = status.toString();
    }
    return toCode(status);
}

template <typename Fn>
int guarded(ec_controller_t* c, Fn fn) {
    if (c == nullptr) return EC_ERR_INVALID_ARGUMENT;
    try {
        return report(c, fn());
    } catch (const std::exception& ex) {
        return report(c, Status(ErrorCode::Internal, ex.what()));
    }
}

template <typename Fn>
int withAxis(ec_controller_t* c, const char* axis, Fn fn) {
    return guarded(c, [&]() -> Status {
        if (axis == nullptr) return {ErrorCode::InvalidArgument, "axis name is NULL"};
        Axis* a = c->controller->motion().findAxis(axis);
        if (a == nullptr) return {ErrorCode::NotFound, std::string("unknown axis '") + axis + "'"};
        return fn(*a);
    });
}

}  // namespace

extern "C" {

ec_controller_t* ec_controller_create(const char* config_file) {
    if (config_file == nullptr) return nullptr;
    try {
        auto* c = new ec_controller;
        c->controller = std::make_unique<Controller>(std::string(config_file));
        return c;
    } catch (...) {
        return nullptr;
    }
}

int ec_controller_initialize(ec_controller_t* c) {
    return guarded(c, [&] { return c->controller->initialize(); });
}

int ec_controller_start(ec_controller_t* c) {
    return guarded(c, [&] { return c->controller->start(); });
}

int ec_controller_stop(ec_controller_t* c) {
    return guarded(c, [&] { return c->controller->stop(); });
}

int ec_controller_step(ec_controller_t* c, uint32_t cycles) {
    return guarded(c, [&] { return c->controller->step(cycles); });
}

void ec_controller_destroy(ec_controller_t* c) { delete c; }

const char* ec_controller_last_error(const ec_controller_t* c) {
    if (c == nullptr) return "controller is NULL";
    std::lock_guard<std::mutex> lock(c->error_mutex);
    return c->last_error.c_str();
}

const char* ec_error_string(int error) {
    if (error > 0 || error < EC_ERR_INTERNAL) return "unknown error";
    return toString(static_cast<ErrorCode>(-error));
}

int ec_controller_emergency_stop(ec_controller_t* c) {
    return guarded(c, [&] {
        c->controller->emergencyStop();
        return Status::ok();
    });
}

int ec_controller_reset_faults(ec_controller_t* c) {
    return guarded(c, [&] { return c->controller->resetFaults(); });
}

int ec_controller_heartbeat(ec_controller_t* c) {
    return guarded(c, [&] {
        c->controller->heartbeat();
        return Status::ok();
    });
}

int ec_controller_axis_count(ec_controller_t* c) {
    if (c == nullptr) return EC_ERR_INVALID_ARGUMENT;
    return static_cast<int>(c->controller->motion().axisCount());
}

int ec_controller_axis_name(ec_controller_t* c, int index, char* buffer, size_t size) {
    return guarded(c, [&]() -> Status {
        if (buffer == nullptr || size == 0) return {ErrorCode::InvalidArgument, "buffer is NULL or empty"};
        if (index < 0 || static_cast<std::size_t>(index) >= c->controller->motion().axisCount()) {
            return {ErrorCode::NotFound, "axis index out of range"};
        }
        const std::string& name = c->controller->motion().axis(static_cast<std::size_t>(index)).name();
        if (name.size() + 1 > size) return {ErrorCode::InvalidArgument, "buffer too small"};
        std::memcpy(buffer, name.c_str(), name.size() + 1);
        return Status::ok();
    });
}

int ec_axis_enable(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.enable(); });
}

int ec_axis_disable(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.disable(); });
}

int ec_axis_quick_stop(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.quickStop(); });
}

int ec_axis_reset_fault(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.resetFault(); });
}

int ec_axis_set_mode(ec_controller_t* c, const char* axis, ec_operation_mode_t mode) {
    return withAxis(c, axis, [mode](Axis& a) { return a.setMode(static_cast<OperationMode>(mode)); });
}

int ec_axis_move_absolute(ec_controller_t* c, const char* axis, double position) {
    return withAxis(c, axis, [position](Axis& a) { return a.moveAbsolute(position); });
}

int ec_axis_move_absolute_ex(ec_controller_t* c, const char* axis, double position, double velocity,
                             double acceleration, double deceleration) {
    return withAxis(c, axis, [&](Axis& a) {
        MoveParams p;
        p.velocity = velocity;
        p.acceleration = acceleration;
        p.deceleration = deceleration;
        return a.moveAbsolute(position, p);
    });
}

int ec_axis_move_relative(ec_controller_t* c, const char* axis, double distance) {
    return withAxis(c, axis, [distance](Axis& a) { return a.moveRelative(distance); });
}

int ec_axis_set_velocity(ec_controller_t* c, const char* axis, double velocity) {
    return withAxis(c, axis, [velocity](Axis& a) { return a.moveVelocity(velocity); });
}

int ec_axis_set_torque(ec_controller_t* c, const char* axis, double torque_percent) {
    return withAxis(c, axis, [torque_percent](Axis& a) { return a.setTorque(torque_percent); });
}

int ec_axis_stop(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.stop(); });
}

int ec_axis_home(ec_controller_t* c, const char* axis) {
    return withAxis(c, axis, [](Axis& a) { return a.home(); });
}

int ec_axis_set_pid(ec_controller_t* c, const char* axis, double kp, double ki, double kd) {
    return withAxis(c, axis, [&](Axis& a) { return a.setPid(kp, ki, kd); });
}

int ec_axis_get_status(ec_controller_t* c, const char* axis, ec_axis_status_t* out) {
    return withAxis(c, axis, [out](Axis& a) -> Status {
        if (out == nullptr) return {ErrorCode::InvalidArgument, "status pointer is NULL"};
        const AxisStatus s = a.status();
        out->drive_state = static_cast<ec_drive_state_t>(s.drive_state);
        out->mode = static_cast<int32_t>(s.mode);
        out->mode_display = static_cast<int32_t>(s.mode_display);
        out->enabled = s.enabled;
        out->fault = s.fault;
        out->moving = s.moving;
        out->motion_done = s.motion_done;
        out->homed = s.homed;
        out->comm_ok = s.comm_ok;
        out->statusword = s.statusword;
        out->error_code = s.error_code;
        out->safety_faults = s.safety_faults;
        out->position = s.position;
        out->velocity = s.velocity;
        out->torque = s.torque;
        out->following_error = s.following_error;
        return Status::ok();
    });
}

int ec_axis_get_position(ec_controller_t* c, const char* axis, double* position) {
    return withAxis(c, axis, [position](Axis& a) -> Status {
        if (position == nullptr) return {ErrorCode::InvalidArgument, "position pointer is NULL"};
        *position = a.getActualPosition();
        return Status::ok();
    });
}

int ec_axis_wait(ec_controller_t* c, const char* axis, uint32_t timeout_ms) {
    return withAxis(c, axis, [timeout_ms](Axis& a) { return a.wait(std::chrono::milliseconds(timeout_ms)); });
}

}  // extern "C"
