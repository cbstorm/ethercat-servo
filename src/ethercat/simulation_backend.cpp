// SPDX-License-Identifier: MIT
#include "ethercat_motion/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include "ethercat_motion/cia402_drive.hpp"
#include "ethercat_motion/ethercat_slave.hpp"
#include "ethercat_motion/trajectory.hpp"

namespace ethercat_motion {

namespace {

// Object dictionary slots of the mock drive.
enum Obj : std::size_t {
    kObjControlword,
    kObjMode,
    kObjTargetPosition,
    kObjTargetVelocity,
    kObjTargetTorque,
    kObjProfileVelocity,
    kObjProfileAcceleration,
    kObjProfileDeceleration,
    kObjQuickStopDeceleration,
    kObjHomingMethod,
    kObjHomingSpeedSearch,
    kObjHomingSpeedZero,
    kObjHomingAcceleration,
    kObjHomeOffset,
    kObjTouchProbeFunction,
    kObjDigitalOutputs,
    kObjMaxTorque,
    kObjTorqueSlope,
    kObjQuickStopOption,
    kObjErrorCode,
    kObjStatusword,
    kObjModeDisplay,
    kObjPositionActual,
    kObjVelocityActual,
    kObjTorqueActual,
    kObjFollowingError,
    kObjDigitalInputs,
    kObjTouchProbeStatus,
    kObjSupportedModes,
    kObjCount,
    kObjUnknown = kObjCount,
};

struct ObjInfo {
    Obj obj;
    bool is_signed;
};

ObjInfo lookup(std::uint16_t index, std::uint8_t sub) noexcept {
    switch (index) {
        case 0x6040: return {kObjControlword, false};
        case 0x6060: return {kObjMode, true};
        case 0x607A: return {kObjTargetPosition, true};
        case 0x60FF: return {kObjTargetVelocity, true};
        case 0x6071: return {kObjTargetTorque, true};
        case 0x6081: return {kObjProfileVelocity, false};
        case 0x6083: return {kObjProfileAcceleration, false};
        case 0x6084: return {kObjProfileDeceleration, false};
        case 0x6085: return {kObjQuickStopDeceleration, false};
        case 0x6098: return {kObjHomingMethod, true};
        case 0x6099:
            if (sub == 1) return {kObjHomingSpeedSearch, false};
            if (sub == 2) return {kObjHomingSpeedZero, false};
            break;
        case 0x609A: return {kObjHomingAcceleration, false};
        case 0x607C: return {kObjHomeOffset, true};
        case 0x60B8: return {kObjTouchProbeFunction, false};
        case 0x60FE:
            if (sub == 1) return {kObjDigitalOutputs, false};
            break;
        case 0x6072: return {kObjMaxTorque, false};
        case 0x6087: return {kObjTorqueSlope, false};
        case 0x605A: return {kObjQuickStopOption, true};
        case 0x603F: return {kObjErrorCode, false};
        case 0x6041: return {kObjStatusword, false};
        case 0x6061: return {kObjModeDisplay, true};
        case 0x6064: return {kObjPositionActual, true};
        case 0x606C: return {kObjVelocityActual, true};
        case 0x6077: return {kObjTorqueActual, true};
        case 0x60F4: return {kObjFollowingError, true};
        case 0x60FD: return {kObjDigitalInputs, false};
        case 0x60B9: return {kObjTouchProbeStatus, false};
        case 0x6502: return {kObjSupportedModes, false};
        default: break;
    }
    return {kObjUnknown, false};
}

std::int64_t signExtend(std::int64_t value, std::size_t bytes) noexcept {
    const unsigned bits = static_cast<unsigned>(bytes * 8);
    if (bits >= 64) return value;
    const std::int64_t mask = (std::int64_t{1} << bits) - 1;
    value &= mask;
    if (value & (std::int64_t{1} << (bits - 1))) value -= (std::int64_t{1} << bits);
    return value;
}

constexpr std::uint16_t kErrorLifeguard = 0x8130;

}  // namespace

struct SimulatedBackend::Impl {
    struct Drive {
        SimulatedSlaveSpec spec;
        std::uint16_t ring_position = 0;
        bool configured = false;
        bool identity_ok = false;
        SlaveDeviceConfig config;
        std::vector<std::pair<PdoEntryLocation, ObjInfo>> outputs;
        std::vector<std::pair<PdoEntryLocation, ObjInfo>> inputs;

        std::array<std::int64_t, kObjCount> od{};
        DriveState state = DriveState::NotReadyToSwitchOn;
        std::uint32_t boot_counter = 0;
        std::uint32_t fault_reaction_counter = 0;
        std::uint16_t previous_cw = 0;         // edge detection in the state machine
        std::uint16_t previous_cw_motion = 0;  // edge detection of mode-specific bits
        double pos = 0.0;  // counts
        double vel = 0.0;  // counts/s
        double torque = 0.0;
        double disturbance = 0.0;
        TrapezoidalProfile profile;
        double profile_t = 0.0;
        bool profile_active = false;
        bool setpoint_ack = false;
        bool target_reached = true;
        bool homing_active = false;
        bool homing_attained = false;
        bool homing_error = false;
        std::int8_t last_mode = 0;

        bool online = true;
        std::uint8_t al_state = 0x01;
        std::uint32_t al_counter = 0;

        SeqLock<SimulatedDriveSnapshot> snapshot;
    };

    struct SdoRequest {
        int handle = -1;
        std::uint16_t index = 0;
        std::uint8_t subindex = 0;
        std::size_t size = 0;
        std::array<std::uint8_t, 8> data{};
        SdoRequestState state = SdoRequestState::Unused;
        int countdown = 0;
        bool write = false;
    };

    struct ControlMessage {
        enum class Kind : std::uint8_t { Fault, Online, Link, Disturbance } kind = Kind::Fault;
        int slave = 0;
        std::int64_t value = 0;
        double dvalue = 0.0;
    };

    std::vector<std::unique_ptr<Drive>> drives;
    std::vector<std::size_t> handle_to_drive;
    std::vector<std::uint8_t> domain;
    std::uint32_t domain_bits = 0;
    std::vector<SdoRequest> sdos;
    MpscRing<ControlMessage, 128> control;
    bool open = false;
    bool active = false;
    bool link_up = true;
    std::uint32_t expected_wc = 0;
    double dt = 0.001;
    std::uint32_t last_wc = 0;

    Drive* driveForHandle(int handle) noexcept {
        if (handle < 0 || static_cast<std::size_t>(handle) >= handle_to_drive.size()) return nullptr;
        return drives[handle_to_drive[static_cast<std::size_t>(handle)]].get();
    }

    void processControl() noexcept {
        ControlMessage msg;
        while (control.tryPop(msg)) {
            if (msg.kind == ControlMessage::Kind::Link) {
                link_up = msg.value != 0;
                continue;
            }
            if (msg.slave < 0 || static_cast<std::size_t>(msg.slave) >= drives.size()) continue;
            Drive& d = *drives[static_cast<std::size_t>(msg.slave)];
            switch (msg.kind) {
                case ControlMessage::Kind::Fault:
                    if (d.state != DriveState::Fault) {
                        d.state = DriveState::FaultReactionActive;
                        d.fault_reaction_counter = 0;
                    }
                    d.od[kObjErrorCode] = msg.value & 0xFFFF;
                    break;
                case ControlMessage::Kind::Online:
                    setOnline(d, msg.value != 0);
                    break;
                case ControlMessage::Kind::Disturbance:
                    d.disturbance = msg.dvalue;
                    break;
                case ControlMessage::Kind::Link:
                    break;
            }
        }
    }

    void setOnline(Drive& d, bool online) noexcept {
        if (d.online == online) return;
        d.online = online;
        if (!online) {
            // SM watchdog expires on the real drive: fault if it was powered.
            if (d.state == DriveState::OperationEnabled || d.state == DriveState::QuickStopActive) {
                d.state = DriveState::Fault;
                d.od[kObjErrorCode] = kErrorLifeguard;
            } else if (d.state == DriveState::SwitchedOn || d.state == DriveState::ReadyToSwitchOn) {
                d.state = DriveState::SwitchOnDisabled;
            }
            d.al_state = 0;
        } else {
            d.al_state = 0x02;  // master re-configures it: PREOP -> SAFEOP -> OP
            d.al_counter = 0;
        }
    }

    void stepAlState(Drive& d) noexcept {
        if (!d.online || !link_up || !active) return;
        if (!d.configured || !d.identity_ok) {
            d.al_state = 0x02;  // misconfigured slave stays in PREOP
            return;
        }
        if (d.al_state == 0x08) return;
        ++d.al_counter;
        if (d.al_counter >= 4) {
            d.al_state = 0x08;
        } else if (d.al_counter >= 2) {
            d.al_state = 0x04;
        } else {
            d.al_state = 0x02;
        }
    }

    bool slaveResponds(const Drive& d) const noexcept { return link_up && d.online && d.al_state >= 0x02; }

    void readOutputs(Drive& d) noexcept {
        for (const auto& [loc, info] : d.outputs) {
            if (info.obj != kObjUnknown) {
                d.od[info.obj] = pd::readEntry(domain.data(), loc, info.is_signed);
            }
        }
    }

    void writeInputs(Drive& d) noexcept {
        for (const auto& [loc, info] : d.inputs) {
            pd::writeEntry(domain.data(), loc, info.obj != kObjUnknown ? d.od[info.obj] : 0);
        }
    }

    double profileVelocity(const Drive& d, Obj obj) const noexcept {
        const double raw = static_cast<double>(d.od[obj]);
        const double factor = d.spec.velocity_factor > 0.0 ? d.spec.velocity_factor : 1.0;
        return raw > 0.0 ? raw / factor : 0.0;
    }
    double profileAccel(const Drive& d, Obj obj) const noexcept {
        const double raw = static_cast<double>(d.od[obj]);
        const double factor = d.spec.acceleration_factor > 0.0 ? d.spec.acceleration_factor : 1.0;
        return raw > 0.0 ? raw / factor : 0.0;
    }

    void stepStateMachine(Drive& d) noexcept {
        const std::uint16_t cw = static_cast<std::uint16_t>(d.od[kObjControlword]);
        const bool reset_edge = (cw & 0x80) != 0 && (d.previous_cw & 0x80) == 0;
        d.previous_cw = cw;
        const bool cmd_shutdown = (cw & 0x87) == 0x06;
        const bool cmd_switch_on = (cw & 0x8F) == 0x07;
        const bool cmd_enable = (cw & 0x8F) == 0x0F;
        const bool cmd_disable_voltage = (cw & 0x82) == 0x00;
        const bool cmd_quick_stop = (cw & 0x86) == 0x02;

        const DriveState before = d.state;
        switch (d.state) {
            case DriveState::Unknown:
            case DriveState::NotReadyToSwitchOn:
                if (++d.boot_counter >= d.spec.boot_cycles) d.state = DriveState::SwitchOnDisabled;
                break;
            case DriveState::SwitchOnDisabled:
                if (cmd_shutdown) d.state = DriveState::ReadyToSwitchOn;
                break;
            case DriveState::ReadyToSwitchOn:
                if (cmd_disable_voltage || cmd_quick_stop) d.state = DriveState::SwitchOnDisabled;
                else if (cmd_switch_on || cmd_enable) d.state = DriveState::SwitchedOn;
                break;
            case DriveState::SwitchedOn:
                if (cmd_disable_voltage || cmd_quick_stop) d.state = DriveState::SwitchOnDisabled;
                else if (cmd_shutdown) d.state = DriveState::ReadyToSwitchOn;
                else if (cmd_enable) d.state = DriveState::OperationEnabled;
                break;
            case DriveState::OperationEnabled:
                if (cmd_disable_voltage) d.state = DriveState::SwitchOnDisabled;
                else if (cmd_quick_stop) d.state = DriveState::QuickStopActive;
                else if (cmd_shutdown) d.state = DriveState::ReadyToSwitchOn;
                else if (cmd_switch_on) d.state = DriveState::SwitchedOn;
                break;
            case DriveState::QuickStopActive:
                if (cmd_disable_voltage) d.state = DriveState::SwitchOnDisabled;
                else if (std::fabs(d.vel) < 1e-6) d.state = DriveState::SwitchOnDisabled;  // option code 2
                break;
            case DriveState::FaultReactionActive:
                if (++d.fault_reaction_counter >= 3) d.state = DriveState::Fault;
                break;
            case DriveState::Fault:
                if (reset_edge) {
                    d.state = DriveState::SwitchOnDisabled;
                    d.od[kObjErrorCode] = 0;
                }
                break;
        }
        if (d.state == DriveState::OperationEnabled && before != DriveState::OperationEnabled) {
            d.profile_active = false;
            d.homing_active = false;
            d.target_reached = true;
        }
    }

    void stepMotion(Drive& d) noexcept {
        const std::uint16_t cw = static_cast<std::uint16_t>(d.od[kObjControlword]);
        const auto mode = static_cast<OperationMode>(static_cast<std::int8_t>(d.od[kObjMode]));
        const double prev_pos = d.pos;
        const bool bit4 = (cw & cia402::kCwBit4) != 0;
        const bool halt = (cw & cia402::kCwHalt) != 0;
        const bool prev_bit4 = (d.previous_cw_motion & cia402::kCwBit4) != 0;
        d.previous_cw_motion = cw;

        if (static_cast<std::int8_t>(mode) != d.last_mode) {
            d.last_mode = static_cast<std::int8_t>(mode);
            d.profile_active = false;
            d.homing_active = false;
            d.homing_attained = false;
            d.homing_error = false;
            d.setpoint_ack = false;
            d.target_reached = true;
        }
        d.od[kObjModeDisplay] = d.od[kObjMode];

        if (d.state != DriveState::OperationEnabled) {
            double qs = profileAccel(d, kObjQuickStopDeceleration);
            if (qs <= 0.0) qs = 1e6;
            if (d.state == DriveState::QuickStopActive || d.state == DriveState::FaultReactionActive) {
                const double dv = qs * dt;
                d.vel = std::fabs(d.vel) <= dv ? 0.0 : d.vel - std::copysign(dv, d.vel);
            } else {
                d.vel *= 0.8;  // coasting
                if (std::fabs(d.vel) < 1e-3) d.vel = 0.0;
            }
            d.pos += d.vel * dt;
            d.torque = 0.0;
            d.od[kObjFollowingError] = 0;
            return;
        }

        switch (mode) {
            case OperationMode::CSP: {
                const double target = static_cast<double>(d.od[kObjTargetPosition]) + d.disturbance;
                const double tau = d.spec.position_time_constant_s;
                d.pos += (target - d.pos) * (dt / (tau + dt));
                d.vel = (d.pos - prev_pos) / dt;
                d.od[kObjFollowingError] = static_cast<std::int64_t>(std::llround(target - d.pos));
                d.target_reached = true;
                break;
            }
            case OperationMode::CSV: {
                const double factor = d.spec.velocity_factor > 0.0 ? d.spec.velocity_factor : 1.0;
                const double v_target = static_cast<double>(d.od[kObjTargetVelocity]) / factor;
                const double tau = d.spec.velocity_time_constant_s;
                d.vel += (v_target - d.vel) * (dt / (tau + dt));
                d.pos += d.vel * dt;
                d.target_reached = std::fabs(v_target - d.vel) < 1.0;
                break;
            }
            case OperationMode::CST: {
                d.torque = static_cast<double>(d.od[kObjTargetTorque]);
                const double inertia = d.spec.inertia > 0.0 ? d.spec.inertia : 1.0;
                const double acc = d.torque / inertia - d.spec.friction * d.vel;
                d.vel += acc * dt;
                d.pos += d.vel * dt;
                d.target_reached = true;
                break;
            }
            case OperationMode::PP: {
                if (bit4 && !prev_bit4) {
                    const bool relative = (cw & cia402::kCwBit6) != 0;
                    const double base = d.profile_active ? d.profile.finalPosition() : d.pos;
                    const double target = static_cast<double>(d.od[kObjTargetPosition]) + (relative ? base : 0.0);
                    double v = profileVelocity(d, kObjProfileVelocity);
                    double a = profileAccel(d, kObjProfileAcceleration);
                    double dec = profileAccel(d, kObjProfileDeceleration);
                    if (v <= 0.0) v = 1.0;
                    if (a <= 0.0) a = 1.0;
                    if (dec <= 0.0) dec = a;
                    d.profile.planPosition(d.pos, d.vel, target, v, a, dec);
                    d.profile_t = 0.0;
                    d.profile_active = true;
                    d.setpoint_ack = true;
                    d.target_reached = false;
                } else if (!bit4) {
                    d.setpoint_ack = false;
                }
                if (halt && d.profile_active && d.profile.kind() != TrapezoidalProfile::Kind::Stop) {
                    double dec = profileAccel(d, kObjProfileDeceleration);
                    if (dec <= 0.0) dec = 1.0;
                    d.profile.planStop(d.pos, d.vel, dec);
                    d.profile_t = 0.0;
                }
                if (d.profile_active) {
                    d.profile_t += dt;
                    const TrajectoryPoint p = d.profile.sample(d.profile_t);
                    d.pos = p.position;
                    d.vel = p.velocity;
                    if (d.profile.finished(d.profile_t)) {
                        d.profile_active = false;
                        d.vel = 0.0;
                        d.target_reached = true;
                    }
                }
                break;
            }
            case OperationMode::PV: {
                const double factor = d.spec.velocity_factor > 0.0 ? d.spec.velocity_factor : 1.0;
                const double v_target = halt ? 0.0 : static_cast<double>(d.od[kObjTargetVelocity]) / factor;
                double a = profileAccel(d, kObjProfileAcceleration);
                double dec = profileAccel(d, kObjProfileDeceleration);
                if (a <= 0.0) a = 1.0;
                if (dec <= 0.0) dec = a;
                const bool speeding_up = std::fabs(v_target) > std::fabs(d.vel) && v_target * d.vel >= 0.0;
                const double step = (speeding_up ? a : dec) * dt;
                const double diff = v_target - d.vel;
                d.vel = std::fabs(diff) <= step ? v_target : d.vel + std::copysign(step, diff);
                d.pos += d.vel * dt;
                d.target_reached = d.vel == v_target;
                break;
            }
            case OperationMode::Homing: {
                if (bit4 && !prev_bit4) {
                    const auto method = static_cast<std::int8_t>(d.od[kObjHomingMethod]);
                    d.homing_attained = false;
                    d.homing_error = false;
                    d.target_reached = false;
                    if (method == 35 || method == 37) {
                        finishHoming(d);
                    } else if (method >= 1 && method <= 34) {
                        double v = profileVelocity(d, kObjHomingSpeedSearch);
                        double a = profileAccel(d, kObjHomingAcceleration);
                        if (v <= 0.0) v = 1000.0;
                        if (a <= 0.0) a = 10000.0;
                        d.profile.planPosition(d.pos, d.vel, static_cast<double>(d.spec.home_switch_counts), v, a, a);
                        d.profile_t = 0.0;
                        d.homing_active = true;
                    } else {
                        d.homing_error = true;
                        d.target_reached = true;
                    }
                }
                if (d.homing_active) {
                    if (!bit4 || halt) {
                        d.homing_active = false;  // interrupted
                        d.vel = 0.0;
                        d.target_reached = true;
                    } else {
                        d.profile_t += dt;
                        const TrajectoryPoint p = d.profile.sample(d.profile_t);
                        d.pos = p.position;
                        d.vel = p.velocity;
                        if (d.profile.finished(d.profile_t)) finishHoming(d);
                    }
                }
                break;
            }
            case OperationMode::None:
                d.vel = 0.0;
                break;
        }
    }

    void finishHoming(Drive& d) noexcept {
        d.homing_active = false;
        d.homing_attained = true;
        d.target_reached = true;
        d.vel = 0.0;
        // The home position becomes the home offset (CiA 402 semantics).
        d.pos = static_cast<double>(d.od[kObjHomeOffset]);
    }

    std::uint16_t composeStatusword(const Drive& d) const noexcept {
        std::uint16_t sw = 0;
        switch (d.state) {
            case DriveState::Unknown:
            case DriveState::NotReadyToSwitchOn: sw = 0x0000; break;
            case DriveState::SwitchOnDisabled: sw = 0x0040; break;
            case DriveState::ReadyToSwitchOn: sw = 0x0031; break;
            case DriveState::SwitchedOn: sw = 0x0033; break;
            case DriveState::OperationEnabled: sw = 0x0037; break;
            case DriveState::QuickStopActive: sw = 0x0017; break;
            case DriveState::FaultReactionActive: sw = 0x000F; break;
            case DriveState::Fault: sw = 0x0008; break;
        }
        sw |= cia402::kSwRemote;
        if (d.state == DriveState::OperationEnabled) {
            const auto mode = static_cast<OperationMode>(static_cast<std::int8_t>(d.od[kObjModeDisplay]));
            if (d.target_reached) sw |= cia402::kSwTargetReached;
            switch (mode) {
                case OperationMode::PP:
                    if (d.setpoint_ack) sw |= cia402::kSwBit12;
                    break;
                case OperationMode::Homing:
                    if (d.homing_attained) sw |= cia402::kSwBit12;
                    if (d.homing_error) sw |= cia402::kSwBit13;
                    break;
                case OperationMode::CSP:
                case OperationMode::CSV:
                case OperationMode::CST:
                    sw |= cia402::kSwBit12;  // drive follows the command value
                    break;
                default:
                    break;
            }
        }
        return sw;
    }

    void stepDrive(Drive& d) noexcept {
        const bool responds = slaveResponds(d);
        if (responds && d.al_state == 0x08) {
            readOutputs(d);
        }
        stepStateMachine(d);
        stepMotion(d);

        const double factor = d.spec.velocity_factor > 0.0 ? d.spec.velocity_factor : 1.0;
        d.od[kObjStatusword] = composeStatusword(d);
        d.od[kObjPositionActual] = static_cast<std::int64_t>(std::llround(d.pos));
        d.od[kObjVelocityActual] = static_cast<std::int64_t>(std::llround(d.vel * factor));
        d.od[kObjTorqueActual] = static_cast<std::int64_t>(std::llround(d.torque));
        if (responds && d.al_state >= 0x04) {
            writeInputs(d);
        }

        SimulatedDriveSnapshot snap;
        snap.controlword = static_cast<std::uint16_t>(d.od[kObjControlword]);
        snap.statusword = static_cast<std::uint16_t>(d.od[kObjStatusword]);
        snap.mode = static_cast<std::int8_t>(d.od[kObjMode]);
        snap.mode_display = static_cast<std::int8_t>(d.od[kObjModeDisplay]);
        snap.target_position = static_cast<std::int32_t>(d.od[kObjTargetPosition]);
        snap.target_velocity = static_cast<std::int32_t>(d.od[kObjTargetVelocity]);
        snap.target_torque = static_cast<std::int16_t>(d.od[kObjTargetTorque]);
        snap.position = d.pos;
        snap.velocity = d.vel;
        snap.error_code = static_cast<std::uint16_t>(d.od[kObjErrorCode]);
        snap.online = d.online;
        snap.al_state = d.al_state;
        d.snapshot.store(snap);
    }

    void stepSdos() noexcept {
        for (auto& r : sdos) {
            if (r.state != SdoRequestState::Busy) continue;
            if (--r.countdown > 0) continue;
            Drive* d = driveForHandle(r.handle);
            if (d == nullptr || !slaveResponds(*d)) {
                r.state = SdoRequestState::Error;
                continue;
            }
            const ObjInfo info = lookup(r.index, r.subindex);
            if (info.obj == kObjUnknown) {
                r.state = SdoRequestState::Error;  // abort: object does not exist
                continue;
            }
            if (r.write) {
                std::int64_t value = 0;
                for (std::size_t i = 0; i < r.size && i < 8; ++i) {
                    value |= static_cast<std::int64_t>(r.data[i]) << (8 * i);
                }
                d->od[info.obj] = info.is_signed ? signExtend(value, r.size) : value;
            } else {
                const std::int64_t value = d->od[info.obj];
                for (std::size_t i = 0; i < r.size && i < 8; ++i) {
                    r.data[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF);
                }
            }
            r.state = SdoRequestState::Success;
        }
    }
};

SimulatedBackend::SimulatedBackend(std::vector<SimulatedSlaveSpec> bus) : impl_(std::make_unique<Impl>()) {
    for (std::size_t i = 0; i < bus.size(); ++i) {
        auto drive = std::make_unique<Impl::Drive>();
        drive->spec = bus[i];
        drive->ring_position = static_cast<std::uint16_t>(i);
        drive->pos = static_cast<double>(bus[i].initial_position_counts);
        drive->od[kObjPositionActual] = bus[i].initial_position_counts;
        drive->od[kObjQuickStopOption] = 2;
        drive->od[kObjSupportedModes] = bus[i].supported_modes;
        impl_->drives.push_back(std::move(drive));
    }
}

SimulatedBackend::~SimulatedBackend() = default;

Status SimulatedBackend::open(unsigned /*master_index*/) {
    impl_->open = true;
    return Status::ok();
}

Result<std::vector<SlaveIdentity>> SimulatedBackend::scanSlaves() {
    if (!impl_->open) {
        return Status(ErrorCode::InvalidState, "simulation backend not open");
    }
    std::vector<SlaveIdentity> out;
    for (const auto& d : impl_->drives) {
        SlaveIdentity id;
        id.ring_position = d->ring_position;
        id.alias = d->spec.alias;
        id.vendor_id = d->spec.vendor_id;
        id.product_code = d->spec.product_code;
        id.revision = d->spec.revision;
        id.serial = d->spec.serial;
        id.al_state = 0x02;
        id.name = d->spec.name;
        out.push_back(id);
    }
    return out;
}

Result<int> SimulatedBackend::configureSlave(const SlaveDeviceConfig& config, std::uint32_t cycle_time_ns,
                                             PdoLayout& layout) {
    if (impl_->active) {
        return Status(ErrorCode::InvalidState, "cannot configure slaves after activation");
    }
    auto scan = scanSlaves();
    if (!scan) return scan.status();
    auto ring = resolveRingPosition(config.alias, config.position, scan.value());
    if (!ring) return ring.status();
    auto& drive = *impl_->drives[ring.value()];
    if (drive.configured) {
        return Status(ErrorCode::InvalidConfig, "slave configured twice");
    }
    drive.configured = true;
    drive.config = config;
    drive.identity_ok = drive.spec.vendor_id == config.vendor_id && drive.spec.product_code == config.product_code;
    impl_->dt = static_cast<double>(cycle_time_ns) * 1e-9;

    bool has_outputs = false;
    bool has_inputs = false;
    for (const auto& [entry, direction] : collectEntries(config.syncs)) {
        PdoEntryLocation loc;
        loc.index = entry.index;
        loc.subindex = entry.subindex;
        loc.bit_length = entry.bit_length;
        loc.direction = direction;
        loc.byte_offset = impl_->domain_bits / 8;
        loc.bit_offset = static_cast<std::uint8_t>(impl_->domain_bits % 8);
        impl_->domain_bits += entry.bit_length;
        layout.add(loc);
        const ObjInfo info = lookup(entry.index, entry.subindex);
        if (direction == PdoDirection::Output) {
            drive.outputs.emplace_back(loc, info);
            has_outputs = true;
        } else {
            drive.inputs.emplace_back(loc, info);
            has_inputs = true;
        }
    }
    impl_->expected_wc += (has_outputs ? 2U : 0U) + (has_inputs ? 1U : 0U);
    impl_->handle_to_drive.push_back(ring.value());
    return static_cast<int>(impl_->handle_to_drive.size() - 1);
}

Result<int> SimulatedBackend::createSdoRequest(int slave, std::uint16_t index, std::uint8_t subindex,
                                               std::size_t size, std::uint32_t /*timeout_ms*/) {
    if (impl_->driveForHandle(slave) == nullptr) {
        return Status(ErrorCode::InvalidArgument, "invalid slave handle");
    }
    if (size == 0 || size > 8) {
        return Status(ErrorCode::InvalidArgument, "SDO request size must be 1..8");
    }
    Impl::SdoRequest request;
    request.handle = slave;
    request.index = index;
    request.subindex = subindex;
    request.size = size;
    impl_->sdos.push_back(request);
    return static_cast<int>(impl_->sdos.size() - 1);
}

Result<std::uint64_t> SimulatedBackend::sdoUploadBlocking(std::uint16_t ring_position, std::uint16_t index,
                                                          std::uint8_t subindex, std::size_t size) {
    if (impl_->active) {
        return Status(ErrorCode::InvalidState, "blocking SDO upload is only allowed before activation");
    }
    if (ring_position >= impl_->drives.size()) {
        return Status(ErrorCode::NotFound, "no slave at ring position " + std::to_string(ring_position));
    }
    const ObjInfo info = lookup(index, subindex);
    if (info.obj == kObjUnknown) {
        return Status(ErrorCode::EthercatError, "SDO abort 0x06020000: object does not exist");
    }
    const std::int64_t value = impl_->drives[ring_position]->od[info.obj];
    const std::uint64_t mask = size >= 8 ? ~0ULL : ((1ULL << (8 * size)) - 1);
    return static_cast<std::uint64_t>(value) & mask;
}

Status SimulatedBackend::activate() {
    if (impl_->active) {
        return {ErrorCode::InvalidState, "already active"};
    }
    impl_->domain.assign((impl_->domain_bits + 7) / 8, 0);
    for (auto& d : impl_->drives) {
        if (!d->configured) continue;
        // Startup SDOs (CoE InitCmds) are applied while entering SAFEOP.
        for (const auto& sdo : d->config.sdo_init) {
            const ObjInfo info = lookup(sdo.index, sdo.subindex);
            if (info.obj != kObjUnknown) {
                d->od[info.obj] = info.is_signed ? signExtend(sdo.value, sdo.size) : sdo.value;
            }
        }
        d->al_state = 0x02;
        d->al_counter = 0;
    }
    impl_->active = true;
    return Status::ok();
}

void SimulatedBackend::deactivate() noexcept { impl_->active = false; }

void SimulatedBackend::close() noexcept { impl_->open = false; }

std::uint8_t* SimulatedBackend::domainData() noexcept { return impl_->domain.data(); }

std::size_t SimulatedBackend::domainSize() const noexcept { return impl_->domain.size(); }

void SimulatedBackend::receive() noexcept {
    if (!impl_->active) return;
    impl_->processControl();
    std::uint32_t wc = 0;
    for (auto& d : impl_->drives) {
        if (!d->configured) continue;
        impl_->stepAlState(*d);
        impl_->stepDrive(*d);
        if (impl_->slaveResponds(*d)) {
            if (!d->inputs.empty() && d->al_state >= 0x04) wc += 1;
            if (!d->outputs.empty() && d->al_state == 0x08) wc += 2;
        }
    }
    impl_->last_wc = wc;
    impl_->stepSdos();
}

void SimulatedBackend::send(std::uint64_t /*app_time_ns*/, bool /*sync_dc*/) noexcept {}

DomainState SimulatedBackend::domainState() noexcept {
    DomainState ds;
    ds.working_counter = impl_->last_wc;
    if (impl_->last_wc == 0) {
        ds.wc_state = WorkingCounterState::Zero;
    } else if (impl_->last_wc == impl_->expected_wc) {
        ds.wc_state = WorkingCounterState::Complete;
    } else {
        ds.wc_state = WorkingCounterState::Incomplete;
    }
    return ds;
}

std::uint32_t SimulatedBackend::expectedWorkingCounter() const noexcept { return impl_->expected_wc; }

MasterLinkState SimulatedBackend::masterState() noexcept {
    MasterLinkState ms;
    ms.link_up = impl_->link_up;
    for (const auto& d : impl_->drives) {
        if (impl_->slaveResponds(*d) || (impl_->link_up && d->online && !impl_->active)) {
            ++ms.slaves_responding;
            ms.al_states = static_cast<std::uint8_t>(ms.al_states | d->al_state);
        }
    }
    return ms;
}

SlaveRuntimeState SimulatedBackend::slaveState(int slave) noexcept {
    SlaveRuntimeState s;
    const Impl::Drive* d = impl_->driveForHandle(slave);
    if (d == nullptr) return s;
    s.online = impl_->link_up && d->online;
    s.al_state = s.online ? d->al_state : 0;
    s.operational = s.online && d->al_state == 0x08;
    return s;
}

SdoRequestState SimulatedBackend::sdoState(int request) noexcept {
    if (request < 0 || static_cast<std::size_t>(request) >= impl_->sdos.size()) return SdoRequestState::Error;
    return impl_->sdos[static_cast<std::size_t>(request)].state;
}

std::uint8_t* SimulatedBackend::sdoData(int request) noexcept {
    if (request < 0 || static_cast<std::size_t>(request) >= impl_->sdos.size()) return nullptr;
    return impl_->sdos[static_cast<std::size_t>(request)].data.data();
}

void SimulatedBackend::sdoWrite(int request) noexcept {
    if (request < 0 || static_cast<std::size_t>(request) >= impl_->sdos.size()) return;
    auto& r = impl_->sdos[static_cast<std::size_t>(request)];
    r.write = true;
    r.state = SdoRequestState::Busy;
    r.countdown = 3;  // mailbox round trip latency
}

void SimulatedBackend::sdoRead(int request) noexcept {
    if (request < 0 || static_cast<std::size_t>(request) >= impl_->sdos.size()) return;
    auto& r = impl_->sdos[static_cast<std::size_t>(request)];
    r.write = false;
    r.state = SdoRequestState::Busy;
    r.countdown = 3;
}

void SimulatedBackend::injectDriveFault(int slave, std::uint16_t error_code) {
    Impl::ControlMessage msg;
    msg.kind = Impl::ControlMessage::Kind::Fault;
    msg.slave = slave;
    msg.value = error_code;
    (void)impl_->control.tryPush(msg);
}

void SimulatedBackend::setSlaveOnline(int slave, bool online) {
    Impl::ControlMessage msg;
    msg.kind = Impl::ControlMessage::Kind::Online;
    msg.slave = slave;
    msg.value = online ? 1 : 0;
    (void)impl_->control.tryPush(msg);
}

void SimulatedBackend::setLinkUp(bool up) {
    Impl::ControlMessage msg;
    msg.kind = Impl::ControlMessage::Kind::Link;
    msg.value = up ? 1 : 0;
    (void)impl_->control.tryPush(msg);
}

void SimulatedBackend::setPositionDisturbance(int slave, double counts) {
    Impl::ControlMessage msg;
    msg.kind = Impl::ControlMessage::Kind::Disturbance;
    msg.slave = slave;
    msg.dvalue = counts;
    (void)impl_->control.tryPush(msg);
}

SimulatedDriveSnapshot SimulatedBackend::driveSnapshot(int slave) const {
    if (slave < 0 || static_cast<std::size_t>(slave) >= impl_->drives.size()) return {};
    return impl_->drives[static_cast<std::size_t>(slave)]->snapshot.load();
}

std::size_t SimulatedBackend::slaveCount() const { return impl_->drives.size(); }

}  // namespace ethercat_motion
