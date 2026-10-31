// SPDX-License-Identifier: MIT
//
// CiA 402 (IEC 61800-7-201) power drive state machine.
//
// Pure logic: it consumes decoded feedback (statusword, mode display) and
// produces the controlword bits 0..3 and 7. It never writes blindly: each
// controlword is chosen from the *decoded current state* and the requested
// target state, and every transition is supervised by a timeout.
//
// Real-time safe: no allocation, no locks, no I/O.
#pragma once

#include <cstdint>

#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

namespace cia402 {

// Controlword commands (bits 0..3, 7).
inline constexpr std::uint16_t kCwShutdown = 0x0006;
inline constexpr std::uint16_t kCwSwitchOn = 0x0007;
inline constexpr std::uint16_t kCwDisableOperation = 0x0007;
inline constexpr std::uint16_t kCwEnableOperation = 0x000F;
inline constexpr std::uint16_t kCwDisableVoltage = 0x0000;
inline constexpr std::uint16_t kCwQuickStop = 0x0002;
inline constexpr std::uint16_t kCwFaultReset = 0x0080;
inline constexpr std::uint16_t kCwStateMask = 0x008F;

// Controlword operation-mode specific bits.
inline constexpr std::uint16_t kCwBit4 = 1U << 4;   // PP: new set-point, Homing: start
inline constexpr std::uint16_t kCwBit5 = 1U << 5;   // PP: change set immediately
inline constexpr std::uint16_t kCwBit6 = 1U << 6;   // PP: relative
inline constexpr std::uint16_t kCwHalt = 1U << 8;
inline constexpr std::uint16_t kCwModeSpecificMask = kCwBit4 | kCwBit5 | kCwBit6 | kCwHalt | (1U << 9);

// Statusword bits.
inline constexpr std::uint16_t kSwReadyToSwitchOn = 1U << 0;
inline constexpr std::uint16_t kSwSwitchedOn = 1U << 1;
inline constexpr std::uint16_t kSwOperationEnabled = 1U << 2;
inline constexpr std::uint16_t kSwFault = 1U << 3;
inline constexpr std::uint16_t kSwVoltageEnabled = 1U << 4;
inline constexpr std::uint16_t kSwQuickStop = 1U << 5;        // 0 = quick stop active
inline constexpr std::uint16_t kSwSwitchOnDisabled = 1U << 6;
inline constexpr std::uint16_t kSwWarning = 1U << 7;
inline constexpr std::uint16_t kSwRemote = 1U << 9;
inline constexpr std::uint16_t kSwTargetReached = 1U << 10;
inline constexpr std::uint16_t kSwInternalLimit = 1U << 11;
inline constexpr std::uint16_t kSwBit12 = 1U << 12;  // PP: set-point ack, Homing: attained, CSx: following
inline constexpr std::uint16_t kSwBit13 = 1U << 13;  // PP/CSP: following error, Homing: error

/// Decodes the power state from a statusword using the masks of CiA 402.
DriveState decodeState(std::uint16_t statusword) noexcept;

/// Bit of `mode` in object 0x6502 "Supported drive modes" (0 if none).
std::uint32_t supportedModeBit(OperationMode mode) noexcept;

}  // namespace cia402

/// Target the application asks for.
///  Disable        : controlled (Disable Operation -> Disable Voltage).
///  DisableVoltage : immediate Disable Voltage from any state (transition 9);
///                   the motor is torque free (coasts / brake engages).
enum class DriveRequest : std::uint8_t { Disable, Enable, QuickStop, DisableVoltage };

struct Cia402Config {
    std::uint32_t transition_timeout_cycles = 1000;   ///< Per state transition.
    std::uint32_t fault_reset_pulse_cycles = 10;      ///< Width of the bit-7 pulse.
    std::uint32_t fault_reset_timeout_cycles = 1000;
};

class Cia402Drive {
public:
    explicit Cia402Drive(const Cia402Config& config = {}) noexcept;

    // ---- Requests (taken into account on the next update()) --------------
    void requestEnable() noexcept;
    void requestDisable() noexcept;
    void requestQuickStop() noexcept;
    void requestDisableVoltage() noexcept;
    /// Clears a drive fault with a rising edge on controlword bit 7.
    /// After a successful reset the request falls back to Disable: the
    /// drive is never re-enabled automatically after a fault.
    void requestFaultReset() noexcept;

    /// Processes one cycle of feedback and computes the next controlword.
    void update(std::uint16_t statusword) noexcept;

    /// Recomputes the controlword from the last decoded state and the
    /// current request. Used when a request changes after update() in the
    /// same cycle, or when no fresh feedback arrived (e.g. incomplete
    /// working counter): reactions such as Disable Voltage must still reach
    /// the drive.
    void recompute() noexcept;

    /// Controlword bits 0..3 and 7 for this cycle.
    std::uint16_t controlword() const noexcept { return controlword_; }

    DriveState state() const noexcept { return state_; }
    DriveState previousState() const noexcept { return previous_state_; }
    DriveRequest request() const noexcept { return request_; }
    bool stateChanged() const noexcept { return state_ != previous_state_; }
    bool isOperationEnabled() const noexcept { return state_ == DriveState::OperationEnabled; }
    bool isFault() const noexcept {
        return state_ == DriveState::Fault || state_ == DriveState::FaultReactionActive;
    }
    /// True once the requested target state has been reached.
    bool requestSatisfied() const noexcept;
    /// Set when a transition did not complete within the timeout. Cleared
    /// by the next request.
    bool transitionTimedOut() const noexcept { return timed_out_; }
    bool faultResetFailed() const noexcept { return reset_failed_; }
    bool faultResetPending() const noexcept { return reset_phase_ != ResetPhase::Idle; }

private:
    enum class ResetPhase : std::uint8_t { Idle, Low, Pulse, Wait };

    void clearRequestFlags() noexcept;
    std::uint16_t commandForState() noexcept;

    Cia402Config config_;
    DriveRequest request_ = DriveRequest::Disable;
    DriveState state_ = DriveState::Unknown;
    DriveState previous_state_ = DriveState::Unknown;
    std::uint16_t controlword_ = cia402::kCwDisableVoltage;
    std::uint32_t cycles_in_state_ = 0;
    bool timed_out_ = false;

    ResetPhase reset_phase_ = ResetPhase::Idle;
    std::uint32_t reset_counter_ = 0;
    bool reset_failed_ = false;
};

}  // namespace ethercat_motion
