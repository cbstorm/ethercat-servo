// SPDX-License-Identifier: MIT
//
// Configuration model and YAML loader. Every value that depends on the
// machine or the drive is configurable; nothing drive specific is hardcoded.
// Example values (in particular PID gains) are NOT tuned for any machine.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ethercat_motion/diagnostics.hpp"
#include "ethercat_motion/ethercat_master.hpp"
#include "ethercat_motion/pdo_mapping.hpp"
#include "ethercat_motion/pid_controller.hpp"
#include "ethercat_motion/types.hpp"

namespace ethercat_motion {

enum class BackendType : std::uint8_t { Igh, Simulation };

/// Who closes which loop. See docs/pid.md.
///  None     : the drive owns all loops (CSP: position, CSV: velocity, CST: torque).
///  Position : library closes a position loop. CSV -> PID output is a velocity
///             correction added to the trajectory feed-forward; CST -> torque.
///  Velocity : library closes a velocity loop (CST only); PID output is torque.
enum class ControlLoop : std::uint8_t { None, Position, Velocity };

const char* toString(ControlLoop loop) noexcept;

/// Safety reactions ordered by severity.
enum class SafetyReaction : std::uint8_t { None = 0, Stop = 1, QuickStop = 2, DisableVoltage = 3 };

const char* toString(SafetyReaction reaction) noexcept;
std::optional<SafetyReaction> parseSafetyReaction(const std::string& text) noexcept;

struct AxisLimits {
    bool soft_limits_enabled = true;
    double position_min = 0.0;
    double position_max = 0.0;
    double velocity_max = 0.0;          ///< units/s
    double acceleration_max = 0.0;      ///< units/s^2
    double deceleration_max = 0.0;      ///< units/s^2 (0 -> acceleration_max)
    double quick_stop_deceleration = 0.0;  ///< units/s^2 for software stops (0 -> 2x deceleration_max)
    double following_error_max = 0.0;   ///< units, 0 = monitoring off
    std::uint32_t following_error_cycles = 3;
    double velocity_tolerance = 0.10;   ///< Overspeed fault above velocity_max * (1 + tolerance)
    std::uint32_t velocity_violation_cycles = 5;
    double position_tolerance = 0.0;    ///< Allowed excursion beyond soft limits before fault (units)
    double torque_max = 100.0;          ///< % of rated torque for CST commands
};

struct HomingConfig {
    std::int8_t method = 0;             ///< 0x6098 value. 0 = homing not configured.
    double speed_search = 0.0;          ///< units/s (0x6099:1)
    double speed_zero = 0.0;            ///< units/s (0x6099:2)
    double acceleration = 0.0;          ///< units/s^2 (0x609A)
    double timeout_s = 60.0;
};

struct MotionDefaults {
    double velocity = 0.0;              ///< 0 -> velocity_max * 0.5
    double acceleration = 0.0;          ///< 0 -> acceleration_max
    double deceleration = 0.0;          ///< 0 -> deceleration_max
    double in_position_window = 0.0;    ///< units; 0 -> 1 count equivalent * 10
    std::uint32_t in_position_time_ms = 10;
    double motion_timeout_s = 0.0;      ///< 0 = automatic (planned duration * 1.5 + 2 s)
    std::uint32_t command_watchdog_ms = 0; ///< Velocity/torque commands stop if not refreshed. 0 = off.
};

struct DriveTimeouts {
    std::uint32_t state_transition_ms = 1000;
    std::uint32_t fault_reset_ms = 1000;
    std::uint32_t mode_switch_ms = 1000;
    std::uint32_t sdo_ms = 500;
};

struct SimulatedAxisConfig {
    double position_time_constant_s = 0.002;   ///< Mock servo lag in CSP.
    double velocity_time_constant_s = 0.01;
    double inertia = 0.001;                     ///< For CST: counts/s^2 per 0.1 % torque scaling
    double friction = 0.0;
    std::int32_t initial_position_counts = 0;
    std::uint32_t boot_cycles = 5;              ///< Cycles in NotReadyToSwitchOn after start.
};

struct AxisConfig {
    std::string name;

    // Slave addressing and identity.
    std::uint16_t alias = 0;
    std::uint16_t position = 0;
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::optional<std::uint32_t> revision;

    // Scaling: user units <-> drive counts.
    //   counts = units / units_per_revolution * gear_ratio * counts_per_revolution * direction
    double counts_per_revolution = 0.0;   ///< Drive position units per motor revolution.
    double gear_ratio = 1.0;              ///< Motor revolutions per load revolution.
    double units_per_revolution = 1.0;    ///< User units per load revolution (e.g. screw lead in mm).
    std::string units = "rev";
    int direction = 1;                    ///< +1 or -1
    /// Drive velocity units per (count/s) for 0x606C / 0x60FF / 0x6081 (hardware dependent!).
    double velocity_factor = 1.0;
    /// Drive acceleration units per (count/s^2) for 0x6083 / 0x6084 / 0x609A.
    double acceleration_factor = 1.0;

    OperationMode operation_mode = OperationMode::CSP;
    std::vector<OperationMode> allowed_modes;     ///< Modes the application may switch to.

    AxisLimits limits;
    MotionDefaults motion;
    HomingConfig homing;
    ControlLoop control_loop = ControlLoop::None;
    PidConfig pid;
    DriveTimeouts timeouts;

    // PDO / device configuration.
    std::string pdo_source = "config";            ///< "config" or "esi"
    std::string esi_file;                         ///< Overrides ethercat.esi_file
    std::vector<SyncManagerConfig> syncs;
    std::vector<SdoInitCommand> sdo_init;
    bool dc_enabled = true;
    std::optional<std::uint16_t> dc_assign_activate;
    std::int32_t dc_sync0_shift_ns = 0;

    SimulatedAxisConfig simulation;
};

struct EthercatSection {
    BackendType backend = BackendType::Igh;
    MasterConfig master;
    std::string esi_file;
};

struct MqttConfig {
    bool enabled = false;
    std::string broker = "tcp://127.0.0.1:1883";
    std::string client_id = "ethercat_controller";
    std::string username;
    std::string password;
    int mqtt_version = 4;                 ///< 4 = MQTT 3.1.1, 5 = MQTT 5.0
    int keepalive_s = 20;
    bool clean_session = true;
    int qos = 1;
    std::uint32_t telemetry_interval_ms = 100;
    std::uint32_t status_interval_ms = 1000;
    std::uint32_t reconnect_min_s = 1;
    std::uint32_t reconnect_max_s = 30;
    std::size_t command_queue_size = 64;
    std::string topic_prefix = "machine";
    std::string ethercat_topic = "ethercat";

    // TLS
    bool tls_enabled = false;
    std::string tls_ca_file;
    std::string tls_cert_file;
    std::string tls_key_file;
    bool tls_verify_server = true;

    /// Action when the broker connection is lost.
    SafetyReaction on_disconnect = SafetyReaction::Stop;
};

struct SafetyConfig {
    SafetyReaction emergency_stop_reaction = SafetyReaction::QuickStop;
    SafetyReaction communication_loss_reaction = SafetyReaction::DisableVoltage;
    SafetyReaction overrun_reaction = SafetyReaction::QuickStop;
    SafetyReaction drive_fault_reaction = SafetyReaction::QuickStop;  ///< Applied to the *other* axes.
    SafetyReaction limit_reaction = SafetyReaction::QuickStop;
    std::uint32_t working_counter_error_cycles = 3;
    std::uint32_t max_consecutive_overruns = 10;
    std::uint32_t heartbeat_timeout_ms = 0;   ///< Application heartbeat watchdog (0 = off).
    std::uint32_t rt_watchdog_ms = 100;       ///< Supervisor: cyclic task stalled.
    bool stop_all_on_axis_fault = true;
    bool require_operational_at_start = true;
    /// Read 0x6502 from each drive at start-up and refuse to start if a
    /// configured mode is not supported. Unreadable 0x6502 only warns.
    bool verify_supported_modes = true;
};

struct Config {
    EthercatSection ethercat;
    std::vector<AxisConfig> axes;
    MqttConfig mqtt;
    SafetyConfig safety;
    LoggingConfig logging;
    std::string base_directory;   ///< Directory of the config file (relative paths).
    bool finalized = false;       ///< Set by finalizeConfig(); finalization is not repeated.
    /// Non-fatal findings of finalizeConfig() (e.g. ESI declarations that
    /// contradict the configuration). Logged by Controller::initialize().
    std::vector<std::string> warnings;
};

Result<Config> loadConfigFile(const std::string& path);
Result<Config> loadConfigString(const std::string& yaml, const std::string& base_directory = ".");

/// Fills derived defaults (deceleration = acceleration, ...) and resolves
/// ESI-based PDO configuration. Called by the loaders.
Status finalizeConfig(Config& config);

/// Semantic validation with path-qualified messages, e.g.
/// "axes[0].limits.velocity_max: must be > 0".
Status validateConfig(const Config& config);

/// Builds the slave description used by the EtherCAT master.
SlaveDeviceConfig toSlaveDeviceConfig(const AxisConfig& axis);

}  // namespace ethercat_motion
