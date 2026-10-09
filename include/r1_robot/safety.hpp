#pragma once

// ---------------------------------------------------------------------------
// r1_robot/safety.hpp
//
// Safety checking framework.  Nothing here drives hardware; it only *rejects*
// commands.  Fail-closed by design: an unknown or unverifiable situation
// produces a fault, never a pass.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

#include "r1_robot/limits.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

enum class SafetyLevel { kOk, kWarning, kFault };

const char* toString(SafetyLevel level);
SafetyLevel safetyLevelFromString(const std::string& s, SafetyLevel fallback);

struct SafetyIssue {
  SafetyLevel level = SafetyLevel::kOk;
  std::string code;     ///< stable machine-readable id, e.g. "cmd.out_of_limits"
  std::string message;  ///< human readable detail
};

struct SafetyReport {
  std::vector<SafetyIssue> issues;

  bool ok() const;            ///< no faults
  bool hasWarnings() const;   ///< at least one warning
  std::vector<std::string> faultCodes() const;
  std::string toString() const;
};

/// All thresholds are *conservative placeholders* until the bring-up procedure
/// in HARDWARE_BRINGUP.md has been executed on the real arm.
struct SafetyConfig {
  /// Master switch.  While false, no backend reporting `isRealHardware()`
  /// may be connected.  Default false on purpose.
  bool allow_real_hardware = false;

  /// Require a valid, in-date feedback frame for *every* joint before any
  /// actuating command is accepted.
  bool require_all_joints_valid = true;

  /// If true, real hardware is refused unless a calibration file was loaded.
  bool require_calibration = true;

  /// Maximum feedback age [s].  TODO(bringup): verify on real 6 Mbaud bus.
  double max_feedback_age_s = 0.05;

  /// Margin kept away from the URDF joint limits [rad].
  double soft_limit_margin = 0.05;

  /// Max |q_cmd - q_meas| per control cycle [rad].  TODO(bringup): tune.
  double max_joint_step_rad = 0.05;

  /// Command envelope.
  double max_kp = 100.0;             ///< [Nm/rad]  TODO(bringup): verify
  double max_kd = 5.0;               ///< [Nm/(rad/s)]
  double max_tau = 30.0;             ///< [Nm]      TODO(bringup): verify
  double max_abs_velocity = 1.0;     ///< [rad/s]   TODO(bringup): verify

  /// Supply voltage floor [V].  TODO(bringup): verify real bus voltage.
  /// The SDK reports error bit 0x8 (transient undervoltage).
  double min_supply_voltage = 40.0;
};

/// Stateless checker; holds only the configuration.
class SafetyMonitor {
 public:
  explicit SafetyMonitor(SafetyConfig cfg = SafetyConfig{});
  SafetyMonitor(const SafetyMonitor&) = default;
  SafetyMonitor& operator=(const SafetyMonitor&) = default;

  const SafetyConfig& config() const { return cfg_; }

  /// Connectivity + freshness + validity + error codes + supply voltage.
  SafetyReport checkState(const RobotState& state, std::uint64_t now_ns) const;

  /// Actuating-command envelope: gains, torque, velocity, soft joint limits and
  /// the per-cycle step limit against the newest state.
  ///
  /// Only called for commands where `hasActuation()` is true; a pure stop
  /// command is always allowed by design (it can only ever remove energy).
  SafetyReport checkCommand(const ArmCommand& cmd, const RobotState& state,
                            const RobotLimits& limits, std::uint64_t now_ns,
                            bool calibration_ok) const;

  /// Preconditions for entering motion-enabled mode.
  SafetyReport checkMotionEnable(const RobotState& state, std::uint64_t now_ns,
                                 bool calibration_ok) const;

  static SafetyReport emptyOk();

 private:
  SafetyConfig cfg_;
};

/// Compose two reports (issues are concatenated).
SafetyReport mergeReports(const SafetyReport& a, const SafetyReport& b);

}  // namespace r1_robot
