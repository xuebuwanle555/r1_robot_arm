#pragma once

// ---------------------------------------------------------------------------
// r1_robot/types.hpp
//
// Core value types shared by every layer of r1_robot.
//
// CONVENTIONS (do not break these):
//   * Every public joint quantity is expressed in the URDF joint frame
//     (joint_0 .. joint_6, in URDF order) and in SI units:
//         q   [rad],  dq [rad/s],  tau [Nm],  kp [Nm/rad],  kd [Nm/(rad/s)]
//     The motor frame is an implementation detail of the hardware backend and
//     is never leaked upward.  Conversion lives in the official SDK
//     (dir * scale * motor + offset), see src/unitree_backend.cpp.
//   * Timestamps are monotonic nanoseconds (see nowNs()).
//   * No layer may assume that a state is fresh or valid without checking
//     RobotState::valid / SafetyMonitor.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <string>

namespace r1_robot {

/// Number of actuated joints of the Unitree R1-7A (7-DOF).
inline constexpr int kNumJoints = 7;

/// Joint-space vector: URDF joint order, SI units.
using JointVector = std::array<double, kNumJoints>;

/// Per-joint boolean flags (freshness / validity / error).
using JointFlags = std::array<bool, kNumJoints>;

/// Requested motor mode.  Mirrors the official SDK's mode byte (0/1).
enum class ControlMode : std::uint8_t {
  kStop = 0,  ///< mode 0: motor output disabled
  kFoc = 1,   ///< mode 1: FOC (torque / impedance control)
};

const char* toString(ControlMode mode);

/// Per-joint command.  URDF frame + SI units.
struct JointCommand {
  ControlMode mode = ControlMode::kStop;
  double q = 0.0;    ///< [rad]      target position (URDF)
  double dq = 0.0;   ///< [rad/s]    target velocity
  double tau = 0.0;  ///< [Nm]       feed-forward torque
  double kp = 0.0;   ///< [Nm/rad]   position gain
  double kd = 0.0;   ///< [Nm/(rad/s)] damping gain
};

using JointCommandVector = std::array<JointCommand, kNumJoints>;

/// One full-arm command frame.
///
/// `timed_out` mirrors the official SDK flag: when true the motor applies its
/// own communication timeout protection.  Phase 1 always sends false from the
/// high-level API (the official examples do the same) - see the note in
/// HARDWARE_BRINGUP.md.
struct ArmCommand {
  JointCommandVector joints{};
  bool timed_out = false;
  std::uint64_t stamp_ns = 0;

  /// All joints in kStop, zero gains, no feed-forward torque.
  static ArmCommand stop();
  /// FOC with kp = kd = tau = 0 (the arm goes compliant/limp).  Only reachable
  /// through an explicit human-confirmed enable.
  static ArmCommand zeroTorque();
  /// FOC position command for every joint with the same gains.
  static ArmCommand position(const JointVector& q, double kp, double kd);
  /// FOC position command with per-joint vectors.
  static ArmCommand position(const JointVector& q, const JointVector& kp,
                             const JointVector& kd);

  bool isStop() const;       ///< every joint is kStop
  bool hasActuation() const;  ///< any joint requests torque / position / gain
  bool allFinite() const;     ///< no NaN / Inf anywhere
};

/// Per-joint feedback, URDF frame + SI units.
struct JointState {
  double q = 0.0;              ///< [rad]
  double dq = 0.0;             ///< [rad/s]
  double tau = 0.0;            ///< [Nm] estimated by the motor
  double voltage = 0.0;        ///< [V]
  double temperature_c = 0.0;  ///< [degC]
  ControlMode mode = ControlMode::kStop;
  std::uint32_t error_code = 0;
  bool valid = false;  ///< a fresh, CRC-valid feedback frame was parsed
  std::uint64_t stamp_ns = 0;
};

/// Snapshot of the whole arm.  `valid[i] == false` means "do not trust
/// joint i", which blocks every actuating command (see SafetyMonitor).
struct RobotState {
  std::uint64_t stamp_ns = 0;
  JointVector q{};
  JointVector dq{};
  JointVector tau{};
  JointVector voltage{};
  JointVector temperature_c{};
  std::array<ControlMode, kNumJoints> mode{};
  std::array<std::uint32_t, kNumJoints> error_code{};
  JointFlags valid{};
  bool connected = false;

  bool allValid() const;
  int invalidCount() const;
  bool allFinite() const;
  bool anyError() const;
  double minSupplyVoltage() const;
};

/// Monotonic clock in nanoseconds (steady_clock based).
std::uint64_t nowNs();

/// Parse a nanosecond timestamp in a human readable form (debug / logging).
std::string formatTimeNs(std::uint64_t stamp_ns);

}  // namespace r1_robot
