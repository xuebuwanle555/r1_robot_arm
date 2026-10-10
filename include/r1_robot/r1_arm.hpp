#pragma once

// ---------------------------------------------------------------------------
// r1_robot/r1_arm.hpp
//
// R1Arm - the hardware abstraction layer used by every upper layer
// (Pinocchio / MuJoCo / ROS2 / MoveIt2 / LeRobot / VLA, all phase 2+).
//
// DESIGN RULES
//   * R1Arm owns the ONLY place that talks to a backend.
//   * Motion is gated three times over:
//       1. SafetyConfig::allow_real_hardware must be true for real hardware.
//       2. enableMotion(human_confirmed = true) must be called explicitly.
//       3. Every actuating command is re-validated by SafetyMonitor.
//   * A pure stop command is always accepted, in any state.
//   * R1Arm never enables the motors by itself - not on connect, not on
//     state read, not on error recovery.
//   * stop() is a mode-0 command: it removes motor output.  It is NOT a
//     brake and must not be assumed to hold the arm against gravity.
// ---------------------------------------------------------------------------

#include <memory>
#include <mutex>
#include <string>

#include "r1_robot/backend.hpp"
#include "r1_robot/limits.hpp"
#include "r1_robot/safety.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

struct R1ArmConfig {
  SafetyConfig safety{};
  /// Limits used for validation.  Defaults to the URDF-derived table.
  RobotLimits limits = r1DefaultLimits();
  /// If true, take limits from the backend instead of `limits`.
  bool use_backend_limits = false;
  /// Informational: path of the calibration file the backend is using.
  std::string calib_path;
};

struct R1ArmStatus {
  bool connected = false;
  bool motion_enabled = false;
  bool real_hardware = false;
  bool calibration_ok = false;
  std::string backend_name;
};

class R1Arm {
 public:
  /// `backend` must not be null.
  explicit R1Arm(ArmBackendPtr backend, R1ArmConfig config = R1ArmConfig{});
  ~R1Arm();

  R1Arm(const R1Arm&) = delete;
  R1Arm& operator=(const R1Arm&) = delete;

  // -- lifecycle -----------------------------------------------------------
  /// Connects the backend.  Refused for real hardware unless
  /// SafetyConfig::allow_real_hardware is set.  Never enables motion.
  bool connect(std::string* error);
  void disconnect();

  bool connected() const;
  /// Snapshot of the status (by value: safe to read from any thread).
  R1ArmStatus status() const;
  std::string describe() const;

  // -- state ---------------------------------------------------------------
  /// Latest state, refreshed from the backend when `refresh` is true.
  RobotState state(bool refresh = true);
  /// Safety evaluation of the latest state (freshness, validity, faults).
  SafetyReport checkState(std::uint64_t now_ns = 0) const;

  // -- motion gating -------------------------------------------------------
  /// Arms the command path.  Requires `human_confirmed == true` AND a healthy
  /// state AND (for real hardware) allow_real_hardware.  Nothing else in this
  /// class may call it.
  bool enableMotion(bool human_confirmed, std::string* error);
  /// Disarms the command path and sends a stop to the backend.
  void disableMotion();
  bool motionEnabled() const;

  // -- commands ------------------------------------------------------------
  /// Validates and forwards one command frame.  Stop commands are always
  /// allowed; actuating commands require motion to be enabled.
  bool sendAction(const ArmCommand& cmd, std::string* error);

  /// Explicit stop (mode 0).  Always allowed.  NOT a brake.
  void stop();

  /// Immediate stop + disarm.  Bypasses every other gate by design.
  void emergencyStop();

  /// Operator bring-up: send ONE zero-torque FOC frame to wake up feedback.
  ///
  /// This POWERS THE MOTOR BRIDGES - the arm goes compliant and will NOT hold
  /// its own weight - so it is deliberately gated:
  ///   * `human_confirmed` must be true, and
  ///   * for a real backend, SafetyConfig::allow_real_hardware must be set.
  ///
  /// It does NOT arm the motion gate and is never called implicitly.  This is
  /// the only path by which anything (including the bring-up tool) can make a
  /// backend emit a non-stop motor frame outside sendAction().
  bool probeFeedbackForBringup(bool human_confirmed, std::string* error);

  /// Blocked in phase 1 - returns false with an explanatory error.
  /// The official SDK examples home by *creeping* toward q = 0 with limited
  /// lead; that behaviour is intentionally not exposed until it has been
  /// brought up and verified on the real arm (see HARDWARE_BRINGUP.md).
  bool home(std::string* error);

  // -- introspection -------------------------------------------------------
  /// Limits used for validation (by value: safe to read from any thread).
  RobotLimits limits() const;
  SafetyConfig safetyConfig() const;
  /// A READ-ONLY handle on the backend.  This deliberately hides every actuating
  /// method (writeCommand/stop/setMotionArmed/probeFeedback are non-const and so
  /// ill-formed through a `const` object), so upper layers can poll state but can
  /// neither drive the motors nor disarm the motion lock behind R1Arm's back.
  /// Commands must go through sendAction(), which re-validates each frame.
  std::shared_ptr<const IArmBackend> backend() const;

  void setLimits(const RobotLimits& limits);

 private:
  // Helpers that assume `mutex_` is already held.
  void refreshLocked();
  RobotState stateLocked(bool refresh);
  SafetyReport checkStateLocked(std::uint64_t now_ns) const;

  ArmBackendPtr backend_;
  R1ArmConfig config_;
  SafetyMonitor monitor_;
  mutable std::mutex mutex_;
  RobotState state_{};
  R1ArmStatus status_{};
  bool motion_enabled_ = false;
  // R1Arm is a friend of MotionControl, so it can mint the token that arms the
  // backend gate.  No other class can obtain one.
  MotionControl control_{};
};

}  // namespace r1_robot
