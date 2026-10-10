#pragma once

// ---------------------------------------------------------------------------
// r1_robot/control_service.hpp
//
// ControlService - an independent, self-contained C++ real-time control owner.
//
// It exists to give the hardware bus EXACTLY ONE owner.  Upper layers (Python /
// VLA / planners) never see the backend and never drive a motor frame
// themselves; they hand setpoints and trajectories to the service and keep the
// connection alive with heartbeats.  The service:
//
//   * runs its own periodic control thread at ControlServiceConfig::period_ns,
//   * refreshes joint state every cycle from the backend's atomic snapshot,
//   * enforces a WATCHDOG: if the upper layer goes silent (no heartbeat and no
//     command) for command_timeout_ns while motion is enabled, it latches a
//     fault and brings the arm to a controlled stop - it does NOT assume a
//     power-off / mode-0 will hold a gravity-loaded arm (HARDWARE_UNVERIFIED),
//   * detects stale / missing feedback and motor errors and latches the same way,
//   * latches faults: recovery is a deliberate, human-confirmed clear + re-enable
//     and a timeout NEVER re-enables motion on its own,
//   * buffers trajectories and executes them with a velocity ramp + per-tick
//     safety re-validation,
//   * owns the only object that can arm the backend (it is a friend of
//     MotionControl); it never hands the backend out, not even read-only.
//
// The three commanded states are distinct and named:
//   * software-controlled hold  - active FOC holding position (holds a load)
//   * controlled stop           - decelerate to zero velocity then hold, then
//   * motor disabled            - mode 0 (removes output; NOT a brake)
//
// Everything here is hardware-independent and testable against MockArmBackend
// (a real-hardware-style backend over MockJointBus).  Real-hardware timing and
// the mode-0 "does it actually hold?" question remain HARDWARE_UNVERIFIED until
// brought up per HARDWARE_BRINGUP.md.
// ---------------------------------------------------------------------------

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "r1_robot/backend.hpp"
#include "r1_robot/limits.hpp"
#include "r1_robot/motion_control.hpp"
#include "r1_robot/safety.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

/// The externally-visible service state.
enum class ServiceState {
  kUnconnected,      ///< backend not connected
  kStandby,          ///< connected, motion not enabled (disarmed, safe)
  kEnabling,         ///< enable checks passed, arming pending the first tick
  kTrackingTrajectory, ///< streaming a buffered trajectory setpoint
  kSoftwareHold,     ///< active FOC holding the last position against a load
  kControlledStop,   ///< fault latch engaged: decelerating to zero, then hold
  kMotorDisabled,    ///< mode 0 (output removed; NOT a brake)
};

const char* toString(ServiceState state);
bool stateIsMotionActive(ServiceState state);  ///< hold / tracking

struct ControlServiceConfig {
  SafetyConfig safety{};
  RobotLimits limits = r1DefaultLimits();
  bool use_backend_limits = false;

  std::uint64_t period_ns = 10ULL * 1000ULL * 1000ULL;      ///< 100 Hz
  std::uint64_t command_timeout_ns = 200ULL * 1000ULL * 1000ULL;  ///< watchdog
  std::uint64_t stop_hold_ns = 500ULL * 1000ULL * 1000ULL;  ///< hold after stop
  double stop_vel_tol = 0.05;                                ///< [rad/s]

  /// Gains used for the software-controlled hold and the trajectory tracking.
  double hold_kp = 40.0;
  double hold_kd = 3.0;

  /// Monotonic clock source.  Defaults to r1_robot::nowNs().  Tests inject a
  /// virtual clock (shared with the mock backend) to drive the watchdog and
  /// trajectory timing deterministically with no sleeps.
  std::function<std::uint64_t()> clock;
};

/// One trajectory waypoint (joint positions + a time relative to upload).
struct TrajWaypoint {
  std::uint64_t t_ns = 0;
  JointVector q{};
};

/// Result of an enable attempt.
struct EnableResult {
  bool ok = false;
  std::string error;
};

/// A snapshot of service status for an upper layer to poll.
struct ServiceStatus {
  ServiceState state = ServiceState::kUnconnected;
  bool motion_enabled = false;
  bool fault_latched = false;
  std::string last_fault_code;
  std::uint64_t ticks = 0;
  std::uint64_t last_upper_contact_ns = 0;
};

class ControlService {
 public:
  explicit ControlService(ArmBackendPtr backend,
                          ControlServiceConfig config = ControlServiceConfig{});
  ~ControlService();

  ControlService(const ControlService&) = delete;
  ControlService& operator=(const ControlService&) = delete;

  // -- lifecycle -----------------------------------------------------------
  bool connect(std::string* error);
  void disconnect();
  void start();  ///< spawn the periodic thread (after connect)
  void stop();   ///< stop the thread, controlled-stop, leave disarmed

  // -- motion gating (explicit, human-confirmed) --------------------------
  EnableResult enableMotion(bool human_confirmed);
  void disableMotion();

  // -- upper-layer interface ----------------------------------------------
  /// Heartbeat: proves the upper layer is alive even while idle.
  void heartbeat();
  /// Submit one setpoint frame (position + gains); resets the watchdog.
  void submitCommand(const ArmCommand& cmd);
  /// Buffer a trajectory and start tracking it; resets the watchdog.
  void submitTrajectory(const std::vector<TrajWaypoint>& waypoints);
  /// Request a controlled stop now (does not clear a latched fault).
  void requestControlledStop();

  /// Run exactly ONE control cycle: refresh state, evaluate the watchdog and
  /// feedback safety, latch faults, and command the current setpoint.  The
  /// periodic thread calls this every `period_ns`; tests call it directly with a
  /// virtual clock to advance the watchdog deterministically.
  void runCycle();

  // -- fault handling ------------------------------------------------------
  bool clearFault(std::string* error);  ///< operator: acknowledge + clear latch

  // -- introspection (lock-free / safe to poll) ----------------------------
  ServiceStatus status() const;
  ServiceState state() const;
  RobotState latestState() const;
  bool faultLatched() const;
  std::string lastFaultCode() const;

 private:
  std::uint64_t now() const { return clock_ ? clock_() : nowNs(); }
  void threadMain();
  // Assumes mutex_ held.
  void latchFaultLocked(const std::string& code);
  void beginControlledStopLocked();
  void sendToBackendLocked(const ArmCommand& cmd);
  JointVector rampLocked(const JointVector& target);
  bool trajectoryDoneLocked(std::uint64_t t) const;

  ArmBackendPtr backend_;
  ControlServiceConfig config_;
  SafetyMonitor monitor_;
  MotionControl control_;  // ControlService is a friend of MotionControl.
  std::function<std::uint64_t()> clock_;

  mutable std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};

  ServiceState state_ = ServiceState::kUnconnected;
  bool motion_enabled_ = false;
  bool fault_latched_ = false;
  std::string last_fault_code_;

  RobotState latest_state_{};  // latest atomic snapshot
  std::uint64_t last_upper_contact_ns_ = 0;
  std::uint64_t ticks_ = 0;

  // Last direct setpoint submitted (used when no trajectory is active).
  ArmCommand pending_cmd_{};
  bool has_pending_cmd_ = false;

  // Controlled-stop sequencing.
  std::uint64_t stop_engaged_ns_ = 0;
  JointVector stop_target_{};

  // Trajectory buffer + smoothing.
  std::vector<TrajWaypoint> traj_{};
  std::uint64_t traj_upload_ns_ = 0;
  JointVector last_target_{};  // ramped setpoint actually commanded last tick
  bool has_last_target_ = false;

  bool calibration_ok_ = false;
};

}  // namespace r1_robot
