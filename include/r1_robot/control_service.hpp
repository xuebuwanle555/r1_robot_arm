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
//   * controlled stop           - decelerate to zero velocity and KEEP holding
//                                 under active FOC, indefinitely
//   * motor disabled            - mode 0 (removes output; NOT a brake)
//
// Moving from a controlled stop to motor-disabled is NEVER automatic.  Zero
// measured velocity and an elapsed hold window are not sufficient: the drive is
// only removed on an explicit operator confirmation (`confirmMotorDisable`),
// and only while feedback is valid and every joint has been observed still for
// `stop_settle_ns`.  Losing feedback is a reason to keep holding, not a reason
// to conclude the arm has stopped.
//
// Even a granted confirmation does not make mode 0 safe to fall into: whether
// the R1 holds a gravity-loaded arm without torque is HARDWARE_UNVERIFIED.
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
  kControlledStop,   ///< decelerating / holding under active FOC
  kMotorDisabled,    ///< mode 0 (output removed; NOT a brake)
};

/// WHY the service is in the controlled stop.  These are deliberately distinct:
/// "the arm stopped" and "we lost the ability to tell" are not the same fact and
/// must never be conflated when deciding whether the drive may be removed.
enum class StopReason {
  kNone,               ///< not in a controlled stop
  kOperatorStop,       ///< an operator asked for a stop; nothing is wrong
  kUpperLayerTimeout,  ///< the heartbeat went silent
  kFeedbackLost,       ///< transport down / feedback stale or invalid: stillness
                         ///< cannot be proven, so the drive must stay engaged
  kActuatorFault,      ///< a motor reported an error
  kUnsafeState,        ///< envelope / safety monitor violation
};

const char* toString(ServiceState state);
const char* toString(StopReason reason);
bool stateIsMotionActive(ServiceState state);  ///< hold / tracking

struct ControlServiceConfig {
  SafetyConfig safety{};
  RobotLimits limits = r1DefaultLimits();
  bool use_backend_limits = false;

  std::uint64_t period_ns = 10ULL * 1000ULL * 1000ULL;      ///< 100 Hz
  std::uint64_t command_timeout_ns = 200ULL * 1000ULL * 1000ULL;  ///< watchdog

  /// How long feedback must CONTINUOUSLY show every joint inside
  /// `stop_vel_tol` before an operator may confirm drive removal.  This is a
  /// precondition on a human decision, not a timer that discharges the drive by
  /// itself: elapsed time alone is never evidence that de-energising is safe.
  std::uint64_t stop_settle_ns = 500ULL * 1000ULL * 1000ULL;
  double stop_vel_tol = 0.05;               ///< [rad/s]

  /// Gains used for the software-controlled hold and the trajectory tracking.
  double hold_kp = 40.0;
  double hold_kd = 3.0;

  // -- trajectory admission ------------------------------------------------
  // These bound what may be uploaded at all, so an unsafe trajectory is refused
  // up front instead of being executed and then clamped frame by frame.
  std::size_t max_traj_points = 512;
  std::uint64_t max_traj_duration_ns = 60ULL * 1000ULL * 1000ULL * 1000ULL;

  /// Peak joint acceleration a uploaded trajectory may ask for [rad/s^2].
  /// TODO(bringup) HARDWARE_UNVERIFIED: not measured on an R1.
  double max_joint_accel_rad_s2 = 5.0;

  /// Upper bound on the synthesized lead-in that brings the arm from its
  /// measured pose onto the first waypoint.  A start offset that needs more
  /// than this is refused rather than silently crept toward.
  std::uint64_t traj_lead_in_max_ns = 5ULL * 1000ULL * 1000ULL * 1000ULL;

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

/// Whether a submission was taken.  A rejection is NOT a fault: nothing was
/// executed, the service keeps holding, and the caller gets a reason back.
enum class SubmitStatus { kAccepted, kRejected };

struct SubmitResult {
  SubmitStatus status = SubmitStatus::kRejected;
  std::string error;
  /// Waypoint index the rejection refers to, -1 when it is not per-waypoint.
  int waypoint_index = -1;

  bool ok() const { return status == SubmitStatus::kAccepted; }
  operator bool() const { return ok(); }

  static SubmitResult accepted() {
    SubmitResult r;
    r.status = SubmitStatus::kAccepted;
    return r;
  }
  static SubmitResult reject(const std::string& reason, int index = -1) {
    SubmitResult r;
    r.status = SubmitStatus::kRejected;
    r.error = reason;
    r.waypoint_index = index;
    return r;
  }
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
  /// Refused (with a reason) unless motion is enabled, no fault is latched and
  /// the target is admissible from the current, valid feedback.
  SubmitResult submitCommand(const ArmCommand& cmd);
  /// Buffer a trajectory and start tracking it; resets the watchdog.
  /// Fully validated before anything is stored; see validateTrajectoryLocked.
  SubmitResult submitTrajectory(const std::vector<TrajWaypoint>& waypoints);
  /// Request a controlled stop now (does not clear a latched fault).
  void requestControlledStop();

  /// Operator-only confirmation that the drive may be removed (mode 0).
  /// Refused unless the service is holding, feedback is currently valid, and
  /// every joint has been observed still for at least `stop_settle_ns`.
  /// Even when granted this is NOT a brake: whether mode 0 holds a
  /// gravity-loaded R1 is `HARDWARE_UNVERIFIED`.
  bool confirmMotorDisable(std::string* error);

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
  StopReason stopReason() const;
  /// True only when feedback has *continuously* shown every joint still for
  /// `stop_settle_ns`.  False whenever feedback is missing or a joint is
  /// moving: absence of evidence is never evidence of absence.
  bool stopHoldVerified() const;
  RobotState latestState() const;
  bool faultLatched() const;
  std::string lastFaultCode() const;

 private:
  std::uint64_t now() const { return clock_ ? clock_() : nowNs(); }
  void threadMain();
  // Assumes mutex_ held.
  void latchFaultLocked(const std::string& code);
  void beginControlledStopLocked(StopReason reason);
  void sendToBackendLocked(const ArmCommand& cmd);
  JointVector rampLocked(const JointVector& target);
  bool trajectoryDoneLocked(std::uint64_t t) const;
  /// Feedback we would be willing to base a decision on: connected, every joint
  /// valid and fresh.  Assumes mutex_ held.
  bool feedbackTrustedLocked(std::uint64_t t) const;
  static StopReason stopReasonForFault(const std::string& code);
  /// Full admission check for an uploaded trajectory.  On success `out` holds
  /// the buffered copy, including any synthesized lead-in segment.
  SubmitResult validateTrajectoryLocked(const std::vector<TrajWaypoint>& in,
                                        std::uint64_t t,
                                        std::vector<TrajWaypoint>* out) const;

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
  StopReason stop_reason_ = StopReason::kNone;
  std::uint64_t stop_engaged_ns_ = 0;
  JointVector stop_target_{};       // the setpoint the FOC hold commands
  /// Start of the current unbroken run of "valid feedback + every joint still".
  /// 0 means "not currently observed still".
  std::uint64_t stop_still_since_ns_ = 0;
  bool stop_verified_ = false;

  // Trajectory buffer + smoothing.
  std::vector<TrajWaypoint> traj_{};
  std::uint64_t traj_upload_ns_ = 0;
  JointVector last_target_{};  // ramped setpoint actually commanded last tick
  bool has_last_target_ = false;

  bool calibration_ok_ = false;
};

}  // namespace r1_robot
