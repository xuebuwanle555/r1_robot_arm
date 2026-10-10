#include "r1_robot/control_service.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace r1_robot {
namespace {

double minJerkS(double tau) {
  if (tau <= 0.0) return 0.0;
  if (tau >= 1.0) return 1.0;
  return 10 * tau * tau * tau - 15 * tau * tau * tau * tau +
         6 * tau * tau * tau * tau * tau;
}

}  // namespace

const char* toString(ServiceState state) {
  switch (state) {
    case ServiceState::kUnconnected:
      return "unconnected";
    case ServiceState::kStandby:
      return "standby";
    case ServiceState::kEnabling:
      return "enabling";
    case ServiceState::kTrackingTrajectory:
      return "tracking_trajectory";
    case ServiceState::kSoftwareHold:
      return "software_hold";
    case ServiceState::kControlledStop:
      return "controlled_stop";
    case ServiceState::kMotorDisabled:
      return "motor_disabled";
  }
  return "unknown";
}

bool stateIsMotionActive(ServiceState state) {
  return state == ServiceState::kTrackingTrajectory ||
         state == ServiceState::kSoftwareHold || state == ServiceState::kEnabling;
}

const char* toString(StopReason reason) {
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kOperatorStop:
      return "operator_stop";
    case StopReason::kUpperLayerTimeout:
      return "upper_layer_timeout";
    case StopReason::kFeedbackLost:
      return "feedback_lost";
    case StopReason::kActuatorFault:
      return "actuator_fault";
    case StopReason::kUnsafeState:
      return "unsafe_state";
  }
  return "unknown";
}

StopReason ControlService::stopReasonForFault(const std::string& code) {
  if (code.rfind("comm.", 0) == 0) {
    return StopReason::kUpperLayerTimeout;
  }
  if (code == "state.not_connected" || code == "state.stale_feedback" ||
      code == "state.joint_invalid" || code == "state.no_feedback") {
    return StopReason::kFeedbackLost;
  }
  if (code == "state.motor_error") {
    return StopReason::kActuatorFault;
  }
  return StopReason::kUnsafeState;
}

bool ControlService::feedbackTrustedLocked(std::uint64_t t) const {
  if (!latest_state_.connected || !latest_state_.allValid()) {
    return false;  // no link, or a joint never reported a valid frame
  }
  // Reuse the safety monitor so "trustworthy feedback" means exactly one thing
  // in this file: connected + per-joint valid + within max_feedback_age_s.
  return monitor_.checkState(latest_state_, t).ok();
}

ControlService::ControlService(ArmBackendPtr backend, ControlServiceConfig config)
    : backend_(std::move(backend)),
      config_(std::move(config)),
      monitor_(config_.safety),
      clock_(config_.clock) {
  if (config_.use_backend_limits && backend_) {
    config_.limits = backend_->limits();
  }
  if (backend_) {
    backend_->setMotionArmed(false, control_);  // fail-closed
  }
}

ControlService::~ControlService() { stop(); }

bool ControlService::connect(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!backend_) {
    if (error) {
      *error = "no backend";
    }
    return false;
  }
  if (state_ != ServiceState::kUnconnected) {
    return true;
  }
  backend_->setMotionArmed(false, control_);
  std::string err;
  if (!backend_->connect(&err)) {
    if (error) {
      *error = err;
    }
    return false;
  }
  std::string calib_detail;
  const bool calib_ok = backend_->hasValidCalibration(&calib_detail);
  calibration_ok_ = calib_ok;
  if (backend_->isRealHardware() && config_.safety.require_calibration &&
      !calib_ok) {
    backend_->disconnect();
    if (error) {
      *error = "refused: real hardware without a valid calibration (" +
               calib_detail + ")";
    }
    return false;
  }
  state_ = ServiceState::kStandby;
  latest_state_ = backend_->readState();
  return true;
}

void ControlService::disconnect() {
  stop();
  std::lock_guard<std::mutex> lock(mutex_);
  if (backend_ && state_ != ServiceState::kUnconnected) {
    backend_->setMotionArmed(false, control_);
    backend_->stop();
    backend_->disconnect();
  }
  state_ = ServiceState::kUnconnected;
  motion_enabled_ = false;
  latest_state_ = RobotState{};
}

void ControlService::start() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_.load() || state_ == ServiceState::kUnconnected) {
      return;
    }
  }
  running_.store(true);
  worker_ = std::thread([this] { threadMain(); });
}

void ControlService::stop() {
  running_.store(false);
  if (worker_.joinable()) {
    worker_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ServiceState::kUnconnected) {
    return;
  }
  motion_enabled_ = false;
  backend_->setMotionArmed(false, control_);
  backend_->stop();
  state_ = ServiceState::kStandby;
}

EnableResult ControlService::enableMotion(bool human_confirmed) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnableResult r;
  if (!backend_ || state_ == ServiceState::kUnconnected) {
    r.error = "refused: not connected";
    return r;
  }
  if (fault_latched_) {
    r.error = "refused: a fault is latched; clear it before enabling motion";
    return r;
  }
  if (!human_confirmed) {
    r.error = "refused: enabling motion requires an explicit human confirmation";
    return r;
  }
  if (backend_->isRealHardware() && !config_.safety.allow_real_hardware) {
    r.error = "refused: real hardware motion requires allow_real_hardware";
    return r;
  }
  latest_state_ = backend_->readState();
  std::string calib_detail;
  const bool calib_ok = backend_->hasValidCalibration(&calib_detail);
  calibration_ok_ = calib_ok;
  const SafetyReport report =
      monitor_.checkMotionEnable(latest_state_, now(), calib_ok);
  if (!report.ok()) {
    r.error = "refused: motion enable failed safety checks: " + report.toString();
    return r;
  }
  motion_enabled_ = true;
  has_pending_cmd_ = false;
  traj_.clear();
  last_upper_contact_ns_ = now();
  backend_->setMotionArmed(true, control_);  // the single owner arms here
  state_ = ServiceState::kSoftwareHold;
  r.ok = true;
  return r;
}

void ControlService::disableMotion() {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_enabled_ = false;
  has_pending_cmd_ = false;
  traj_.clear();
  if (backend_ && state_ != ServiceState::kUnconnected) {
    backend_->setMotionArmed(false, control_);
    backend_->stop();
    if (!fault_latched_) {
      state_ = ServiceState::kStandby;
    }
  }
}

void ControlService::heartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_upper_contact_ns_ = now();
}

void ControlService::submitCommand(const ArmCommand& cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fault_latched_ || !motion_enabled_) {
    return;  // refused while latched / not enabled; a stop may still be issued
  }
  pending_cmd_ = cmd;
  has_pending_cmd_ = true;
  traj_.clear();
  last_upper_contact_ns_ = now();
  if (state_ == ServiceState::kEnabling) {
    state_ = ServiceState::kTrackingTrajectory;
  }
}

void ControlService::submitTrajectory(const std::vector<TrajWaypoint>& waypoints) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fault_latched_ || !motion_enabled_ || waypoints.size() < 2) {
    return;
  }
  // Up-front envelope check: an out-of-limits destination must be refused now,
  // not crept toward one ramped step at a time.  Per-cycle validation of the
  // ramped frame would otherwise mask the violation (each step is in-limit even
  // though the goal is not).
  for (const TrajWaypoint& wp : waypoints) {
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      const JointLimits& lim = config_.limits.joints[si];
      const double centre = 0.5 * (lim.lower + lim.upper);
      const double lo = std::min(lim.lower + config_.safety.soft_limit_margin, centre);
      const double hi = std::max(lim.upper - config_.safety.soft_limit_margin, centre);
      if (wp.q[si] < lo || wp.q[si] > hi) {
        latchFaultLocked("cmd.out_of_limits");
        beginControlledStopLocked(StopReason::kUnsafeState);
        return;
      }
    }
  }
  traj_ = waypoints;
  has_pending_cmd_ = false;
  traj_upload_ns_ = now();
  last_upper_contact_ns_ = traj_upload_ns_;
  last_target_ = waypoints.front().q;
  has_last_target_ = true;
  state_ = ServiceState::kTrackingTrajectory;
}

void ControlService::requestControlledStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ServiceState::kUnconnected ||
      state_ == ServiceState::kMotorDisabled ||
      state_ == ServiceState::kControlledStop) {
    return;
  }
  beginControlledStopLocked(StopReason::kOperatorStop);
}

bool ControlService::clearFault(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!backend_ || state_ == ServiceState::kUnconnected) {
    if (error) {
      *error = "refused: not connected";
    }
    return false;
  }
  if (!fault_latched_) {
    return true;
  }
  // An operator acknowledges the fault.  The latch is cleared and the backend
  // stays DISARMED: clearing is not the same as re-enabling motion.  A timeout
  // can never auto-clear - this call is the only way out, and it still requires
  // a fresh enableMotion(true) afterwards.
  fault_latched_ = false;
  last_fault_code_.clear();
  motion_enabled_ = false;
  has_pending_cmd_ = false;
  traj_.clear();
  backend_->setMotionArmed(false, control_);
  state_ = ServiceState::kStandby;
  return true;
}

void ControlService::latchFaultLocked(const std::string& code) {
  if (!fault_latched_) {
    fault_latched_ = true;
    last_fault_code_ = code;
  }
  motion_enabled_ = false;
  has_pending_cmd_ = false;
  traj_.clear();
}

void ControlService::beginControlledStopLocked(StopReason reason) {
  // A controlled stop disables "motion enabled" (whether it was reached via a
  // latched fault or an explicit requestControlledStop()); resuming always
  // requires a fresh enableMotion(true).  Keep the gate ARMED so the
  // deceleration/hold frame can still be sent: the arm is caught by active FOC
  // rather than dropped (mode 0 alone is NOT a brake and cannot hold a
  // gravity-loaded arm - HARDWARE_UNVERIFIED).
  //
  // The hold target is the setpoint we were *commanding*, not the position
  // feedback happens to report: when the stop was triggered precisely because
  // feedback went bad, the measured value is stale and re-aiming at it would be
  // a backward jump built out of the very data we stopped trusting.
  motion_enabled_ = false;
  has_pending_cmd_ = false;
  traj_.clear();

  stop_target_ = has_last_target_ ? last_target_ : latest_state_.q;
  stop_engaged_ns_ = now();
  stop_reason_ = reason;
  stop_still_since_ns_ = 0;
  stop_verified_ = false;
  has_last_target_ = true;
  last_target_ = stop_target_;
  state_ = ServiceState::kControlledStop;
}

bool ControlService::confirmMotorDisable(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!backend_ || state_ == ServiceState::kUnconnected) {
    if (error) {
      *error = "refused: not connected";
    }
    return false;
  }
  if (state_ == ServiceState::kMotorDisabled) {
    return true;  // already the requested end state
  }
  const std::uint64_t t = now();
  if (!feedbackTrustedLocked(t)) {
    if (error) {
      *error =
          "refused: feedback is not currently valid, so stillness cannot be "
          "proven; the drive stays engaged (a stop with lost feedback is not a "
          "verified stop)";
    }
    return false;
  }
  if (!stop_verified_) {
    if (error) {
      *error = "refused: joints have not been observed still for the full " +
               std::to_string(config_.stop_settle_ns / 1000000ULL) +
               " ms settle window";
    }
    return false;
  }
  // Operator-confirmed drive removal.  This is still NOT a brake.
  backend_->setMotionArmed(false, control_);
  backend_->stop();  // mode 0
  state_ = ServiceState::kMotorDisabled;
  motion_enabled_ = false;
  if (error) {
    error->clear();
  }
  return true;
}

void ControlService::sendToBackendLocked(const ArmCommand& cmd) {
  backend_->writeCommand(cmd);
}

JointVector ControlService::rampLocked(const JointVector& target) {
  JointVector out;
  const double step = config_.safety.max_joint_step_rad;
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    double cur = has_last_target_ ? last_target_[si] : latest_state_.q[si];
    double d = target[si] - cur;
    if (d > step) d = step;
    if (d < -step) d = -step;
    out[si] = cur + d;
  }
  return out;
}

bool ControlService::trajectoryDoneLocked(std::uint64_t t) const {
  if (traj_.size() < 2) return true;
  return (t - traj_upload_ns_) >= traj_.back().t_ns;
}

void ControlService::runCycle() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!backend_ || state_ == ServiceState::kUnconnected) {
    return;
  }
  const std::uint64_t t = [&] {
    // Refresh feedback FIRST, then take the reference time, so a backend that
    // stamps its state with its own clock reading (taken moments before `t`)
    // can never appear "in the future" relative to the safety check.
    latest_state_ = backend_->readState();
    return now();
  }();
  ++ticks_;

  // 1) Controlled-stop sequencing.  Runs for a stop reached via a latched fault
  //    OR an explicit requestControlledStop(), independent of the watchdog.
  //
  //    The stop holds under ACTIVE FOC and stays there.  Nothing in this branch
  //    removes the drive: zero velocity plus an elapsed timer is not proof that
  //    de-energising is safe, and lost feedback is even less so.  Only the
  //    operator's confirmMotorDisable() may cross into kMotorDisabled.
  if (state_ == ServiceState::kControlledStop) {
    const bool trusted = feedbackTrustedLocked(t);
    bool still_now = trusted;
    if (still_now) {
      for (int i = 0; i < kNumJoints; ++i) {
        if (std::fabs(latest_state_.dq[static_cast<std::size_t>(i)]) >
            config_.stop_vel_tol) {
          still_now = false;
          break;
        }
      }
    }
    if (still_now) {
      if (stop_still_since_ns_ == 0) {
        stop_still_since_ns_ = t;
      }
      if ((t - stop_still_since_ns_) >= config_.stop_settle_ns) {
        stop_verified_ = true;  // settle window observed without a break
      }
    } else {
      // Lost feedback, or a joint moving: the settle run restarts and any prior
      // verification is withdrawn.  We deliberately do NOT keep a stale
      // "verified" flag, because that would let an operator drop the drive on
      // evidence from a moment we can no longer see.
      stop_still_since_ns_ = 0;
      stop_verified_ = false;
    }

    ArmCommand hold =
        ArmCommand::position(stop_target_, config_.hold_kp, config_.hold_kd);
    sendToBackendLocked(hold);
    return;
  }
  if (state_ == ServiceState::kMotorDisabled) {
    return;  // terminal until an operator clears the fault and re-enables
  }

  // 2) A latched fault that is not yet in the controlled stop routes into it on
  //    the next tick.  It can never leave this state on its own.
  if (fault_latched_) {
    beginControlledStopLocked(stopReasonForFault(last_fault_code_));
    return;
  }

  // 3) Motion enabled: upper-layer watchdog + feedback safety.
  if (motion_enabled_) {
    const bool upper_alive = (t - last_upper_contact_ns_) <= config_.command_timeout_ns;
    if (!upper_alive) {
      latchFaultLocked("comm.watchdog_timeout");
      beginControlledStopLocked(StopReason::kUpperLayerTimeout);
      return;
    }
    if (!latest_state_.connected) {
      latchFaultLocked("state.not_connected");
      beginControlledStopLocked(StopReason::kFeedbackLost);
      return;
    }
    const SafetyReport rep = monitor_.checkState(latest_state_, t);
    if (!rep.ok()) {
      const std::string code =
          rep.faultCodes().empty() ? std::string("state.unsafe")
                                   : rep.faultCodes().front();
      latchFaultLocked(code);
      beginControlledStopLocked(stopReasonForFault(code));
      return;
    }
  } else {
    if (backend_->isRealHardware() && backend_->motionArmed()) {
      backend_->setMotionArmed(false, control_);  // safety net: never armed idle
    }
    state_ = ServiceState::kStandby;
    return;
  }

  // 4) Motion enabled, upper layer alive, state healthy: command the setpoint.
  ArmCommand target_cmd;
  if (!traj_.empty()) {
    const std::uint64_t rel = t - traj_upload_ns_;
    std::size_t seg = traj_.size() - 2;
    for (std::size_t i = 0; i + 1 < traj_.size(); ++i) {
      if (rel < traj_[i + 1].t_ns) {
        seg = i;
        break;
      }
    }
    const TrajWaypoint& a = traj_[seg];
    const TrajWaypoint& b = traj_[seg + 1];
    double tau = 1.0;
    if (b.t_ns > a.t_ns) {
      tau = static_cast<double>(rel - a.t_ns) / static_cast<double>(b.t_ns - a.t_ns);
    }
    if (tau < 0.0) tau = 0.0;
    if (tau > 1.0) tau = 1.0;
    const double s = minJerkS(tau);
    JointVector interp{};
    for (int j = 0; j < kNumJoints; ++j) {
      const std::size_t sj = static_cast<std::size_t>(j);
      interp[sj] = a.q[sj] + (b.q[sj] - a.q[sj]) * s;
    }
    if (trajectoryDoneLocked(t)) {
      traj_.clear();
    }
    target_cmd = ArmCommand::position(interp, config_.hold_kp, config_.hold_kd);
  } else if (has_pending_cmd_) {
    target_cmd = pending_cmd_;
  } else {
    target_cmd = ArmCommand::position(latest_state_.q, config_.hold_kp,
                                      config_.hold_kd);
    state_ = ServiceState::kSoftwareHold;
  }

  // Ramp the commanded position so it never steps more than
  // max_joint_step_rad from the last commanded target (anti-jump smoothing).
  JointVector target_q{};
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    target_q[si] = target_cmd.joints[si].q;
  }
  const JointVector ramped = rampLocked(target_q);
  ArmCommand to_send = target_cmd;
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    to_send.joints[si].q = ramped[si];
  }
  last_target_ = ramped;
  has_last_target_ = true;

  if (to_send.hasActuation()) {
    const SafetyReport cmd_rep = monitor_.checkCommand(
        to_send, latest_state_, config_.limits, t, calibration_ok_);
    if (!cmd_rep.ok()) {
      const std::string code = cmd_rep.faultCodes().empty()
                                   ? std::string("cmd.unsafe")
                                   : cmd_rep.faultCodes().front();
      latchFaultLocked(code);
      beginControlledStopLocked(stopReasonForFault(code));
      return;
    }
    if (backend_->isRealHardware() && !backend_->motionArmed()) {
      latchFaultLocked("internal.disarmed_while_enabled");
      beginControlledStopLocked(StopReason::kUnsafeState);
      return;
    }
    if (state_ != ServiceState::kTrackingTrajectory) {
      state_ = ServiceState::kSoftwareHold;
    }
  }

  sendToBackendLocked(to_send);
}

void ControlService::threadMain() {
  auto period = std::chrono::nanoseconds(config_.period_ns);
  while (running_.load()) {
    runCycle();
    std::this_thread::sleep_for(period);
  }
}

ServiceStatus ControlService::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ServiceStatus s;
  s.state = state_;
  s.motion_enabled = motion_enabled_;
  s.fault_latched = fault_latched_;
  s.last_fault_code = last_fault_code_;
  s.ticks = ticks_;
  s.last_upper_contact_ns = last_upper_contact_ns_;
  return s;
}

ServiceState ControlService::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

StopReason ControlService::stopReason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ == ServiceState::kControlledStop ? stop_reason_
                                                  : StopReason::kNone;
}

bool ControlService::stopHoldVerified() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_verified_;
}

RobotState ControlService::latestState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_state_;
}

bool ControlService::faultLatched() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_latched_;
}

std::string ControlService::lastFaultCode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_fault_code_;
}

}  // namespace r1_robot
