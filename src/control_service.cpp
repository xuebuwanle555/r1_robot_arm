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
        beginControlledStopLocked();
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
  beginControlledStopLocked();
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

void ControlService::beginControlledStopLocked() {
  // A controlled stop disables "motion enabled" (whether it was reached via a
  // latched fault or an explicit requestControlledStop()); resuming always
  // requires a fresh enableMotion(true).  Keep the gate ARMED so the
  // deceleration/hold frame can still be sent: the arm is caught by active FOC
  // rather than dropped (mode 0 alone is NOT a brake and cannot hold a
  // gravity-loaded arm - HARDWARE_UNVERIFIED).  Only after the arm has stopped
  // do we transition to motor-disabled and disarm.
  motion_enabled_ = false;
  has_pending_cmd_ = false;
  traj_.clear();
  stop_target_ = latest_state_.q;
  stop_engaged_ns_ = now();
  has_last_target_ = true;
  last_target_ = stop_target_;
  state_ = ServiceState::kControlledStop;
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
  const std::uint64_t t = now();
  latest_state_ = backend_->readState();
  ++ticks_;

  // 1) Controlled-stop / motor-disable sequencing.  Runs for a stop reached via
  //    a latched fault OR an explicit requestControlledStop(), independent of the
  //    watchdog.  Decelerate under active FOC, then drop to mode 0 and disarm.
  if (state_ == ServiceState::kControlledStop) {
    const bool settled = [&] {
      if (!latest_state_.connected) {
        return true;  // cannot command a dead port; treat as stopped
      }
      for (int i = 0; i < kNumJoints; ++i) {
        if (std::fabs(latest_state_.dq[static_cast<std::size_t>(i)]) >
            config_.stop_vel_tol) {
          return false;
        }
      }
      return true;
    }();
    const bool held = (t - stop_engaged_ns_) >= config_.stop_hold_ns;
    if (settled && held) {
      backend_->setMotionArmed(false, control_);  // close the write path
      backend_->stop();                           // mode 0; NOT a brake
      state_ = ServiceState::kMotorDisabled;
    } else {
      ArmCommand hold =
          ArmCommand::position(stop_target_, config_.hold_kp, config_.hold_kd);
      sendToBackendLocked(hold);
    }
    return;
  }
  if (state_ == ServiceState::kMotorDisabled) {
    return;  // terminal until an operator clears the fault and re-enables
  }

  // 2) A latched fault that is not yet in the controlled stop routes into it on
  //    the next tick.  It can never leave this state on its own.
  if (fault_latched_) {
    beginControlledStopLocked();
    return;
  }

  // 3) Motion enabled: upper-layer watchdog + feedback safety.
  if (motion_enabled_) {
    const bool upper_alive = (t - last_upper_contact_ns_) <= config_.command_timeout_ns;
    if (!upper_alive) {
      latchFaultLocked("comm.watchdog_timeout");
      beginControlledStopLocked();
      return;
    }
    if (!latest_state_.connected) {
      latchFaultLocked("state.not_connected");
      beginControlledStopLocked();
      return;
    }
    const SafetyReport rep = monitor_.checkState(latest_state_, t);
    if (!rep.ok()) {
      const std::string code =
          rep.faultCodes().empty() ? std::string("state.unsafe")
                                   : rep.faultCodes().front();
      latchFaultLocked(code);
      beginControlledStopLocked();
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
      beginControlledStopLocked();
      return;
    }
    if (backend_->isRealHardware() && !backend_->motionArmed()) {
      latchFaultLocked("internal.disarmed_while_enabled");
      beginControlledStopLocked();
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
