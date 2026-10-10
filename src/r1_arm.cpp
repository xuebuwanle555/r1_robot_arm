#include "r1_robot/r1_arm.hpp"

#include <sstream>
#include <utility>

namespace r1_robot {

R1Arm::R1Arm(ArmBackendPtr backend, R1ArmConfig config)
    : backend_(std::move(backend)),
      config_(std::move(config)),
      monitor_(config_.safety) {
  status_.backend_name = backend_ ? backend_->name() : "none";
  status_.real_hardware = backend_ && backend_->isRealHardware();
  if (config_.use_backend_limits && backend_) {
    config_.limits = backend_->limits();
  }
  // Fail-closed: a real backend is never armed before connect()/enableMotion().
  if (backend_) {
    backend_->setMotionArmed(false, control_);
  }
}

R1Arm::~R1Arm() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Fail-safe teardown: disarm and tell the backend to stop before closing.
  // NOTE: stop() removes motor output; it cannot hold the arm against gravity.
  if (backend_) {
    backend_->setMotionArmed(false, control_);
    if (status_.connected) {
      backend_->stop();
      backend_->disconnect();
    }
  }
  motion_enabled_ = false;
  status_.motion_enabled = false;
  status_.connected = false;
}

bool R1Arm::connect(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto fail = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!backend_) {
    return fail("refused: no backend attached");
  }
  if (status_.connected) {
    return true;
  }

  if (backend_->isRealHardware() && !config_.safety.allow_real_hardware) {
    std::ostringstream oss;
    oss << "refused: backend '" << backend_->name()
        << "' drives real hardware but SafetyConfig::allow_real_hardware is "
           "false (see HARDWARE_BRINGUP.md)";
    return fail(oss.str());
  }

  // A backend is always opened disarmed, whatever its previous state was.
  backend_->setMotionArmed(false, control_);

  std::string backend_error;
  if (!backend_->connect(&backend_error)) {
    return fail("backend connect failed: " + backend_error);
  }

  if (config_.use_backend_limits) {
    config_.limits = backend_->limits();
  }

  std::string calib_detail;
  status_.calibration_ok = backend_->hasValidCalibration(&calib_detail);

  if (backend_->isRealHardware() && config_.safety.require_calibration &&
      !status_.calibration_ok) {
    backend_->disconnect();
    return fail("refused: real hardware without a valid joint calibration (" +
                calib_detail + ")");
  }

  status_.connected = true;
  status_.real_hardware = backend_->isRealHardware();
  status_.backend_name = backend_->name();
  status_.motion_enabled = false;
  motion_enabled_ = false;
  refreshLocked();
  return true;
}

void R1Arm::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_enabled_ = false;
  status_.motion_enabled = false;
  if (backend_) {
    backend_->setMotionArmed(false, control_);
    if (status_.connected) {
      backend_->stop();
      backend_->disconnect();
    }
  }
  status_.connected = false;
  status_.calibration_ok = false;
  state_ = RobotState{};
}

bool R1Arm::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_.connected;
}

R1ArmStatus R1Arm::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

std::string R1Arm::describe() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "R1Arm{backend=" << status_.backend_name
      << ", real_hardware=" << (status_.real_hardware ? "yes" : "no")
      << ", connected=" << (status_.connected ? "yes" : "no")
      << ", calibration=" << (status_.calibration_ok ? "ok" : "MISSING")
      << ", motion_enabled=" << (motion_enabled_ ? "yes" : "no")
      << ", allow_real_hardware="
      << (config_.safety.allow_real_hardware ? "true" : "false") << "}";
  return oss.str();
}

RobotState R1Arm::state(bool refresh) {
  std::lock_guard<std::mutex> lock(mutex_);
  return stateLocked(refresh);
}

RobotState R1Arm::stateLocked(bool refresh) {
  if (refresh && status_.connected) {
    refreshLocked();
  }
  return state_;
}

void R1Arm::refreshLocked() {
  if (backend_ && status_.connected) {
    state_ = backend_->readState();
  }
}

SafetyReport R1Arm::checkState(std::uint64_t now_ns) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return checkStateLocked(now_ns);
}

SafetyReport R1Arm::checkStateLocked(std::uint64_t now_ns) const {
  return monitor_.checkState(state_, now_ns);
}

bool R1Arm::enableMotion(bool human_confirmed, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto fail = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!human_confirmed) {
    return fail(
        "refused: enabling motion requires an explicit human confirmation "
        "(enableMotion(true))");
  }
  if (!status_.connected) {
    return fail("refused: not connected");
  }
  if (backend_->isRealHardware() && !config_.safety.allow_real_hardware) {
    return fail(
        "refused: real hardware motion requires "
        "SafetyConfig::allow_real_hardware = true");
  }

  refreshLocked();
  std::string calib_detail;
  const bool calib_ok = backend_->hasValidCalibration(&calib_detail);

  const SafetyReport report =
      monitor_.checkMotionEnable(state_, nowNs(), calib_ok);
  if (!report.ok()) {
    return fail("refused: motion enable failed safety checks: " +
                report.toString());
  }

  status_.calibration_ok = calib_ok;
  motion_enabled_ = true;
  status_.motion_enabled = true;
  // Defense in depth: arm the backend only now that every gate above passed.
  backend_->setMotionArmed(true, control_);
  return true;
}

void R1Arm::disableMotion() {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_enabled_ = false;
  status_.motion_enabled = false;
  if (backend_) {
    backend_->setMotionArmed(false, control_);
    if (status_.connected) {
      backend_->stop();
    }
  }
  refreshLocked();
}

bool R1Arm::motionEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_enabled_;
}

bool R1Arm::sendAction(const ArmCommand& cmd, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto fail = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!status_.connected) {
    return fail("refused: not connected");
  }
  if (!cmd.allFinite()) {
    return fail("refused: command contains NaN/Inf");
  }

  // Stop commands can only remove energy and are always accepted, so that a
  // fault can never block the way back to a safe state.
  if (!cmd.hasActuation()) {
    const bool sent = backend_->writeCommand(cmd);
    refreshLocked();
    return sent ? true : fail("backend rejected the stop command");
  }

  if (!motion_enabled_) {
    return fail(
        "refused: motion is not enabled (call enableMotion(true) after the "
        "bring-up checks)");
  }
  if (backend_->isRealHardware() && !config_.safety.allow_real_hardware) {
    return fail("refused: real hardware motion requires allow_real_hardware");
  }
  // Defense in depth: even if the flag above is somehow set, the backend must
  // be armed before it accepts an actuating frame.
  if (!backend_->motionArmed()) {
    return fail(
        "refused: the backend is not armed for motion (internal state error)");
  }

  refreshLocked();
  const SafetyReport report = monitor_.checkCommand(
      cmd, state_, config_.limits, nowNs(), status_.calibration_ok);
  if (!report.ok()) {
    return fail("refused: command rejected by safety checks: " +
                report.toString());
  }

  if (!backend_->writeCommand(cmd)) {
    return fail("backend rejected the command");
  }
  refreshLocked();
  return true;
}

void R1Arm::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (backend_ && status_.connected) {
    // Deliberately does NOT disarm: stop() is a mode-0 command that may be
    // used between actuating frames; disableMotion()/emergencyStop() are the
    // calls that close the command path.
    backend_->stop();
    refreshLocked();
  }
}

void R1Arm::emergencyStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_enabled_ = false;
  status_.motion_enabled = false;
  if (backend_) {
    backend_->setMotionArmed(false, control_);
    if (status_.connected) {
      backend_->stop();
    }
  }
  refreshLocked();
}

bool R1Arm::probeFeedbackForBringup(bool human_confirmed, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto fail = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!human_confirmed) {
    return fail(
        "refused: the feedback probe powers the motor bridges (the arm goes "
        "compliant) and requires human_confirmed = true");
  }
  if (!status_.connected) {
    return fail("refused: not connected");
  }
  if (backend_->isRealHardware() && !config_.safety.allow_real_hardware) {
    return fail(
        "refused: probing a real backend requires allow_real_hardware = true");
  }
  return backend_->probeFeedback(error, control_);
}

bool R1Arm::home(std::string* error) {
  if (error) {
    *error =
        "blocked: homing is intentionally not exposed in phase 1. "
        "The official SDK homes by creeping toward q=0 with a limited lead; "
        "that behaviour must first be brought up and verified on the real arm "
        "(see HARDWARE_BRINGUP.md).";
  }
  return false;
}

RobotLimits R1Arm::limits() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.limits;
}

SafetyConfig R1Arm::safetyConfig() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.safety;
}

std::shared_ptr<const IArmBackend> R1Arm::backend() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // Return a const view: only read-only methods (readState/limits/isConnected/
  // motionArmed/...) are callable, so no caller can actuate or arm through it.
  return std::static_pointer_cast<const IArmBackend>(backend_);
}

void R1Arm::setLimits(const RobotLimits& limits) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.limits = limits;
}

}  // namespace r1_robot
