#include "r1_robot/gripper.hpp"

#include <cmath>
#include <sstream>

#include "r1_robot/types.hpp"

namespace r1_robot {
namespace {

const char* kDm4310Pending = "not implemented in phase 1";

std::string pendingMessage(const char* what) {
  std::ostringstream oss;
  oss << kDm4310Pending << ": " << what
      << ". The DM4310 gripper must be brought up first - see the gripper "
         "section of HARDWARE_BRINGUP.md (CAN interface/bitrate, motor id, MIT "
         "parameter ranges, rack travel, current/velocity limits, homing).";
  return oss.str();
}

}  // namespace

const char* toString(GripperCommandMode mode) {
  switch (mode) {
    case GripperCommandMode::kStop:
      return "stop";
    case GripperCommandMode::kPosition:
      return "position";
    case GripperCommandMode::kCurrent:
      return "current";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// MockGripper
// ---------------------------------------------------------------------------
MockGripper::MockGripper(GripperLimits limits) : limits_(limits) {
  if (limits_.source.empty()) {
    limits_.source = "mock defaults (NOT hardware verified)";
    if (!(limits_.max_current > 0.0)) {
      limits_.max_current = 1.0;
    }
    if (!(limits_.max_velocity > 0.0)) {
      limits_.max_velocity = 1.0;
    }
    if (!(limits_.default_current_limit > 0.0)) {
      limits_.default_current_limit = 0.5;
    }
  }
  state_.position = limits_.position_min;
  state_.connected = false;
  state_.valid = false;
}

bool MockGripper::connect(std::string* error) {
  (void)error;
  connected_ = true;
  state_.connected = true;
  state_.valid = true;
  state_.stamp_ns = nowNs();
  return true;
}

void MockGripper::disconnect() {
  connected_ = false;
  state_.connected = false;
  state_.valid = false;
}

bool MockGripper::isConnected() const { return connected_; }

GripperState MockGripper::state() const { return state_; }

bool MockGripper::writeCommand(const GripperCommand& cmd, std::string* error) {
  auto reject = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!connected_) {
    return reject("refused: gripper is not connected");
  }
  if (!std::isfinite(cmd.position) || !std::isfinite(cmd.current) ||
      !std::isfinite(cmd.velocity_limit) || !std::isfinite(cmd.current_limit)) {
    return reject("refused: command contains NaN/Inf");
  }

  switch (cmd.mode) {
    case GripperCommandMode::kPosition: {
      if (cmd.position < limits_.position_min ||
          cmd.position > limits_.position_max) {
        std::ostringstream oss;
        oss << "refused: position " << cmd.position
            << " outside normalized range [" << limits_.position_min << ", "
            << limits_.position_max << "]";
        return reject(oss.str());
      }
      break;
    }
    case GripperCommandMode::kCurrent: {
      if (std::fabs(cmd.current) > limits_.max_current) {
        std::ostringstream oss;
        oss << "refused: |current| " << std::fabs(cmd.current)
            << " A exceeds max_current=" << limits_.max_current;
        return reject(oss.str());
      }
      break;
    }
    case GripperCommandMode::kStop:
      break;
  }

  if (cmd.velocity_limit > 0.0 && cmd.velocity_limit > limits_.max_velocity) {
    std::ostringstream oss;
    oss << "refused: velocity_limit " << cmd.velocity_limit
        << " exceeds max_velocity=" << limits_.max_velocity;
    return reject(oss.str());
  }
  if (cmd.current_limit > 0.0 && cmd.current_limit > limits_.max_current) {
    std::ostringstream oss;
    oss << "refused: current_limit " << cmd.current_limit
        << " exceeds max_current=" << limits_.max_current;
    return reject(oss.str());
  }

  last_cmd_ = cmd;
  ++command_count_;
  state_.stamp_ns = nowNs();

  if (cmd.mode == GripperCommandMode::kPosition) {
    state_.position = cmd.position;  // ideal servo: instant
    state_.velocity = 0.0;
    state_.current = 0.0;
  } else if (cmd.mode == GripperCommandMode::kStop) {
    state_.velocity = 0.0;
    state_.current = 0.0;
  }
  return true;
}

void MockGripper::stop() {
  std::string ignored;
  writeCommand(GripperCommand{}, &ignored);
}

void MockGripper::setState(const GripperState& state) { state_ = state; }

GripperCommand MockGripper::lastCommand() const { return last_cmd_; }

std::uint64_t MockGripper::commandCount() const { return command_count_; }

// ---------------------------------------------------------------------------
// Dm4310Gripper - fail-closed placeholder
// ---------------------------------------------------------------------------
bool Dm4310Gripper::connect(std::string* error) {
  if (error) {
    *error = pendingMessage("connect");
  }
  return false;
}

void Dm4310Gripper::disconnect() {}

GripperLimits Dm4310Gripper::limits() const {
  GripperLimits limits;
  limits.source =
      "UNVERIFIED placeholder - no value here may be used to drive hardware";
  return limits;
}

GripperState Dm4310Gripper::state() const {
  GripperState s;
  s.connected = false;
  s.valid = false;
  return s;
}

bool Dm4310Gripper::writeCommand(const GripperCommand& cmd, std::string* error) {
  (void)cmd;
  if (error) {
    *error = pendingMessage("writeCommand");
  }
  return false;
}

void Dm4310Gripper::stop() {}

}  // namespace r1_robot
