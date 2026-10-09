#include "r1_robot/types.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

namespace r1_robot {
namespace {

bool finiteVec(const JointVector& v) {
  for (double x : v) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  return true;
}

}  // namespace

const char* toString(ControlMode mode) {
  switch (mode) {
    case ControlMode::kStop:
      return "stop";
    case ControlMode::kFoc:
      return "foc";
  }
  return "unknown";
}

ArmCommand ArmCommand::stop() {
  ArmCommand cmd;
  cmd.timed_out = false;
  for (JointCommand& j : cmd.joints) {
    j = JointCommand{};
    j.mode = ControlMode::kStop;
  }
  return cmd;
}

ArmCommand ArmCommand::zeroTorque() {
  ArmCommand cmd;
  cmd.timed_out = false;
  for (JointCommand& j : cmd.joints) {
    j = JointCommand{};
    j.mode = ControlMode::kFoc;  // FOC with zero gain/torque = compliant
  }
  return cmd;
}

ArmCommand ArmCommand::position(const JointVector& q, double kp, double kd) {
  JointVector kp_vec{};
  JointVector kd_vec{};
  kp_vec.fill(kp);
  kd_vec.fill(kd);
  return position(q, kp_vec, kd_vec);
}

ArmCommand ArmCommand::position(const JointVector& q, const JointVector& kp,
                                const JointVector& kd) {
  ArmCommand cmd;
  cmd.timed_out = false;
  for (int i = 0; i < kNumJoints; ++i) {
    JointCommand& j = cmd.joints[static_cast<std::size_t>(i)];
    j.mode = ControlMode::kFoc;
    j.q = q[static_cast<std::size_t>(i)];
    j.dq = 0.0;
    j.tau = 0.0;
    j.kp = kp[static_cast<std::size_t>(i)];
    j.kd = kd[static_cast<std::size_t>(i)];
  }
  return cmd;
}

bool ArmCommand::isStop() const {
  for (const JointCommand& j : joints) {
    if (j.mode != ControlMode::kStop) {
      return false;
    }
  }
  return true;
}

bool ArmCommand::hasActuation() const {
  // Any joint leaving kStop powers the motor bridge, even with zero gains.
  for (const JointCommand& j : joints) {
    if (j.mode != ControlMode::kStop) {
      return true;
    }
  }
  return false;
}

bool ArmCommand::allFinite() const {
  for (const JointCommand& j : joints) {
    if (!std::isfinite(j.q) || !std::isfinite(j.dq) || !std::isfinite(j.tau) ||
        !std::isfinite(j.kp) || !std::isfinite(j.kd)) {
      return false;
    }
  }
  return true;
}

bool RobotState::allValid() const {
  for (bool v : valid) {
    if (!v) {
      return false;
    }
  }
  return true;
}

int RobotState::invalidCount() const {
  int n = 0;
  for (bool v : valid) {
    if (!v) {
      ++n;
    }
  }
  return n;
}

bool RobotState::allFinite() const {
  return finiteVec(q) && finiteVec(dq) && finiteVec(tau);
}

bool RobotState::anyError() const {
  for (std::uint32_t e : error_code) {
    if (e != 0) {
      return true;
    }
  }
  return false;
}

double RobotState::minSupplyVoltage() const {
  double min_v = 0.0;
  bool first = true;
  for (double v : voltage) {
    if (v <= 0.0) {
      continue;  // unknown / not reported by this backend
    }
    if (first || v < min_v) {
      min_v = v;
      first = false;
    }
  }
  return first ? 0.0 : min_v;
}

std::uint64_t RobotState::jointStampNs(int joint) const {
  if (joint < 0 || joint >= kNumJoints) {
    return 0;
  }
  const std::uint64_t s = joint_stamp_ns[static_cast<std::size_t>(joint)];
  return (s != 0) ? s : stamp_ns;
}

std::uint64_t nowNs() {
  // time_since_epoch keeps the value large and monotonically increasing from
  // process start, so timestamp arithmetic (staleness, "in the future") never
  // underflows during the first seconds of a run.
  const auto delta = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count());
}

std::string formatTimeNs(std::uint64_t stamp_ns) {
  char buf[64];
  const double seconds = static_cast<double>(stamp_ns) * 1e-9;
  std::snprintf(buf, sizeof(buf), "%.6f s", seconds);
  return std::string(buf);
}

}  // namespace r1_robot
