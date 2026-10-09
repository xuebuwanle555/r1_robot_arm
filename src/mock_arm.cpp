#include "r1_robot/mock_arm.hpp"

#include <algorithm>
#include <cmath>

#include "r1_robot/limits.hpp"

namespace r1_robot {
namespace {

constexpr double kDefaultInertia = 0.5;   // [kg m^2]  placeholder
constexpr double kDefaultDamping = 0.5;   // [Nm/(rad/s)] placeholder

JointVector withDefault(const JointVector& v, double fallback) {
  JointVector out = v;
  for (double& x : out) {
    if (!(x > 0.0)) {
      x = fallback;
    }
  }
  return out;
}

}  // namespace

MockArm::MockArm(MockArmConfig cfg) : cfg_(cfg) {
  cfg_.inertia = withDefault(cfg_.inertia, kDefaultInertia);
  cfg_.damping = withDefault(cfg_.damping, kDefaultDamping);
  if (!(cfg_.time_step_s > 0.0)) {
    cfg_.time_step_s = 0.02;
  }
  if (!(cfg_.max_dq > 0.0)) {
    cfg_.max_dq = 2.0;
  }
  last_cmd_ = ArmCommand::stop();
  q_ = JointVector{};
  dq_ = JointVector{};
}

bool MockArm::connect(std::string* error) {
  (void)error;
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = true;
  time_ns_ = 0;
  command_count_ = 0;
  last_cmd_ = ArmCommand::stop();
  // A freshly connected mock arm reports valid feedback for every joint.  Use
  // setFeedbackValid()/setFeedbackValidAll(false) to simulate missing frames.
  fb_valid_.fill(true);
  error_code_.fill(0);
  return true;
}

void MockArm::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
  fb_valid_.fill(false);
}

bool MockArm::isConnected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

bool MockArm::hasValidCalibration(std::string* detail) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (cfg_.calibration_ok) {
    if (detail) {
      *detail = "mock calibration";
    }
    return true;
  }
  if (detail) {
    *detail = "mock calibration disabled (MockArmConfig::calibration_ok=false)";
  }
  return false;
}

RobotLimits MockArm::limits() const { return r1DefaultLimits(); }

RobotState MockArm::readState() const {
  std::lock_guard<std::mutex> lock(mutex_);

  RobotState s;
  s.connected = connected_;
  const std::uint64_t delay_ns =
      static_cast<std::uint64_t>(std::max(0.0, cfg_.feedback_delay_s) * 1e9);
  const std::uint64_t now = nowNs();
  s.stamp_ns = (now > delay_ns) ? (now - delay_ns) : 0;

  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    s.q[si] = q_[si];
    s.dq[si] = dq_[si];
    s.tau[si] = tau_[si];
    s.voltage[si] = cfg_.supply_voltage;
    s.temperature_c[si] = 30.0;  // placeholder
    s.mode[si] = last_cmd_.joints[si].mode;
    s.error_code[si] = error_code_[si];
    s.valid[si] = connected_ && fb_valid_[si];
    // The mock "receives" a frame at `s.stamp_ns` for every joint it reports
    // as valid; an invalid joint has no reception evidence at all.
    s.joint_stamp_ns[si] = s.valid[si] ? s.stamp_ns : 0;
  }
  return s;
}

bool MockArm::writeCommand(const ArmCommand& cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return false;
  }
  last_cmd_ = cmd;
  last_cmd_ns_ = time_ns_;
  ++command_count_;
  return true;
}

void MockArm::stop() {
  writeCommand(ArmCommand::stop());
}

void MockArm::advance(double dt_s) {
  if (!(dt_s > 0.0)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return;
  }
  const double step = cfg_.time_step_s;
  const int substeps = std::max(1, static_cast<int>(std::ceil(dt_s / step)));
  const double h = dt_s / static_cast<double>(substeps);
  for (int k = 0; k < substeps; ++k) {
    integrate(h);
  }
  time_ns_ += static_cast<std::uint64_t>(dt_s * 1e9);
}

void MockArm::integrate(double h) {
  const RobotLimits limits = r1DefaultLimits();

  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    const JointCommand& c = last_cmd_.joints[si];

    double tau = -cfg_.damping[si] * dq_[si] - cfg_.gravity_torque[si];
    if (c.mode == ControlMode::kFoc) {
      tau += c.kp * (c.q - q_[si]) + c.kd * (c.dq - dq_[si]) + c.tau;
    }

    const double ddq = tau / cfg_.inertia[si];
    double dq = dq_[si] + ddq * h;
    dq = std::min(std::max(dq, -cfg_.max_dq), cfg_.max_dq);
    dq_[si] = dq;

    double q = q_[si] + dq * h;
    const JointLimits& l = limits.joints[si];
    if (q < l.lower) {
      q = l.lower;
      dq_[si] = 0.0;
    } else if (q > l.upper) {
      q = l.upper;
      dq_[si] = 0.0;
    }
    q_[si] = q;
    tau_[si] = tau;
  }
}

std::uint64_t MockArm::simulatedTimeNs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return time_ns_;
}

void MockArm::setJointPosition(int joint, double q) {
  if (joint < 0 || joint >= kNumJoints) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  q_[static_cast<std::size_t>(joint)] = q;
}

void MockArm::setJointVelocity(int joint, double dq) {
  if (joint < 0 || joint >= kNumJoints) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dq_[static_cast<std::size_t>(joint)] = dq;
}

void MockArm::setJointPositions(const JointVector& q) {
  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q;
}

void MockArm::setFeedbackValid(int joint, bool valid) {
  if (joint < 0 || joint >= kNumJoints) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  fb_valid_[static_cast<std::size_t>(joint)] = valid;
}

void MockArm::setFeedbackValidAll(bool valid) {
  std::lock_guard<std::mutex> lock(mutex_);
  fb_valid_.fill(valid);
}

void MockArm::setCalibrationOk(bool ok) {
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_.calibration_ok = ok;
}

void MockArm::setSupplyVoltage(double volts) {
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_.supply_voltage = volts;
}

void MockArm::setFeedbackDelay(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_.feedback_delay_s = seconds;
}

void MockArm::injectError(int joint, std::uint32_t code) {
  if (joint < 0 || joint >= kNumJoints) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_[static_cast<std::size_t>(joint)] = code;
}

void MockArm::clearErrors() {
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_.fill(0);
}

ArmCommand MockArm::lastCommand() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_cmd_;
}

std::uint64_t MockArm::commandCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_count_;
}

ArmBackendPtr makeMockBackend() {
  return std::make_shared<MockArm>(MockArmConfig{});
}

}  // namespace r1_robot
