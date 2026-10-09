#include "r1_robot/unitree_backend.hpp"

// This translation unit is only compiled when the official SDK was found at
// configure time (see cmake/R1SdkDependency.cmake).
#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

#include "arm_motor_bus.h"

#include "r1_robot/limits.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {
namespace {

/// Lightweight pre-check of the calibration file.
///
/// Motivate: ArmMotorBus::open() calls std::exit(1) when the calibration file
/// is missing or incomplete (verified by reading the official sources:
/// src/arm_motor_bus.cpp, ArmMotorBus::open).  A library must not be able to
/// kill the host process, so the file is validated here first and connect()
/// returns an error instead.
bool calibrationFileUsable(const std::string& path, std::string* detail) {
  std::ifstream ifs(path);
  if (!ifs) {
    if (detail) {
      *detail = "cannot open calibration file: " + path;
    }
    return false;
  }

  std::array<bool, kNumJoints> seen{};
  std::string line;
  int lineno = 0;
  while (std::getline(ifs, line)) {
    ++lineno;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    int joint = -1;
    double dir = 0.0;
    double a = 0.0;
    if (!(ss >> joint >> dir >> a)) {
      if (detail) {
        std::ostringstream oss;
        oss << path << ":" << lineno << ": expected 'joint dir scale offset'";
        *detail = oss.str();
      }
      return false;
    }
    if (joint < 0 || joint >= kNumJoints) {
      if (detail) {
        std::ostringstream oss;
        oss << path << ":" << lineno << ": joint index " << joint
            << " out of range";
        *detail = oss.str();
      }
      return false;
    }
    if (std::fabs(dir) < 1e-6) {
      if (detail) {
        std::ostringstream oss;
        oss << path << ":" << lineno << ": invalid dir for joint " << joint;
        *detail = oss.str();
      }
      return false;
    }
    seen[static_cast<std::size_t>(joint)] = true;
  }

  for (int i = 0; i < kNumJoints; ++i) {
    if (!seen[static_cast<std::size_t>(i)]) {
      if (detail) {
        std::ostringstream oss;
        oss << path << ": missing joint " << i << " (all " << kNumJoints
            << " joints are required)";
        *detail = oss.str();
      }
      return false;
    }
  }

  if (detail) {
    *detail = path;
  }
  return true;
}

ControlMode modeFromMotor(std::uint8_t motor_mode) {
  return (motor_mode == 1) ? ControlMode::kFoc : ControlMode::kStop;
}

std::uint8_t motorFromMode(ControlMode mode) {
  return (mode == ControlMode::kFoc) ? 1u : 0u;
}

}  // namespace

struct UnitreeArmBackend::Impl {
  ArmMotorBus bus;
  bool opened = false;
};

UnitreeArmBackend::UnitreeArmBackend(UnitreeBackendConfig cfg)
    : impl_(std::make_unique<Impl>()), cfg_(std::move(cfg)) {}

UnitreeArmBackend::~UnitreeArmBackend() { disconnect(); }

bool UnitreeArmBackend::connect(std::string* error) {
  auto fail = [error](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (impl_->opened) {
    return true;
  }

  // Fail-closed: a freshly connected backend is never armed for motion.
  gate_.disarm();

  std::string detail;
  if (!calibrationFileUsable(cfg_.calib_path, &detail)) {
    return fail("refused: unusable joint calibration (" + detail + ")");
  }

  if (!impl_->bus.open(cfg_.port, cfg_.baudrate, cfg_.calib_path)) {
    return fail("failed to open serial port '" + cfg_.port + "' at " +
                std::to_string(cfg_.baudrate) + " baud");
  }

  impl_->opened = true;
  return true;
}

void UnitreeArmBackend::disconnect() {
  if (!impl_ || !impl_->opened) {
    return;
  }
  // Mode 0 on every joint, then close.  NOT a brake.
  gate_.disarm();
  impl_->bus.stop();
  impl_->bus.close();
  impl_->opened = false;
}

bool UnitreeArmBackend::isConnected() const { return impl_ && impl_->opened; }

bool UnitreeArmBackend::hasValidCalibration(std::string* detail) const {
  return calibrationFileUsable(cfg_.calib_path, detail);
}

RobotLimits UnitreeArmBackend::limits() const {
  // The official SDK does not expose joint limits.  Fall back to the
  // URDF-derived table and keep its provenance string intact.
  // TODO(bringup): verify against the real actuators.
  return r1DefaultLimits();
}

RobotState UnitreeArmBackend::readState() const {
  RobotState out;
  if (!impl_ || !impl_->opened) {
    out.connected = false;
    return out;
  }

  out.connected = true;
  const std::array<ArmJointState, kArmJointCount> states = impl_->bus.states();
  const std::array<bool, kArmJointCount> ready = impl_->bus.feedbackReady();

  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    out.q[si] = states[si].q;
    out.dq[si] = states[si].dq;
    out.tau[si] = states[si].tau_est;
    out.mode[si] = modeFromMotor(states[si].mode);
    out.error_code[si] = states[si].error_code;
    out.valid[si] = ready[si];
    // NOT AVAILABLE: ArmJointState carries neither voltage nor temperature.
    // Keep them at 0 ("unknown"); SafetyMonitor skips a 0 V reading instead of
    // guessing.  TODO(bringup): extend the backend if the bus exposes them.
    out.voltage[si] = 0.0;
    out.temperature_c[si] = 0.0;
  }

  // The SDK does not expose a per-frame reception timestamp, so the state is
  // stamped at read time.  TODO(bringup): verify that this is representative of
  // the real feedback age at the configured control rate.
  out.stamp_ns = nowNs();
  return out;
}

bool UnitreeArmBackend::writeCommand(const ArmCommand& cmd) {
  if (!impl_ || !impl_->opened) {
    return false;
  }

  // Defense in depth: refuse actuating commands unless R1Arm armed this
  // backend.  A caller that reaches the backend directly cannot drive the
  // motors.  A pure stop command is always allowed through.
  if (!gate_.allows(cmd)) {
    return false;
  }

  std::array<ArmJointCmd, kArmJointCount> translated{};
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    const JointCommand& src = cmd.joints[si];
    ArmJointCmd& dst = translated[si];
    dst.mode = motorFromMode(src.mode);
    dst.q = static_cast<float>(src.q);   // URDF frame: converted inside the SDK
    dst.dq = static_cast<float>(src.dq);
    dst.tau = static_cast<float>(src.tau);
    dst.kp = static_cast<float>(src.kp);
    dst.kd = static_cast<float>(src.kd);
    dst.timed_out = cmd.timed_out;
  }

  impl_->bus.step(translated);
  return true;
}

void UnitreeArmBackend::stop() {
  if (impl_ && impl_->opened) {
    impl_->bus.stop();
  }
}

bool UnitreeArmBackend::probeFeedback(std::string* error) {
  if (!impl_ || !impl_->opened) {
    if (error) {
      *error = "refused: not connected";
    }
    return false;
  }

  // Mode 1 (FOC) with zero gains and zero torque on every joint.
  // THIS POWERS THE BRIDGES - the arm will be limp.
  std::array<ArmJointCmd, kArmJointCount> zero{};
  for (int i = 0; i < kNumJoints; ++i) {
    ArmJointCmd& c = zero[static_cast<std::size_t>(i)];
    c.mode = 1;
    c.timed_out = false;
    c.q = 0.f;
    c.dq = 0.f;
    c.tau = 0.f;
    c.kp = 0.f;
    c.kd = 0.f;
  }
  impl_->bus.step(zero);
  return true;
}

}  // namespace r1_robot

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND
