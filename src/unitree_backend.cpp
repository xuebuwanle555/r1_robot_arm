#include "r1_robot/unitree_backend.hpp"

// This translation unit is only compiled when the official SDK was found at
// configure time (see cmake/R1SdkDependency.cmake).
#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

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

}

struct UnitreeArmBackend::Impl {
  /// Transport.  Defaults to the real SDK bus; injectable for offline tests.
  JointBusPtr bus;
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

  if (!impl_->bus) {
    impl_->bus = cfg_.bus_factory ? cfg_.bus_factory() : makeSdkJointBus();
  }
  if (!impl_->bus) {
    return fail("refused: this build has no SDK joint bus (configure with "
                "-DR1_ROBOT_WITH_UNITREE_BACKEND=ON)");
  }

  std::string detail;
  if (!calibrationFileUsable(cfg_.calib_path, &detail)) {
    return fail("refused: unusable joint calibration (" + detail + ")");
  }

  std::string bus_error;
  if (!impl_->bus->open(cfg_.port, cfg_.baudrate, cfg_.calib_path, &bus_error)) {
    return fail(bus_error.empty()
                    ? ("failed to open serial port '" + cfg_.port + "' at " +
                       std::to_string(cfg_.baudrate) + " baud")
                    : bus_error);
  }

  impl_->opened = true;
  return true;
}

void UnitreeArmBackend::disconnect() {
  if (!impl_ || !impl_->opened || !impl_->bus) {
    return;
  }
  // Mode 0 on every joint, then close.  NOT a brake.
  gate_.disarm();
  impl_->bus->stop();
  impl_->bus->close();
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
  // A dropped port is not a usable state: report "not connected" so that every
  // actuating command is refused, even if nobody called disconnect() yet.
  if (!impl_ || !impl_->opened || !impl_->bus || !impl_->bus->isOpen()) {
    out.connected = false;
    return out;
  }

  out.connected = true;
  // ONE atomic snapshot: values + reception evidence captured together, so a
  // stale joint value can never be paired with a fresh age (the two-read race
  // that existed when feedback() and feedbackStats() were called separately).
  const JointBusSnapshot snap = impl_->bus->snapshot();
  // A single monotonic reference (this backend's clock) converts every joint's
  // age into a reception stamp, so the whole state shares ONE clock epoch.  The
  // snapshot's own `now_ns` (the instant the bus measured the ages) is retained
  // for diagnostics/atomicity assertions but is not needed to derive the stamp.
  const std::uint64_t read_now = nowNs();

  std::uint64_t oldest = 0;
  bool any = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    const JointBusFeedback& fb = snap.feedback[si];
    const JointBusFeedbackStats& stats = snap.stats[si];

    out.q[si] = fb.q;
    out.dq[si] = fb.dq;
    out.tau[si] = fb.tau_est;
    out.mode[si] = fb.mode;
    out.error_code[si] = fb.error_code;
    // `valid` reflects "a CRC-valid frame has been received", which is all the
    // bus can prove.  Freshness is carried separately by the timestamps below.
    out.valid[si] = stats.received;
    // NOT AVAILABLE: the SDK's ArmJointState carries neither voltage nor
    // temperature.  Keep them at 0 ("unknown"); SafetyMonitor skips a 0 V
    // reading instead of guessing.  TODO(bringup): extend if the bus exposes it.
    out.voltage[si] = 0.0;
    out.temperature_c[si] = 0.0;

    // Reception time of the newest valid frame = read time - age.  NEVER the
    // time of the next call: stamping at read time would make freshness vacuous.
    std::uint64_t stamp = 0;
    if (stats.received) {
      stamp = (read_now > stats.age_ns) ? (read_now - stats.age_ns) : read_now;
    }
    out.joint_stamp_ns[si] = stamp;
    if (stats.received && (!any || stamp < oldest)) {
      oldest = stamp;
      any = true;
    }
  }

  // The global stamp is the age of the STALEST joint, so the existing global
  // freshness check catches a single silent joint as well.
  out.stamp_ns = any ? oldest : 0;
  return out;
}

bool UnitreeArmBackend::writeCommand(const ArmCommand& cmd) {
  if (!impl_ || !impl_->opened || !impl_->bus) {
    return false;
  }

  // Defense in depth: refuse actuating commands unless R1Arm armed this
  // backend.  A caller that reaches the backend directly cannot drive the
  // motors.  A pure stop command is always allowed through.
  if (!gate_.allows(cmd)) {
    return false;
  }

  std::array<JointBusCommand, kNumJoints> translated{};
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    const JointCommand& src = cmd.joints[si];
    JointBusCommand& dst = translated[si];
    dst.mode = src.mode;
    dst.q = src.q;    // URDF frame: converted inside the SDK
    dst.dq = src.dq;
    dst.tau = src.tau;
    dst.kp = src.kp;
    dst.kd = src.kd;
    dst.timed_out = cmd.timed_out;
  }

  impl_->bus->step(translated);
  return true;
}

void UnitreeArmBackend::stop() {
  if (impl_ && impl_->opened && impl_->bus) {
    impl_->bus->stop();
  }
}

bool UnitreeArmBackend::probeFeedback(std::string* error,
                                      const MotionControl& control) {
  (void)control;  // Possession of the token proves the caller is R1Arm or
                  // ControlService; ordinary business code cannot construct one.
  if (!impl_ || !impl_->opened || !impl_->bus) {
    if (error) {
      *error = "refused: not connected";
    }
    return false;
  }

  // Mode 1 (FOC) with zero gains and zero torque on every joint.
  // THIS POWERS THE BRIDGES - the arm will be limp.
  std::array<JointBusCommand, kNumJoints> zero{};
  for (int i = 0; i < kNumJoints; ++i) {
    JointBusCommand& c = zero[static_cast<std::size_t>(i)];
    c.mode = ControlMode::kFoc;
    c.timed_out = false;
    c.q = 0.0;
    c.dq = 0.0;
    c.tau = 0.0;
    c.kp = 0.0;
    c.kd = 0.0;
  }
  impl_->bus->step(zero);
  return true;
}

}  // namespace r1_robot

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND
