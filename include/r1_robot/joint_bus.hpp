#pragma once

// ---------------------------------------------------------------------------
// r1_robot/joint_bus.hpp
//
// Transport-level joint bus interface, in *r1_robot* types (SDK-free).
//
// WHY THIS EXISTS
//   The real bus is the official SDK's ArmMotorBus, but the safety-relevant
//   logic (feedback freshness, error handling, disconnect/recovery) must be
//   testable offline and must not depend on the SDK.  UnitreeArmBackend
//   therefore talks to IJointBus, and:
//     * SdkJointBus (src/sdk_joint_bus.cpp) wraps the official ArmMotorBus and
//       is only compiled when the SDK is present;
//     * tests inject a fake bus to drive delivery, loss, CRC-style drops and
//       port disconnects deterministically (all without hardware or a serial
//       port).
//
// FEEDBACK EVIDENCE
//   `feedback()` carries the values; `feedbackStats()` carries the *evidence*
//   that a valid (CRC-checked) frame actually arrived, when it arrived
//   (monotonic age) and how many arrived.  A sticky "ready" flag is NOT enough:
//   it cannot distinguish "receiving continuously" from "stopped receiving
//   after the first frame".  Callers must never substitute the time of the
//   read for the time of reception.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "r1_robot/types.hpp"

namespace r1_robot {

/// One joint command at the bus edge (URDF frame, SI units).
struct JointBusCommand {
  ControlMode mode = ControlMode::kStop;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  bool timed_out = false;
};

/// One joint feedback frame at the bus edge (URDF frame, SI units).
struct JointBusFeedback {
  double q = 0.0;
  double dq = 0.0;
  double tau_est = 0.0;
  ControlMode mode = ControlMode::kStop;
  bool timeout = false;
  std::uint32_t error_code = 0;
};

/// Per-joint reception evidence.
struct JointBusFeedbackStats {
  /// At least one valid (CRC-checked) frame has arrived since the port opened.
  bool received = false;
  /// Number of valid frames received since the port opened.
  std::uint64_t packets = 0;
  /// Age of the newest valid frame [ns].  0 when `received == false`.
  std::uint64_t age_ns = 0;
};

/// Transport-level joint bus.
class IJointBus {
 public:
  virtual ~IJointBus() = default;

  /// Open the transport and load the calibration.
  /// Returns false + `error` on failure.  Must never terminate the process.
  virtual bool open(const std::string& port, int baudrate,
                    const std::string& calib_path, std::string* error) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  /// Send one command frame for every joint.
  virtual void step(const std::array<JointBusCommand, kNumJoints>& cmds) = 0;

  /// Latest feedback values (may be zeros when nothing has arrived yet).
  virtual std::array<JointBusFeedback, kNumJoints> feedback() const = 0;

  /// Reception evidence per joint (see the header comment).
  virtual std::array<JointBusFeedbackStats, kNumJoints> feedbackStats() const =
      0;

  /// Mode-0 stop on every joint.
  virtual void stop() = 0;
  virtual void clearFaults() = 0;
  virtual void resetMotors() = 0;
};

using JointBusPtr = std::shared_ptr<IJointBus>;

/// True when this build can create a real SDK bus.
bool hasSdkJointBus();

/// Create the real SDK-backed bus (nullptr when the SDK backend is not built).
/// NOTE: creating it does not open any port.
JointBusPtr makeSdkJointBus();

}  // namespace r1_robot
