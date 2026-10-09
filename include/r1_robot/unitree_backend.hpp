#pragma once

// ---------------------------------------------------------------------------
// r1_robot/unitree_backend.hpp
//
// Real-hardware backend: Unitree R1-7A over the official low-level SDK
// (github.com/unitreerobotics/r1-7a_sdk, IM6014 serial bus).
//
// Only compiled when R1_ROBOT_WITH_UNITREE_BACKEND is defined, i.e. when the
// official SDK checkout was found at configure time.
//
// WHAT THIS CLASS DOES
//   * owns one ArmMotorBus (the SDK's public interface),
//   * translates ArmCommand <-> the SDK's ArmJointCmd (URDF frame in, URDF
//     frame out; dir/scale/offset conversion happens inside the SDK),
//   * verifies that a usable calibration file exists *before* opening the port.
//
// WHAT IT DOES NOT DO
//   * it never enables motion, never homes, never sends zero torque by itself,
//   * it never bypasses R1Arm/SafetyMonitor.
//
// KNOWN OFFICIAL-SDK BEHAVIOUR (verified by reading the SDK sources):
//   * ArmMotorBus::open() calls std::exit(1) when the calibration file is
//     missing/incomplete, so the file is checked here first.
//   * Motors only transmit feedback after receiving a control packet, so the
//     first frames must be probed (phase-2 bring-up, see HARDWARE_BRINGUP.md).
//   * The SDK default port is /dev/ttyUSB3 at 6 Mbaud, joint i <-> motor id i.
//
// 待验证: real bus voltage, achievable control rate, feedback latency, whether
// `timed_out` should be true or false for a given application.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>

#include "r1_robot/backend.hpp"
#include "r1_robot/motion_gate.hpp"

namespace r1_robot {

struct UnitreeBackendConfig {
  std::string port = "/dev/ttyUSB3";
  int baudrate = 6000000;
  /// Path to joint_calib.txt.  Relative paths resolve against the CWD, exactly
  /// like the official examples.
  std::string calib_path = "config/joint_calib.txt";
};

class UnitreeArmBackend final : public IArmBackend {
 public:
  explicit UnitreeArmBackend(UnitreeBackendConfig cfg = UnitreeBackendConfig{});
  ~UnitreeArmBackend() override;

  UnitreeArmBackend(const UnitreeArmBackend&) = delete;
  UnitreeArmBackend& operator=(const UnitreeArmBackend&) = delete;

  const char* name() const override { return "unitree"; }
  bool isRealHardware() const override { return true; }

  bool connect(std::string* error) override;
  void disconnect() override;
  bool isConnected() const override;

  bool hasValidCalibration(std::string* detail) const override;
  RobotLimits limits() const override;
  RobotState readState() const override;

  bool writeCommand(const ArmCommand& cmd) override;
  void stop() override;

  /// Arm/disarm the backend-level motion gate.  See r1_robot/motion_gate.hpp.
  /// R1Arm::enableMotion() is the only legitimate caller that passes `true`.
  void setMotionArmed(bool armed) override { gate_.set(armed); }
  bool motionArmed() const override { return gate_.armed(); }

  /// Send ONE zero-torque FOC frame to every joint.
  ///
  /// Motivate: the official SDK notes that the motors only start reporting
  /// feedback after they receive a control packet, so the very first frame has
  /// to come from somewhere.
  ///
  /// WARNING: this powers the motor bridges (mode 1) with zero gain/torque.
  /// The arm will be compliant and WILL NOT hold its own weight.  This method
  /// is never called automatically - an operator must invoke it deliberately
  /// as part of the bring-up procedure in HARDWARE_BRINGUP.md.
  bool probeFeedback(std::string* error);

  const UnitreeBackendConfig& config() const { return cfg_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  UnitreeBackendConfig cfg_;
  /// Backend-level arming latch.  Actuating commands are refused while disarmed.
  MotionGate gate_;
};

}  // namespace r1_robot
