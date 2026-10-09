#pragma once

// ---------------------------------------------------------------------------
// r1_robot/mock_arm.hpp
//
// Offline, deterministic backend.  This is the ONLY backend enabled by default
// and it never touches hardware.  It exists so that:
//   * the whole stack (R1Arm + SafetyMonitor) is unit-testable without an arm,
//   * safety behaviour (stale feedback, faults, missing calibration, limits)
//     can be exercised on purpose.
//
// MODEL (deliberately simple, not a physics engine):
//   per joint:  tau = kp*(q_cmd - q) + kd*(dq_cmd - dq) + tau_ff - gravity
//               ddq = tau / inertia
//   integrated in fixed substeps of MockArmConfig::time_step_s, with velocity
//   and position clamped.  Time only advances when advance() is called, so
//   tests are deterministic and never sleep.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <mutex>
#include <string>

#include "r1_robot/backend.hpp"

namespace r1_robot {

struct MockArmConfig {
  /// Reported by hasValidCalibration().  Set false to test the "no calibration"
  /// refusal path.
  bool calibration_ok = true;

  /// Reported supply voltage [V].  Set below
  /// SafetyConfig::min_supply_voltage to test the undervoltage path.
  double supply_voltage = 48.0;

  /// Simulated inertia [kg m^2] per joint (must be > 0).
  JointVector inertia{};
  /// Simulated viscous damping [Nm/(rad/s)] per joint.
  JointVector damping{};
  /// Constant gravity load [Nm] per joint (sign = direction of the load).
  /// Default zero: the mock arm is NOT gravity-loaded unless a test asks.
  JointVector gravity_torque{};

  /// Hard clamp on |dq| inside the model [rad/s].
  double max_dq = 2.0;
  /// Integration substep [s].
  double time_step_s = 0.02;

  /// Artificial feedback age [s].  State timestamps are reported this much in
  /// the past so freshness checks can be exercised.
  double feedback_delay_s = 0.0;

  /// Errors injected into every reported feedback frame.
  std::uint32_t error_code = 0;
};

class MockArm final : public IArmBackend {
 public:
  explicit MockArm(MockArmConfig cfg = MockArmConfig{});

  // -- IArmBackend ---------------------------------------------------------
  const char* name() const override { return "mock"; }
  bool isRealHardware() const override { return false; }
  bool connect(std::string* error) override;
  void disconnect() override;
  bool isConnected() const override;
  bool hasValidCalibration(std::string* detail) const override;
  RobotLimits limits() const override;
  RobotState readState() const override;
  bool writeCommand(const ArmCommand& cmd) override;
  void stop() override;

  // -- deterministic test helpers -----------------------------------------
  /// Advance simulated time by `dt_s`, integrating the model in fixed
  /// substeps.  Calls advance(0) is a no-op.
  void advance(double dt_s);

  /// Current simulated time [ns].
  std::uint64_t simulatedTimeNs() const;

  /// Overwrite a joint position directly (test setup / "human moved the arm").
  void setJointPosition(int joint, double q);
  void setJointVelocity(int joint, double dq);
  void setJointPositions(const JointVector& q);

  /// Per-joint feedback validity.  Invalid joints are reported with
  /// valid == false and mimic "no frame received yet".
  void setFeedbackValid(int joint, bool valid);
  void setFeedbackValidAll(bool valid);

  void setCalibrationOk(bool ok);
  void setSupplyVoltage(double volts);
  /// Report feedback timestamps this many seconds in the past (freshness tests).
  void setFeedbackDelay(double seconds);
  void injectError(int joint, std::uint32_t code);
  void clearErrors();

  ArmCommand lastCommand() const;
  std::uint64_t commandCount() const;

 private:
  void integrate(double dt_s);

  mutable std::mutex mutex_;
  MockArmConfig cfg_;
  bool connected_ = false;
  std::uint64_t time_ns_ = 0;
  std::uint64_t command_count_ = 0;

  JointVector q_{};
  JointVector dq_{};
  JointVector tau_{};
  ArmCommand last_cmd_{};
  std::uint64_t last_cmd_ns_ = 0;
  JointFlags fb_valid_{};
  std::array<std::uint32_t, kNumJoints> error_code_{};
};

/// Convenience for tests/examples: build a ready-to-connect mock backend.
ArmBackendPtr makeMockBackend();

}  // namespace r1_robot
