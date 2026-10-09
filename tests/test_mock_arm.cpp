#include <cmath>
#include <string>

#include "r1_robot/limits.hpp"
#include "r1_robot/mock_arm.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

JointVector filled(double value) {
  JointVector v{};
  v.fill(value);
  return v;
}

}  // namespace

R1_TEST(mock_arm_is_offline_and_starts_disconnected) {
  MockArm arm;
  R1_CHECK(!arm.isConnected());
  R1_CHECK(!arm.readState().connected);
  R1_CHECK_EQ(arm.commandCount(), 0u);
  R1_CHECK(arm.writeCommand(ArmCommand::stop()) == false);
  arm.advance(1.0);  // must be a no-op before connect
  R1_CHECK_EQ(arm.simulatedTimeNs(), 0u);
}

R1_TEST(mock_arm_is_not_real_hardware) {
  MockArm arm;
  R1_CHECK(!arm.isRealHardware());
  R1_CHECK_EQ(std::string(arm.name()), std::string("mock"));
}

R1_TEST(mock_arm_reports_fresh_valid_feedback_after_connect) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.isConnected());

  const RobotState s = arm.readState();
  R1_CHECK(s.connected);
  R1_CHECK(s.allValid());
  R1_CHECK(!s.anyError());
  R1_CHECK_NEAR(s.voltage[0], 48.0, 1e-9);

  const std::uint64_t now = nowNs();
  R1_CHECK(s.stamp_ns <= now);
  R1_CHECK(now - s.stamp_ns < 100ULL * 1000ULL * 1000ULL);  // younger than 100 ms
}

R1_TEST(mock_arm_servos_toward_a_position_command) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));
  arm.setJointPositions(filled(0.0));

  R1_CHECK(arm.writeCommand(ArmCommand::position(filled(0.2), 20.0, 3.0)));
  R1_CHECK_EQ(arm.commandCount(), 1u);

  arm.advance(3.0);

  const RobotState s = arm.readState();
  for (int i = 0; i < kNumJoints; ++i) {
    R1_CHECK_NEAR(s.q[static_cast<std::size_t>(i)], 0.2, 0.02);
  }
}

R1_TEST(mock_arm_stop_mode_bleeds_off_velocity) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));
  arm.setJointVelocity(0, 1.0);

  R1_CHECK(arm.writeCommand(ArmCommand::stop()));
  arm.advance(1.0);

  const RobotState s = arm.readState();
  R1_CHECK(std::fabs(s.dq[0]) < 0.6);  // viscous decay, but not instant
}

R1_TEST(mock_arm_clamps_to_the_urdf_limits) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));

  R1_CHECK(arm.writeCommand(ArmCommand::position(filled(9.0), 20.0, 3.0)));
  arm.advance(3.0);

  const RobotLimits limits = r1DefaultLimits();
  const RobotState s = arm.readState();
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    R1_CHECK(s.q[si] <= limits.joints[si].upper + 1e-9);
    R1_CHECK(s.q[si] >= limits.joints[si].lower - 1e-9);
  }
}

R1_TEST(mock_arm_can_simulate_missing_feedback) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));

  arm.setFeedbackValidAll(false);
  R1_CHECK_EQ(arm.readState().invalidCount(), kNumJoints);

  arm.setFeedbackValid(2, true);
  const RobotState s = arm.readState();
  R1_CHECK(s.valid[2]);
  R1_CHECK_EQ(s.invalidCount(), kNumJoints - 1);

  arm.disconnect();
  R1_CHECK(!arm.readState().valid[2]);
}

R1_TEST(mock_arm_can_simulate_motor_errors) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));

  arm.injectError(2, 0x8);
  RobotState s = arm.readState();
  R1_CHECK(s.anyError());
  R1_CHECK_EQ(s.error_code[2], 0x8u);

  arm.clearErrors();
  s = arm.readState();
  R1_CHECK(!s.anyError());
}

R1_TEST(mock_arm_can_simulate_stale_feedback) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));

  arm.setFeedbackDelay(1.0);
  const RobotState s = arm.readState();
  R1_CHECK(s.stamp_ns + 900ULL * 1000ULL * 1000ULL < nowNs());
}

R1_TEST(mock_arm_can_simulate_a_missing_calibration) {
  MockArm arm;
  std::string detail;

  R1_CHECK(arm.hasValidCalibration(&detail));
  R1_CHECK_CONTAINS(detail, "mock calibration");

  arm.setCalibrationOk(false);
  detail.clear();
  R1_CHECK(!arm.hasValidCalibration(&detail));
  R1_CHECK_CONTAINS(detail, "disabled");
}

R1_TEST(mock_arm_can_simulate_an_undervoltage_supply) {
  MockArm arm;
  std::string error;
  R1_CHECK(arm.connect(&error));
  arm.setSupplyVoltage(20.0);

  const RobotState s = arm.readState();
  R1_CHECK_NEAR(s.minSupplyVoltage(), 20.0, 1e-9);
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.undervoltage");
}

R1_TEST(mock_arm_limits_come_from_the_urdf_table) {
  MockArm arm;
  const RobotLimits limits = arm.limits();
  R1_CHECK_NEAR(limits.joints[6].upper, 1.571, 1e-9);
}
