#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "r1_robot/limits.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

R1_TEST(arm_command_stop_is_inert) {
  const ArmCommand cmd = ArmCommand::stop();
  R1_CHECK(cmd.isStop());
  R1_CHECK(!cmd.hasActuation());
  R1_CHECK(cmd.allFinite());
}

R1_TEST(arm_command_zero_torque_is_actuating) {
  // Zero-torque still puts every motor in FOC (mode 1), i.e. it powers the
  // bridges.  It must therefore count as actuation and stay behind the motion
  // gate.
  const ArmCommand cmd = ArmCommand::zeroTorque();
  R1_CHECK(!cmd.isStop());
  R1_CHECK(cmd.hasActuation());
  for (const JointCommand& j : cmd.joints) {
    R1_CHECK(j.mode == ControlMode::kFoc);
    R1_CHECK_EQ(j.kp, 0.0);
    R1_CHECK_EQ(j.tau, 0.0);
  }
}

R1_TEST(arm_command_position_fills_vectors) {
  JointVector q{};
  q.fill(0.2);
  const ArmCommand cmd = ArmCommand::position(q, 30.0, 2.0);
  R1_CHECK(cmd.hasActuation());
  for (int i = 0; i < kNumJoints; ++i) {
    R1_CHECK_EQ(cmd.joints[static_cast<std::size_t>(i)].mode, ControlMode::kFoc);
    R1_CHECK_NEAR(cmd.joints[static_cast<std::size_t>(i)].q, 0.2, 1e-12);
    R1_CHECK_NEAR(cmd.joints[static_cast<std::size_t>(i)].kp, 30.0, 1e-12);
    R1_CHECK_NEAR(cmd.joints[static_cast<std::size_t>(i)].kd, 2.0, 1e-12);
  }
}

R1_TEST(arm_command_rejects_non_finite) {
  ArmCommand cmd = ArmCommand::position(JointVector{}, 10.0, 1.0);
  cmd.joints[3].q = std::numeric_limits<double>::quiet_NaN();
  R1_CHECK(!cmd.allFinite());

  cmd = ArmCommand::position(JointVector{}, 10.0, 1.0);
  cmd.joints[0].tau = std::numeric_limits<double>::infinity();
  R1_CHECK(!cmd.allFinite());
}

R1_TEST(robot_state_helpers) {
  RobotState s;
  R1_CHECK(!s.allValid());
  R1_CHECK_EQ(s.invalidCount(), kNumJoints);

  s.valid.fill(true);
  R1_CHECK(s.allValid());
  R1_CHECK_EQ(s.invalidCount(), 0);

  s.valid[6] = false;
  R1_CHECK_EQ(s.invalidCount(), 1);
  R1_CHECK(!s.anyError());
  s.error_code[2] = 0x8;
  R1_CHECK(s.anyError());

  R1_CHECK_EQ(s.minSupplyVoltage(), 0.0);  // "unknown" is not a voltage
  s.voltage.fill(48.0);
  s.voltage[4] = 41.5;
  R1_CHECK_NEAR(s.minSupplyVoltage(), 41.5, 1e-12);
}

R1_TEST(default_limits_match_official_urdf) {
  const RobotLimits limits = r1DefaultLimits();

  const std::array<double, kNumJoints> lower = {-2.723, 0.0,   -0.349, 0.0,
                                                -2.269, -0.524, -1.571};
  const std::array<double, kNumJoints> upper = {2.723,  3.1416, 3.491,  3.1415,
                                                2.269,  0.524,  1.571};
  const std::array<double, kNumJoints> vel = {18.7, 18.7, 33.4, 33.4,
                                              33.4, 37.7, 37.7};
  const std::array<double, kNumJoints> torque = {60.0, 60.0, 33.0, 33.0,
                                                 33.0, 10.0, 10.0};

  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    R1_CHECK_NEAR(limits.joints[si].lower, lower[si], 1e-9);
    R1_CHECK_NEAR(limits.joints[si].upper, upper[si], 1e-9);
    R1_CHECK_NEAR(limits.joints[si].max_velocity, vel[si], 1e-9);
    R1_CHECK_NEAR(limits.joints[si].max_torque, torque[si], 1e-9);
  }
  R1_CHECK_CONTAINS(limits.source, "UNVERIFIED");
}

R1_TEST(check_limits_reports_the_offending_joint) {
  const RobotLimits limits = r1DefaultLimits();

  JointVector q{};
  q.fill(0.1);
  std::string error;
  R1_CHECK(checkLimits(q, limits, 0.0, &error));

  q[2] = 9.0;  // J3 upper bound is 3.491
  error.clear();
  R1_CHECK(!checkLimits(q, limits, 0.0, &error));
  R1_CHECK_CONTAINS(error, "joint 2");

  q[2] = 2.0;
  R1_CHECK(checkLimits(q, limits, 0.0, &error));
}

R1_TEST(clamp_to_limits_reports_modification) {
  const RobotLimits limits = r1DefaultLimits();

  JointVector q{};
  q.fill(0.1);
  R1_CHECK(!clampToLimits(q, limits, 0.0));

  q[2] = 9.0;
  R1_CHECK(clampToLimits(q, limits, 0.0));
  R1_CHECK_NEAR(q[2], limits.joints[2].upper, 1e-12);

  q[2] = 9.0;
  R1_CHECK(clampToLimits(q, limits, 0.1));
  R1_CHECK_NEAR(q[2], limits.joints[2].upper - 0.1, 1e-12);
}
