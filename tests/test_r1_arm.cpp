#include <memory>
#include <string>

#include "r1_robot/mock_arm.hpp"
#include "r1_robot/r1_arm.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

/// A MockArm that pretends to be real hardware, so the
/// "allow_real_hardware" gate can be exercised without any hardware.
class FakeRealBackend final : public IArmBackend {
 public:
  FakeRealBackend() : mock_(MockArmConfig{}) {}

  const char* name() const override { return "fake-real"; }
  bool isRealHardware() const override { return true; }
  bool connect(std::string* error) override { return mock_.connect(error); }
  void disconnect() override { mock_.disconnect(); }
  bool isConnected() const override { return mock_.isConnected(); }
  bool hasValidCalibration(std::string* detail) const override {
    return mock_.hasValidCalibration(detail);
  }
  RobotLimits limits() const override { return mock_.limits(); }
  RobotState readState() const override { return mock_.readState(); }
  bool writeCommand(const ArmCommand& cmd) override {
    return mock_.writeCommand(cmd);
  }
  void stop() override { mock_.stop(); }

  MockArm& mock() { return mock_; }

 private:
  MockArm mock_;
};

JointVector filled(double value) {
  JointVector v{};
  v.fill(value);
  return v;
}

}  // namespace

R1_TEST(r1_arm_mock_connects_but_never_enables_motion_by_itself) {
  R1Arm arm(makeMockBackend());
  std::string error;

  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.connected());
  R1_CHECK(!arm.motionEnabled());
  R1_CHECK(!arm.status().real_hardware);
  R1_CHECK(arm.status().calibration_ok);
  R1_CHECK_CONTAINS(arm.describe(), "motion_enabled=no");

  // A position command must be refused while motion is not enabled.
  R1_CHECK(!arm.sendAction(ArmCommand::position(filled(0.2), 20.0, 3.0), &error));
  R1_CHECK_CONTAINS(error, "motion is not enabled");
}

R1_TEST(r1_arm_refuses_a_real_backend_without_the_master_switch) {
  auto backend = std::make_shared<FakeRealBackend>();
  R1Arm arm(backend);
  std::string error;

  R1_CHECK(!arm.connect(&error));
  R1_CHECK_CONTAINS(error, "refused");
  R1_CHECK_CONTAINS(error, "allow_real_hardware");
  R1_CHECK(!arm.connected());
  R1_CHECK(!backend->isConnected());  // the backend was never opened
}

R1_TEST(r1_arm_allows_a_real_backend_only_with_the_master_switch) {
  auto backend = std::make_shared<FakeRealBackend>();
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.status().real_hardware);
  R1_CHECK(!arm.motionEnabled());  // connecting still does not enable anything
}

R1_TEST(r1_arm_requires_an_explicit_human_confirmation_to_enable) {
  R1Arm arm(makeMockBackend());
  std::string error;
  R1_CHECK(arm.connect(&error));

  R1_CHECK(!arm.enableMotion(false, &error));
  R1_CHECK_CONTAINS(error, "human confirmation");
  R1_CHECK(!arm.motionEnabled());

  R1_CHECK(arm.enableMotion(true, &error));
  R1_CHECK(arm.motionEnabled());
}

R1_TEST(r1_arm_refuses_to_enable_motion_without_calibration) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  backend->setCalibrationOk(false);
  R1Arm arm(backend);
  std::string error;
  R1_CHECK(arm.connect(&error));

  R1_CHECK(!arm.enableMotion(true, &error));
  R1_CHECK_CONTAINS(error, "enable.no_calibration");
  R1_CHECK(!arm.motionEnabled());
}

R1_TEST(r1_arm_refuses_to_enable_motion_without_full_feedback) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  R1Arm arm(backend);
  std::string error;
  R1_CHECK(arm.connect(&error));
  backend->setFeedbackValid(3, false);
  backend->setFeedbackValid(5, false);

  R1_CHECK(!arm.enableMotion(true, &error));
  R1_CHECK_CONTAINS(error, "state.joint_invalid");
}

R1_TEST(r1_arm_accepts_a_valid_command_once_enabled) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  backend->setJointPositions(filled(0.18));
  R1Arm arm(backend);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));

  R1_CHECK(arm.sendAction(ArmCommand::position(filled(0.2), 20.0, 3.0), &error));
  R1_CHECK_EQ(backend->commandCount(), 1u);
}

R1_TEST(r1_arm_still_validates_commands_after_enabling) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  backend->setJointPositions(filled(0.18));
  R1Arm arm(backend);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));

  // Too far from the measured position (anti-step check).
  R1_CHECK(!arm.sendAction(ArmCommand::position(filled(0.3), 20.0, 3.0), &error));
  R1_CHECK_CONTAINS(error, "cmd.step_too_large");

  // Gain outside the envelope.
  R1_CHECK(!arm.sendAction(ArmCommand::position(filled(0.2), 5000.0, 3.0), &error));
  R1_CHECK_CONTAINS(error, "cmd.gain_out_of_range");

  // Outside the soft joint limits.
  JointVector q = filled(0.2);
  q[1] = 2.0;  // J2 is [0, 3.1416]; 2.0 is fine, so use J6 instead
  q[5] = -0.5;  // J6 is [-0.524, 0.524] -> outside the 0.05 margin
  R1_CHECK(!arm.sendAction(ArmCommand::position(q, 20.0, 3.0), &error));
  R1_CHECK_CONTAINS(error, "cmd.out_of_limits");

  R1_CHECK_EQ(backend->commandCount(), 0u);
}

R1_TEST(r1_arm_always_accepts_a_stop_command) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  R1Arm arm(backend);
  std::string error;
  R1_CHECK(arm.connect(&error));

  // Even with motion disabled and no feedback at all.
  backend->setFeedbackValidAll(false);
  R1_CHECK(arm.sendAction(ArmCommand::stop(), &error));
  R1_CHECK_EQ(backend->commandCount(), 1u);
}

R1_TEST(r1_arm_emergency_stop_disarms_and_stops) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  backend->setJointPositions(filled(0.18));
  R1Arm arm(backend);
  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));

  arm.emergencyStop();
  R1_CHECK(!arm.motionEnabled());
  R1_CHECK(backend->lastCommand().isStop());

  // After an emergency stop the command path stays closed.
  R1_CHECK(!arm.sendAction(ArmCommand::position(filled(0.2), 20.0, 3.0), &error));
  R1_CHECK_CONTAINS(error, "motion is not enabled");
}

R1_TEST(r1_arm_homing_is_blocked_in_phase_1) {
  R1Arm arm(makeMockBackend());
  std::string error;
  R1_CHECK(arm.connect(&error));

  R1_CHECK(!arm.home(&error));
  R1_CHECK_CONTAINS(error, "blocked");
  R1_CHECK_CONTAINS(error, "phase 1");
}

R1_TEST(r1_arm_disconnect_leaves_a_stopped_backend) {
  auto backend = std::make_shared<MockArm>(MockArmConfig{});
  R1Arm arm(backend);
  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));

  arm.disconnect();
  R1_CHECK(!arm.connected());
  R1_CHECK(!arm.motionEnabled());
  R1_CHECK(!backend->isConnected());
  R1_CHECK(backend->lastCommand().isStop());
}

R1_TEST(r1_arm_state_reports_disconnected_before_connect) {
  R1Arm arm(makeMockBackend());
  R1_CHECK(!arm.state().connected);
  R1_CHECK_FAULT(arm.checkState(), "state.not_connected");
}

R1_TEST(r1_arm_without_a_backend_refuses_everything) {
  R1Arm arm(nullptr);
  std::string error;
  R1_CHECK(!arm.connect(&error));
  R1_CHECK_CONTAINS(error, "no backend");
}
