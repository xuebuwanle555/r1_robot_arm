#include <string>

#include "r1_robot/gripper.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

R1_TEST(dm4310_gripper_fails_closed) {
  Dm4310Gripper gripper;
  std::string error;

  R1_CHECK(gripper.isRealHardware());
  R1_CHECK(!gripper.connect(&error));
  R1_CHECK_CONTAINS(error, "not implemented");
  R1_CHECK(!gripper.isConnected());
  R1_CHECK(!gripper.state().valid);

  error.clear();
  R1_CHECK(!gripper.writeCommand(GripperCommand{}, &error));
  R1_CHECK_CONTAINS(error, "not implemented");

  R1_CHECK_CONTAINS(gripper.limits().source, "UNVERIFIED");
  gripper.stop();  // must be a harmless no-op
}

R1_TEST(mock_gripper_round_trip) {
  MockGripper gripper;
  std::string error;

  R1_CHECK(gripper.connect(&error));
  R1_CHECK(gripper.isConnected());
  R1_CHECK(gripper.state().valid);
  R1_CHECK(gripper.limits().max_current > 0.0);

  GripperCommand cmd;
  cmd.mode = GripperCommandMode::kPosition;
  cmd.position = 0.7;
  R1_CHECK(gripper.writeCommand(cmd, &error));
  R1_CHECK_NEAR(gripper.state().position, 0.7, 1e-9);
  R1_CHECK_EQ(gripper.commandCount(), 1u);

  gripper.stop();
  R1_CHECK(gripper.lastCommand().mode == GripperCommandMode::kStop);
  R1_CHECK_EQ(gripper.commandCount(), 2u);
}

R1_TEST(mock_gripper_validates_its_envelope) {
  MockGripper gripper;
  std::string error;
  R1_CHECK(gripper.connect(&error));

  GripperCommand cmd;
  cmd.mode = GripperCommandMode::kPosition;
  cmd.position = 1.5;  // outside the normalized travel
  R1_CHECK(!gripper.writeCommand(cmd, &error));
  R1_CHECK_CONTAINS(error, "outside normalized range");

  error.clear();
  cmd = GripperCommand{};
  cmd.mode = GripperCommandMode::kCurrent;
  cmd.current = 99.0;
  R1_CHECK(!gripper.writeCommand(cmd, &error));
  R1_CHECK_CONTAINS(error, "max_current");

  error.clear();
  cmd = GripperCommand{};
  cmd.mode = GripperCommandMode::kPosition;
  cmd.position = 0.5;
  cmd.current_limit = 50.0;
  R1_CHECK(!gripper.writeCommand(cmd, &error));
  R1_CHECK_CONTAINS(error, "current_limit");

  R1_CHECK_EQ(gripper.commandCount(), 0u);  // nothing was accepted
}

R1_TEST(mock_gripper_refuses_commands_while_disconnected) {
  MockGripper gripper;
  std::string error;
  R1_CHECK(!gripper.writeCommand(GripperCommand{}, &error));
  R1_CHECK_CONTAINS(error, "not connected");
}
