// ---------------------------------------------------------------------------
// Regression tests for the backend-level motion gate.
//
// Threat model: R1Arm is the only layer supposed to talk to a backend, but
// R1Arm::backend() hands out the backend pointer.  A caller could therefore
// drop straight to the backend and drive motors while bypassing every gate in
// R1Arm/SafetyMonitor.  The contract is that a real-hardware backend refuses
// *actuating* commands until it has been explicitly armed, and that only
// R1Arm::enableMotion() arms it.
//
// GatedRealBackend mirrors UnitreeArmBackend exactly (same MotionGate, same
// enforcement in writeCommand), so this exercises the production logic without
// needing the SDK or a serial port.
// ---------------------------------------------------------------------------
#include <memory>
#include <string>

#include "r1_robot/mock_arm.hpp"
#include "r1_robot/motion_gate.hpp"
#include "r1_robot/r1_arm.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

JointVector filled(double value) {
  JointVector v{};
  v.fill(value);
  return v;
}

class GatedRealBackend final : public IArmBackend {
 public:
  const char* name() const override { return "gated-real"; }
  bool isRealHardware() const override { return true; }

  bool connect(std::string* error) override {
    gate_.disarm();
    return mock_.connect(error);
  }
  void disconnect() override {
    gate_.disarm();
    mock_.disconnect();
  }
  bool isConnected() const override { return mock_.isConnected(); }
  bool hasValidCalibration(std::string* detail) const override {
    return mock_.hasValidCalibration(detail);
  }
  RobotLimits limits() const override { return mock_.limits(); }
  RobotState readState() const override { return mock_.readState(); }

  bool writeCommand(const ArmCommand& cmd) override {
    if (!gate_.allows(cmd)) {
      return false;
    }
    return mock_.writeCommand(cmd);
  }
  void stop() override { mock_.stop(); }

  void setMotionArmed(bool armed) override { gate_.set(armed); }
  bool motionArmed() const override { return gate_.armed(); }

  MockArm& mock() { return mock_; }

 private:
  MockArm mock_;
  MotionGate gate_;
};

}  // namespace

R1_TEST(motion_gate_is_fail_closed) {
  MotionGate gate;
  R1_CHECK(!gate.armed());

  // A stop can only remove energy: always allowed, even while disarmed.
  R1_CHECK(gate.allows(ArmCommand::stop()));

  // Every actuating command is refused while disarmed.
  R1_CHECK(!gate.allows(ArmCommand::position(filled(0.0), 10.0, 1.0)));
  R1_CHECK(!gate.allows(ArmCommand::zeroTorque()));

  gate.arm();
  R1_CHECK(gate.armed());
  R1_CHECK(gate.allows(ArmCommand::position(filled(0.0), 10.0, 1.0)));
  R1_CHECK(gate.allows(ArmCommand::stop()));

  gate.disarm();
  R1_CHECK(!gate.armed());
  R1_CHECK(!gate.allows(ArmCommand::zeroTorque()));
  R1_CHECK(gate.allows(ArmCommand::stop()));
}

R1_TEST(real_backend_refuses_a_direct_actuating_write_while_disarmed) {
  auto backend = std::make_shared<GatedRealBackend>();
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(!backend->motionArmed());

  // The bypass: a caller that holds the backend pointer calls it directly.
  R1_CHECK(!backend->writeCommand(ArmCommand::position(filled(0.0), 10.0, 1.0)));
  R1_CHECK(!backend->writeCommand(ArmCommand::zeroTorque()));
  R1_CHECK_EQ(backend->mock().commandCount(), 0u);

  // A stop must still go through (it is the way back to safety).
  R1_CHECK(backend->writeCommand(ArmCommand::stop()));
  R1_CHECK_EQ(backend->mock().commandCount(), 1u);
}

R1_TEST(r1_arm_is_the_only_thing_that_arms_the_backend) {
  auto backend = std::make_shared<GatedRealBackend>();
  backend->mock().setJointPositions(filled(0.18));
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(!backend->motionArmed());  // connect still arms nothing

  R1_CHECK(arm.enableMotion(true, &error));
  R1_CHECK(backend->motionArmed());
  R1_CHECK(arm.sendAction(ArmCommand::position(filled(0.2), 20.0, 3.0), &error));
  R1_CHECK_EQ(backend->mock().commandCount(), 1u);
}

R1_TEST(disabling_motion_disarms_the_backend_again) {
  auto backend = std::make_shared<GatedRealBackend>();
  backend->mock().setJointPositions(filled(0.18));
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));
  R1_CHECK(backend->motionArmed());

  arm.disableMotion();
  R1_CHECK(!backend->motionArmed());
  // Even a direct actuating write is refused again.
  R1_CHECK(!backend->writeCommand(ArmCommand::position(filled(0.2), 20.0, 3.0)));

  // Re-enabling re-arms it.
  R1_CHECK(arm.enableMotion(true, &error));
  R1_CHECK(backend->motionArmed());

  arm.emergencyStop();
  R1_CHECK(!backend->motionArmed());
}

R1_TEST(connecting_a_real_backend_without_the_master_switch_never_arms) {
  auto backend = std::make_shared<GatedRealBackend>();
  R1Arm arm(backend);  // allow_real_hardware defaults to false
  std::string error;

  R1_CHECK(!arm.connect(&error));
  R1_CHECK(!backend->motionArmed());
  R1_CHECK(!backend->isConnected());
}
