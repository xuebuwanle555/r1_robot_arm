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
#include <type_traits>

#include "r1_robot/mock_arm.hpp"
#include "r1_robot/motion_control.hpp"
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

  // Receives the token but cannot mint one (this test class is not a friend of
  // MotionControl); it only forwards what R1Arm hands it into the gate.
  void setMotionArmed(bool armed, const MotionControl& control) override {
    (void)control;
    gate_.set(armed);
  }
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

// ---------------------------------------------------------------------------
// Bypass regressions (Task 1).  These prove the control-permission boundary is
// enforced structurally, not by convention.
// ---------------------------------------------------------------------------

// The handle R1Arm hands out must be read-only: it is a `const IArmBackend`, so
// writeCommand()/stop()/setMotionArmed()/probeFeedback() (all non-const) are
// ill-formed through it.  A caller that reaches for the backend can poll state
// but physically cannot drive or arm the motors with it.
R1_TEST(r1_arm_backend_handle_is_read_only) {
  static_assert(std::is_const<std::remove_reference<
                    decltype(*std::declval<const r1_robot::R1Arm&>().backend())>::
                    type>::value,
                "R1Arm::backend() must return a const (read-only) handle");

  auto backend = std::make_shared<GatedRealBackend>();
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;  // else a real backend won't connect
  R1Arm arm(backend, config);
  std::string error;
  R1_CHECK(arm.connect(&error));

  // Read-only polling compiles and works through the handle.
  auto view = arm.backend();
  const RobotState s = view->readState();
  R1_CHECK(s.connected);
  R1_CHECK(!view->motionArmed());
  // (Uncommenting any of these does NOT compile: they are non-const members.)
  //   view->writeCommand(ArmCommand::stop());
  //   view->setMotionArmed(true, ...);   // and no token is constructible here
  //   view->stop();
}

// Once motion is enabled the backend gate is armed, so a caller that somehow
// holds a *non-const* backend can push an out-of-envelope frame straight through
// writeCommand(), skipping SafetyMonitor.  R1Arm::backend() no longer hands such
// a pointer out - this test documents exactly why that matters.
R1_TEST(enabled_gate_still_bypasses_safety_via_writecommand_so_backend_is_hidden) {
  auto backend = std::make_shared<GatedRealBackend>();
  backend->mock().setJointPositions(filled(0.18));
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(arm.enableMotion(true, &error));  // arms the backend gate

  // The legitimate path refuses a command outside max_joint_step_rad...
  const ArmCommand wild = ArmCommand::position(filled(3.0), 20.0, 3.0);
  R1_CHECK(!arm.sendAction(wild, &error));

  // ...but the raw backend (which R1Arm::backend() no longer exposes) would
  // accept it now that the gate is armed.  This is the hole we closed.
  R1_CHECK(backend->motionArmed());
  R1_CHECK(backend->writeCommand(wild));
}

// A business caller cannot fabricate the MotionControl token, so it cannot even
// name setMotionArmed()/probeFeedback().  We cannot construct a token here for
// the same reason - that IS the guarantee.  What we can assert is that the
// gated probe path never secretly arms motion.
R1_TEST(feedback_probe_does_not_arm_motion) {
  auto backend = std::make_shared<GatedRealBackend>();
  backend->mock().setJointPositions(filled(0.18));
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));

  // Refused without human confirmation...
  R1_CHECK(!arm.probeFeedbackForBringup(/*human_confirmed=*/false, &error));
  // ...and GatedReal drives no hardware so the interface probe refuses even when
  // confirmed.  Either way the motion gate stays disarmed and motion stays off.
  R1_CHECK(!arm.probeFeedbackForBringup(/*human_confirmed=*/true, &error));
  R1_CHECK(!backend->motionArmed());
  R1_CHECK(!arm.motionEnabled());

  // Without allow_real_hardware the backend is never even connected, so a
  // confirmed probe is refused outright.
  auto backend2 = std::make_shared<GatedRealBackend>();
  R1Arm arm2(backend2);  // allow_real_hardware = false
  std::string err2;
  R1_CHECK(!arm2.probeFeedbackForBringup(/*human_confirmed=*/true, &err2));
  R1_CHECK(!backend2->motionArmed());
}

// Motion enable is the ONLY thing that arms; the token cannot be obtained or
// forged by a caller that merely holds the R1Arm.
R1_TEST(enable_motion_is_the_only_armed_transition) {
  auto backend = std::make_shared<GatedRealBackend>();
  backend->mock().setJointPositions(filled(0.18));
  R1ArmConfig config;
  config.safety.allow_real_hardware = true;
  R1Arm arm(backend, config);

  std::string error;
  R1_CHECK(arm.connect(&error));
  R1_CHECK(!backend->motionArmed());

  // Failed enables must never leave the gate armed.
  R1_CHECK(!arm.enableMotion(/*human_confirmed=*/false, &error));
  R1_CHECK(!backend->motionArmed());

  R1_CHECK(arm.enableMotion(/*human_confirmed=*/true, &error));
  R1_CHECK(backend->motionArmed());
}
