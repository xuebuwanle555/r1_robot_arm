#include <limits>
#include <string>

#include "r1_robot/limits.hpp"
#include "r1_robot/safety.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

RobotLimits limits() { return r1DefaultLimits(); }

RobotState goodState(std::uint64_t now) {
  RobotState s;
  s.connected = true;
  s.stamp_ns = now;
  s.valid.fill(true);
  s.q.fill(0.2);
  s.voltage.fill(48.0);
  return s;
}

/// Command that is valid against goodState() under the default SafetyConfig.
ArmCommand goodCommand() {
  JointVector q{};
  q.fill(0.2);
  return ArmCommand::position(q, 20.0, 3.0);
}

}  // namespace

R1_TEST(safety_accepts_a_healthy_state) {
  const std::uint64_t now = nowNs();
  const SafetyMonitor monitor;
  R1_CHECK_OK(monitor.checkState(goodState(now), now));
}

R1_TEST(safety_rejects_a_disconnected_state) {
  RobotState s = goodState(nowNs());
  s.connected = false;
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.not_connected");
}

R1_TEST(safety_rejects_missing_timestamp) {
  RobotState s = goodState(nowNs());
  s.stamp_ns = 0;
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.no_timestamp");
}

R1_TEST(safety_rejects_stale_feedback) {
  const std::uint64_t now = 10ULL * 1000ULL * 1000ULL * 1000ULL;
  RobotState s = goodState(now);
  s.stamp_ns = now - 1000ULL * 1000ULL * 1000ULL;  // 1 s old
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, now), "state.stale_feedback");
}

R1_TEST(safety_rejects_a_future_timestamp) {
  const std::uint64_t now = 10ULL * 1000ULL * 1000ULL * 1000ULL;
  RobotState s = goodState(now);
  s.stamp_ns = now + 1000ULL;  // 1 us in the future
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, now),
                 "state.timestamp_in_future");
}

R1_TEST(safety_rejects_partial_feedback) {
  RobotState s = goodState(nowNs());
  s.valid[4] = false;
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.joint_invalid");
}

R1_TEST(safety_rejects_motor_errors) {
  RobotState s = goodState(nowNs());
  s.error_code[1] = 0x8;  // transient undervoltage (SDK error bit)
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.motor_error");
}

R1_TEST(safety_rejects_undervoltage) {
  RobotState s = goodState(nowNs());
  s.voltage[6] = 20.0;
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()), "state.undervoltage");
}

R1_TEST(safety_skips_the_voltage_check_when_voltage_is_unknown) {
  // UnitreeArmBackend cannot report voltage (the SDK does not expose it) and
  // reports 0.  That must be treated as "unknown", not as 0 V.
  RobotState s = goodState(nowNs());
  s.voltage.fill(0.0);
  R1_CHECK_OK(SafetyMonitor{}.checkState(s, nowNs()));
}

R1_TEST(safety_always_allows_a_stop_command) {
  const SafetyMonitor monitor;
  RobotState s;  // completely disconnected / invalid
  R1_CHECK_OK(monitor.checkCommand(ArmCommand::stop(), s, limits(), nowNs(),
                                   /*calibration_ok=*/false));
}

R1_TEST(safety_blocks_actuation_on_a_bad_state) {
  RobotState s;  // disconnected
  const SafetyReport report = SafetyMonitor{}.checkCommand(
      goodCommand(), s, limits(), nowNs(), true);
  R1_CHECK(!report.ok());
  R1_CHECK_FAULT(report, "state.not_connected");
}

R1_TEST(safety_rejects_gains_outside_the_envelope) {
  const std::uint64_t now = nowNs();
  const SafetyMonitor monitor;
  const RobotState s = goodState(now);

  ArmCommand cmd = goodCommand();
  cmd.joints[3].kp = 1000.0;
  R1_CHECK_FAULT(monitor.checkCommand(cmd, s, limits(), now, true),
                 "cmd.gain_out_of_range");

  cmd = goodCommand();
  cmd.joints[0].kd = 50.0;
  R1_CHECK_FAULT(monitor.checkCommand(cmd, s, limits(), now, true),
                 "cmd.gain_out_of_range");
}

R1_TEST(safety_rejects_torque_above_the_envelope) {
  const std::uint64_t now = nowNs();
  ArmCommand cmd = goodCommand();
  cmd.joints[0].tau = 100.0;
  R1_CHECK_FAULT(
      SafetyMonitor{}.checkCommand(cmd, goodState(now), limits(), now, true),
      "cmd.torque_out_of_range");
}

R1_TEST(safety_rejects_targets_outside_the_soft_limits) {
  const std::uint64_t now = nowNs();
  ArmCommand cmd = goodCommand();
  cmd.joints[5].q = 3.0;  // J6 range is [-0.524, 0.524]
  R1_CHECK_FAULT(
      SafetyMonitor{}.checkCommand(cmd, goodState(now), limits(), now, true),
      "cmd.out_of_limits");
}

R1_TEST(safety_rejects_a_step_larger_than_one_cycle) {
  const std::uint64_t now = nowNs();
  RobotState s = goodState(now);
  s.q.fill(0.18);

  ArmCommand cmd = goodCommand();
  cmd.joints[5].q = 0.45;  // inside limits, but 0.27 rad away from measured

  const SafetyReport report =
      SafetyMonitor{}.checkCommand(cmd, s, limits(), now, true);
  R1_CHECK_FAULT(report, "cmd.step_too_large");
}

R1_TEST(safety_rejects_actuation_without_calibration) {
  const std::uint64_t now = nowNs();
  const SafetyReport report = SafetyMonitor{}.checkCommand(
      goodCommand(), goodState(now), limits(), now, /*calibration_ok=*/false);
  R1_CHECK_FAULT(report, "cmd.no_calibration");
}

R1_TEST(safety_rejects_non_finite_commands) {
  const std::uint64_t now = nowNs();
  ArmCommand cmd = goodCommand();
  cmd.joints[2].dq = std::numeric_limits<double>::infinity();
  R1_CHECK_FAULT(
      SafetyMonitor{}.checkCommand(cmd, goodState(now), limits(), now, true),
      "cmd.non_finite");
}

R1_TEST(safety_warns_about_a_fully_compliant_arm) {
  const std::uint64_t now = nowNs();
  const SafetyReport report = SafetyMonitor{}.checkCommand(
      ArmCommand::zeroTorque(), goodState(now), limits(), now, true);
  R1_CHECK(report.ok());
  R1_CHECK(report.hasWarnings());
  R1_CHECK_CONTAINS(report.toString(), "compliant");
}

R1_TEST(safety_motion_enable_gate) {
  const std::uint64_t now = nowNs();
  const SafetyMonitor monitor;

  R1_CHECK_OK(monitor.checkMotionEnable(goodState(now), now, true));

  R1_CHECK_FAULT(monitor.checkMotionEnable(goodState(now), now, false),
                 "enable.no_calibration");

  RobotState s = goodState(now);
  s.valid[0] = false;
  R1_CHECK_FAULT(monitor.checkMotionEnable(s, now, true),
                 "state.joint_invalid");
}

R1_TEST(safety_warns_when_the_arm_is_already_actuated) {
  const std::uint64_t now = nowNs();
  RobotState s = goodState(now);
  s.mode[2] = ControlMode::kFoc;
  const SafetyReport report = SafetyMonitor{}.checkMotionEnable(s, now, true);
  R1_CHECK(report.ok());
  R1_CHECK_CONTAINS(report.toString(), "enable.already_actuated");
}

// ---------------------------------------------------------------------------
// Regression: a missing feedback frame must fail closed for the *command*
// envelope even when the state report is configured to only warn about it.
// Before the fix, `require_all_joints_valid = false` (a supported, documented
// setting) let an actuating command through for a joint with no valid
// feedback, and the anti-step check was skipped for it too - so an arbitrarily
// large jump on an unmeasured joint was accepted.
// ---------------------------------------------------------------------------
R1_TEST(safety_rejects_actuation_for_a_joint_without_feedback) {
  const std::uint64_t now = nowNs();
  SafetyConfig cfg;
  cfg.require_all_joints_valid = false;  // the *state report* only warns...
  const SafetyMonitor monitor(cfg);

  RobotState s = goodState(now);
  s.q.fill(0.18);
  s.valid[5] = false;  // J6 position unknown

  const SafetyReport st = monitor.checkState(s, now);
  R1_CHECK(st.ok());            // ...so the state itself is not a fault...
  R1_CHECK(st.hasWarnings());

  // ...but an actuating command for J6 must be refused, and it must not
  // silently skip the anti-step check (0.45 is a 0.27 rad jump).
  ArmCommand cmd = goodCommand();
  cmd.joints[5].q = 0.45;
  const SafetyReport report =
      monitor.checkCommand(cmd, s, limits(), now, /*calibration_ok=*/true);
  R1_CHECK(!report.ok());
  R1_CHECK_FAULT(report, "cmd.no_feedback_for_joint");
}

// ---------------------------------------------------------------------------
// Disambiguation: stop / zero-torque / FOC-hold / motor-disable are NOT the
// same thing and must never be conflated.
//   * stop()        -> mode 0 on every joint (motor output disabled)
//   * zeroTorque()  -> mode 1, zero gains: compliant, does NOT hold gravity
//   * position(..., kp>0) -> mode 1 with authority: can hold
// ---------------------------------------------------------------------------
R1_TEST(stop_zero_torque_and_position_are_distinct) {
  const ArmCommand stop = ArmCommand::stop();
  const ArmCommand zero = ArmCommand::zeroTorque();
  const ArmCommand hold = ArmCommand::position(goodState(nowNs()).q, 20.0, 3.0);

  // stop is inert and carries no actuation.
  R1_CHECK(stop.isStop());
  R1_CHECK(!stop.hasActuation());
  for (const JointCommand& j : stop.joints) {
    R1_CHECK(j.mode == ControlMode::kStop);
    R1_CHECK_EQ(j.kp, 0.0);
    R1_CHECK_EQ(j.tau, 0.0);
  }

  // zero-torque is FOC (it powers the bridges) with no authority.
  R1_CHECK(!zero.isStop());
  R1_CHECK(zero.hasActuation());
  for (const JointCommand& j : zero.joints) {
    R1_CHECK(j.mode == ControlMode::kFoc);
    R1_CHECK_EQ(j.kp, 0.0);
    R1_CHECK_EQ(j.kd, 0.0);
    R1_CHECK_EQ(j.tau, 0.0);
  }

  // A position hold is FOC with position authority.
  R1_CHECK(hold.hasActuation());
  R1_CHECK(hold.joints[0].mode == ControlMode::kFoc);
  R1_CHECK(hold.joints[0].kp > 0.0);

  // A fully compliant command is allowed but warned about, while a stop is
  // never warned about.
  const std::uint64_t now = nowNs();
  const SafetyMonitor monitor;
  const SafetyReport zero_report =
      monitor.checkCommand(zero, goodState(now), limits(), now, true);
  R1_CHECK(zero_report.ok());
  R1_CHECK(zero_report.hasWarnings());
  R1_CHECK_CONTAINS(zero_report.toString(), "cmd.compliant_foc");

  const SafetyReport stop_report =
      monitor.checkCommand(stop, goodState(now), limits(), now, true);
  R1_CHECK(stop_report.ok());
  R1_CHECK(!stop_report.hasWarnings());
}
