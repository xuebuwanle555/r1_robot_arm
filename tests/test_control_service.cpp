// ---------------------------------------------------------------------------
// ControlService + feedback-consistency tests.
//
// All offline and deterministic: a MockJointBus (real-hardware-style, gated)
// drives a shared *virtual* monotonic clock, so the watchdog, feedback
// freshness and controlled-stop sequencing are exercised with no sleeps, no
// hardware and no reliance on mode-0 holding the arm.
//
// Central guarantees under test:
//   * the upper layer's silence latches a fault and brings the arm to a
//     controlled stop, then motor-disable, and it NEVER re-enables on its own;
//   * only ControlService arms the backend (it owns the sole write path);
//   * a value can never be paired with a mismatched reception time.
// ---------------------------------------------------------------------------
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "r1_robot/control_service.hpp"
#include "r1_robot/limits.hpp"
#include "r1_robot/mock_arm_backend.hpp"
#include "r1_robot/mock_joint_bus.hpp"
#include "r1_robot/safety.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

constexpr std::uint64_t kMs = 1000ULL * 1000ULL;

struct Harness {
  std::shared_ptr<MockJointBus> bus = std::make_shared<MockJointBus>();
  std::shared_ptr<MockArmBackend> backend = std::make_shared<MockArmBackend>(bus);
  ControlServiceConfig cfg;
  std::shared_ptr<ControlService> service;
  std::string error;

  Harness() {
    cfg.safety.allow_real_hardware = true;
    cfg.command_timeout_ns = 50 * kMs;
    cfg.stop_hold_ns = 20 * kMs;
    cfg.period_ns = kMs;
    cfg.clock = [b = bus] { return b->nowNs(); };
    backend->setClock([b = bus] { return b->nowNs(); });
    service = std::make_shared<ControlService>(backend, cfg);
  }

  void receiveAll() {
    for (int i = 0; i < kNumJoints; ++i) {
      bus->receive(i);
    }
  }
  /// The URDF zeros several joints' lower limits at 0 (J2/J4), so an all-zero
  /// pose is already outside the soft margin.  Park every joint at 0.1 rad,
  /// which is inside every joint's soft range, before commanding a hold.
  void setSafePose() {
    for (int i = 0; i < kNumJoints; ++i) {
      bus->setPosition(i, 0.1);
    }
  }
  /// Connect, park at a safe pose, and enable motion (human-confirmed).
  bool bringup() {
    if (!service->connect(&error)) {
      return false;
    }
    setSafePose();
    return service->enableMotion(true).ok;
  }
  /// Advance the shared clock one period, keep feedback fresh and refresh the
  /// upper-layer heartbeat.
  void alive(std::uint64_t dt_ns = kMs) {
    bus->tick(dt_ns);
    receiveAll();
    service->heartbeat();
    service->runCycle();
  }
};

JointVector filled(double v) {
  JointVector q{};
  q.fill(v);
  return q;
}

}  // namespace

// -- basic enable / software hold -------------------------------------------

R1_TEST(control_service_enable_requires_human_confirmation) {
  Harness h;
  R1_CHECK(h.service->connect(&h.error));
  R1_CHECK(h.service->state() == ServiceState::kStandby);

  R1_CHECK(!h.service->enableMotion(false).ok);
  R1_CHECK(!h.backend->motionArmed());

  auto r = h.service->enableMotion(true);
  R1_CHECK(r.ok);
  R1_CHECK(h.backend->motionArmed());
  R1_CHECK(h.service->state() == ServiceState::kSoftwareHold);
}

R1_TEST(control_service_hold_is_active_foc_not_a_drop) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();
  // Software hold must command FOC (mode 1) at the measured position: a gravity
  // loaded arm is caught, not dropped.  This is the "software-controlled hold"
  // state, distinct from a stop.
  const auto last = h.bus->lastCommands();
  R1_CHECK(last[0].mode == ControlMode::kFoc);
  R1_CHECK(last[0].kp > 0.0);
}

R1_TEST(control_service_watchdog_latches_on_silence_then_refuses_motion) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();  // healthy, upper layer alive

  // Upper layer dies: advance past the timeout, no heartbeat, feedback stays OK.
  h.bus->tick(60 * kMs);
  h.receiveAll();
  h.service->runCycle();

  R1_CHECK(h.service->faultLatched());
  R1_CHECK_EQ(h.service->lastFaultCode(), std::string("comm.watchdog_timeout"));
  R1_CHECK(h.service->state() == ServiceState::kControlledStop);

  // Controlled stop then motor-disable once it has held.
  h.bus->tick(30 * kMs);
  h.receiveAll();
  h.service->runCycle();
  R1_CHECK(h.service->state() == ServiceState::kMotorDisabled);
  R1_CHECK(!h.backend->motionArmed());  // write path closed

  // The upper layer comes back and tries to resume.  It must be refused: a
  // timeout NEVER auto-restores motion.
  h.service->heartbeat();
  h.service->submitCommand(ArmCommand::position(filled(0.1), 20.0, 3.0));
  R1_CHECK(h.service->state() == ServiceState::kMotorDisabled);
  R1_CHECK(!h.backend->motionArmed());
  R1_CHECK(!h.service->enableMotion(true).ok);  // must clear the latch first

  // Operator clears the fault; still not armed, still no motion.
  R1_CHECK(h.service->clearFault(&h.error));
  R1_CHECK(!h.service->faultLatched());
  R1_CHECK(h.service->state() == ServiceState::kStandby);
  R1_CHECK(!h.backend->motionArmed());
  // In standby the service emits no control frames at all.
  const std::uint64_t steps_before = h.bus->stepCount();
  h.bus->tick(kMs);
  h.receiveAll();
  h.service->heartbeat();
  h.service->runCycle();
  R1_CHECK_EQ(h.bus->stepCount(), steps_before);
  R1_CHECK(h.service->state() == ServiceState::kStandby);
}

R1_TEST(control_service_stale_feedback_latches) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();

  // Upper layer stays alive but the bus stops delivering frames.
  h.bus->tick(100 * kMs);
  h.service->heartbeat();
  h.service->runCycle();

  R1_CHECK(h.service->faultLatched());
  R1_CHECK_EQ(h.service->lastFaultCode(), std::string("state.stale_feedback"));
}

R1_TEST(control_service_motor_error_latches) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();

  h.bus->setErrorCode(3, 0x8);  // transient undervoltage on joint 3
  h.bus->tick(kMs);
  h.receiveAll();
  h.service->heartbeat();
  h.service->runCycle();

  R1_CHECK(h.service->faultLatched());
  R1_CHECK_EQ(h.service->lastFaultCode(), std::string("state.motor_error"));
}

R1_TEST(control_service_port_drop_latches) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();

  h.bus->close();  // transport dies
  h.bus->tick(kMs);
  h.service->heartbeat();
  h.service->runCycle();

  R1_CHECK(h.service->faultLatched());
  R1_CHECK_EQ(h.service->lastFaultCode(), std::string("state.not_connected"));
  R1_CHECK(h.service->state() == ServiceState::kControlledStop);

  // Once it has "held" it drops to motor-disabled and closes the write path.
  h.bus->tick(30 * kMs);
  h.service->runCycle();
  R1_CHECK(h.service->state() == ServiceState::kMotorDisabled);
  R1_CHECK(!h.backend->motionArmed());
}

R1_TEST(control_service_rejects_unsafe_trajectory) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();

  // A target beyond the soft upper limit of joint 0 (URDF max ~2.723); the
  // start is a safe pose so the refusal is clearly about the destination.
  std::vector<TrajWaypoint> wp(2);
  wp[0].t_ns = 0;
  wp[0].q = filled(0.1);
  wp[1].t_ns = 1000 * kMs;
  wp[1].q = filled(3.2);
  h.service->submitTrajectory(wp);

  // The up-front envelope check latches on submit; the cycle then holds.
  h.bus->tick(kMs);
  h.receiveAll();
  h.service->runCycle();

  R1_CHECK(h.service->faultLatched());
  R1_CHECK_EQ(h.service->lastFaultCode(), std::string("cmd.out_of_limits"));
}

R1_TEST(control_service_clear_fault_requires_a_fresh_enable) {
  Harness h;
  R1_CHECK(h.bringup());
  h.bus->tick(60 * kMs);  // no heartbeat -> watchdog
  h.receiveAll();
  h.service->runCycle();
  R1_CHECK(h.service->faultLatched());

  // Let the controlled stop reach motor-disable.
  h.bus->tick(30 * kMs);
  h.receiveAll();
  h.service->runCycle();

  R1_CHECK(h.service->clearFault(&h.error));
  R1_CHECK(!h.service->faultLatched());
  R1_CHECK(h.service->state() == ServiceState::kStandby);
  // Re-enable works again only with a fresh, human-confirmed enable.
  R1_CHECK(!h.service->enableMotion(false).ok);
  R1_CHECK(h.service->enableMotion(true).ok);
  R1_CHECK(h.backend->motionArmed());
}

R1_TEST(control_service_stop_states_are_distinct) {
  Harness h;
  R1_CHECK(h.bringup());
  h.alive();

  // software hold: active FOC, armed
  R1_CHECK(h.service->state() == ServiceState::kSoftwareHold);
  R1_CHECK(h.bus->lastCommands()[0].mode == ControlMode::kFoc);

  // request a controlled stop: decelerating hold, still armed
  h.service->requestControlledStop();
  R1_CHECK(h.service->state() == ServiceState::kControlledStop);
  h.bus->tick(kMs);
  h.receiveAll();
  h.service->runCycle();
  R1_CHECK(h.backend->motionArmed());  // caught by FOC while slowing

  // once held long enough: motor disabled + disarmed
  h.bus->tick(30 * kMs);
  h.receiveAll();
  h.service->runCycle();
  R1_CHECK(h.service->state() == ServiceState::kMotorDisabled);
  R1_CHECK(!h.backend->motionArmed());
  // A motor-disabled joint reports mode 0 in feedback, and a direct actuating
  // write (bypassing the service) is refused because the gate is disarmed.
  R1_CHECK(h.backend->readState().mode[0] == ControlMode::kStop);
  R1_CHECK(!h.backend->writeCommand(ArmCommand::position(filled(0.2), 20.0, 3.0)));
}

R1_TEST(control_service_reconnect_never_rearms) {
  Harness h;
  R1_CHECK(h.bringup());
  R1_CHECK(h.backend->motionArmed());
  h.service->stop();  // disarm on stop
  R1_CHECK(!h.backend->motionArmed());
  // A raw bus reconnect (no service enable path) must not arm anything.
  h.bus->close();
  h.bus->open("mock", 6000000, "calib", &h.error);
  R1_CHECK(!h.backend->motionArmed());
  R1_CHECK(!h.backend->writeCommand(ArmCommand::position(filled(0.2), 20.0, 3.0)));
}

// -- Task 3: atomic feedback snapshot ---------------------------------------

R1_TEST(snapshot_atomic_all_joints_share_one_clock) {
  auto bus = std::make_shared<MockJointBus>();
  std::string err;
  R1_CHECK(bus->open("p", 1, "c", &err));
  bus->tick(500 * kMs);
  bus->receive(0);
  bus->tick(120 * kMs);
  for (int i = 1; i < kNumJoints; ++i) {
    bus->receive(i);
  }
  bus->tick(37 * kMs);
  const JointBusSnapshot snap = bus->snapshot();
  // One capture instant for the whole snapshot: no per-joint clock skew.
  const std::uint64_t now0 = snap.stats[0].now_ns;
  for (int i = 0; i < kNumJoints; ++i) {
    R1_CHECK_EQ(snap.stats[static_cast<std::size_t>(i)].now_ns, now0);
  }
}

R1_TEST(snapshot_stale_joint_is_named_and_not_read_refreshed) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));
  R1_CHECK(h.backend->motionArmed() == false);

  h.bus->tick(10 * kMs);
  for (int i = 0; i < kNumJoints; ++i) {
    if (i != 4) {
      h.bus->receive(i);
    }
  }
  h.bus->tick(300 * kMs);  // joint 4 last received 310 ms ago at the newest now
  for (int i = 0; i < kNumJoints; ++i) {
    if (i != 4) {
      h.bus->receive(i);
    }
  }

  const RobotState s = h.backend->readState();
  R1_CHECK(s.valid[4]);  // sticky: a valid frame WAS received once
  // Reading the state must not refresh joint 4's reception stamp.
  const SafetyReport report = SafetyMonitor{}.checkState(s, h.bus->nowNs());
  R1_CHECK(!report.ok());
  R1_CHECK_FAULT(report, "state.joint_stale_feedback");
  R1_CHECK_CONTAINS(report.toString(), "joint 4");
}

R1_TEST(snapshot_crc_drop_is_stale_not_fresh) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));
  h.bus->tick(300 * kMs);
  for (int i = 0; i < kNumJoints; ++i) {
    h.bus->receiveCorrupted(i);  // arrives but CRC-rejected: nothing updates
  }
  const RobotState s = h.backend->readState();
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, h.bus->nowNs()),
                 "state.stale_feedback");
}

R1_TEST(snapshot_never_received_joint_is_invalid) {
  Harness h;
  h.bus->setAlive(2, false);  // joint 2 never transmits
  R1_CHECK(h.backend->connect(&h.error));
  const RobotState s = h.backend->readState();
  R1_CHECK(!s.valid[2]);
  R1_CHECK_EQ(s.invalidCount(), 1);
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, h.bus->nowNs()),
                 "state.joint_invalid");
}
