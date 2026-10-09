// ---------------------------------------------------------------------------
// Feedback freshness / transport-failure regression tests.
//
// These drive UnitreeArmBackend through an injected fake bus, so delivery
// loss, CRC-style frame drops, partial joint failure, motor errors and port
// disconnects can be exercised deterministically - without hardware, a serial
// port, or the SDK.
//
// The central guarantee under test: a state is stamped with the time a frame
// was RECEIVED, never with the time it was read.  If read time were used, every
// freshness check would be vacuous.
// ---------------------------------------------------------------------------
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "r1_robot/joint_bus.hpp"
#include "r1_robot/r1_arm.hpp"
#include "r1_robot/safety.hpp"
#include "r1_robot/types.hpp"
#include "test_framework.hpp"

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)
#include "r1_robot/unitree_backend.hpp"
#endif

using namespace r1_robot;

namespace {

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

constexpr std::uint64_t kMs = 1000ULL * 1000ULL;

/// Deterministic in-process transport.  "Time" is a virtual monotonic clock;
/// a frame arrives only when the test says so.  This models, at the units that
/// matter, all the failure modes of the real bus:
///   * a dropped frame      -> receive() simply is not called for that joint
///   * a CRC-rejected frame -> same as a dropped frame (nothing updates)
///   * a dead joint         -> never receive() for it
///   * a dead port          -> close()
class FakeJointBus final : public IJointBus {
 public:
  bool open(const std::string& port, int baud, const std::string& calib,
            std::string* error) override {
    if (!open_succeeds) {
      if (error) {
        *error = "simulated open failure";
      }
      return false;
    }
    port_ = port;
    baud_ = baud;
    calib_ = calib;
    open_ = true;
    last_rx_.fill(0);
    packets_.fill(0);
    fb_ = {};
    // Give every joint one initial frame so a freshly opened bus is healthy,
    // unless the test disabled it.
    for (int i = 0; i < kNumJoints; ++i) {
      if (alive_[static_cast<std::size_t>(i)]) {
        last_rx_[static_cast<std::size_t>(i)] = clock_ns_;
      }
    }
    return true;
  }

  void close() override { open_ = false; }
  bool isOpen() const override { return open_; }

  void step(const std::array<JointBusCommand, kNumJoints>& cmds) override {
    ++step_count_;
    last_cmd_ = cmds;
  }

  std::array<JointBusFeedback, kNumJoints> feedback() const override {
    return fb_;
  }

  std::array<JointBusFeedbackStats, kNumJoints> feedbackStats() const override {
    std::array<JointBusFeedbackStats, kNumJoints> out{};
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      out[si].received = last_rx_[si] != 0;
      out[si].packets = packets_[si];
      out[si].age_ns =
          out[si].received && clock_ns_ >= last_rx_[si] ? clock_ns_ - last_rx_[si] : 0;
    }
    return out;
  }

  void stop() override { ++stop_count_; }
  void clearFaults() override {}
  void resetMotors() override {}

  // -- test control -------------------------------------------------------
  /// Virtual time passes (no frames are delivered by itself).
  void tick(std::uint64_t dt_ns) { clock_ns_ += dt_ns; }

  /// A valid (CRC-checked) frame for `joint` arrives now.
  void receive(int joint) {
    const std::size_t si = static_cast<std::size_t>(joint);
    last_rx_[si] = clock_ns_;
    ++packets_[si];
  }

  /// A frame for `joint` is delivered but rejected (bad CRC) - nothing updates.
  void receiveCorrupted(int /*joint*/) { /* deliberately a no-op */ }

  void setAlive(int joint, bool alive) {
    const std::size_t si = static_cast<std::size_t>(joint);
    alive_[si] = alive;
    if (!alive) {
      last_rx_[si] = 0;
    }
  }

  void setPosition(int joint, double q) {
    fb_[static_cast<std::size_t>(joint)].q = q;
  }
  void setErrorCode(int joint, std::uint32_t code) {
    fb_[static_cast<std::size_t>(joint)].error_code = code;
  }

  bool open_succeeds = true;
  std::uint64_t step_count_ = 0;
  std::uint64_t stop_count_ = 0;
  std::array<JointBusCommand, kNumJoints> last_cmd_{};

 private:
  bool open_ = false;
  std::string port_, calib_;
  int baud_ = 0;
  std::uint64_t clock_ns_ = 10ULL * 1000ULL * 1000ULL * 1000ULL;
  std::array<std::uint64_t, kNumJoints> last_rx_{};
  std::array<std::uint64_t, kNumJoints> packets_{};
  std::array<bool, kNumJoints> alive_ = {true, true, true, true, true, true, true};
  std::array<JointBusFeedback, kNumJoints> fb_{};
};

/// Writes a valid calibration file (the backend refuses to open without one).
std::string writeCalibFile() {
  const auto path =
      std::filesystem::temp_directory_path() / "r1_robot_freshness_calib.txt";
  std::ofstream out(path);
  out << "# test calibration\n"
      << "0 -1.0 1.0 -2.721\n"
      << "1  1.0 1.0 -0.05\n"
      << "2  1.0 1.0 -0.35011\n"
      << "3 -1.0 1.0 -0.025\n"
      << "4 -1.0 1.0 -2.27451\n"
      << "5  1.0 1.0 -0.52377\n"
      << "6 -1.0 1.0 -1.57638\n";
  out.close();
  return path.string();
}

struct Harness {
  std::shared_ptr<FakeJointBus> bus = std::make_shared<FakeJointBus>();
  UnitreeBackendConfig cfg;
  std::shared_ptr<UnitreeArmBackend> backend;
  std::string error;

  Harness() {
    cfg.port = "/dev/fake";
    cfg.calib_path = writeCalibFile();
    cfg.bus_factory = [this]() -> JointBusPtr { return bus; };
    backend = std::make_shared<UnitreeArmBackend>(cfg);
  }
  ~Harness() {
    std::error_code ec;
    std::filesystem::remove(cfg.calib_path, ec);
  }
};

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND

}  // namespace

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

R1_TEST(freshness_state_is_stamped_with_reception_time_not_read_time) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));

  const std::uint64_t before = nowNs();
  const RobotState fresh = h.backend->readState();
  const std::uint64_t after = nowNs();
  R1_CHECK(fresh.connected);
  R1_CHECK(fresh.allValid());
  // Freshly received: the stamp must be "now", i.e. inside the call window.
  R1_CHECK(fresh.stamp_ns >= before && fresh.stamp_ns <= after);
  R1_CHECK(fresh.joint_stamp_ns[0] >= before &&
           fresh.joint_stamp_ns[0] <= after);

  // 200 ms pass with NO frames delivered.  The stamp must stay at the last
  // RECEPTION time - if it were reset to "now" the freshness check below
  // could never fire.
  h.bus->tick(200 * kMs);
  const std::uint64_t now = nowNs();
  const RobotState stale = h.backend->readState();
  R1_CHECK(stale.stamp_ns < now);
  const double age_s = static_cast<double>(now - stale.stamp_ns) * 1e-9;
  R1_CHECK_NEAR(age_s, 0.2, 0.05);

  R1_CHECK_FAULT(SafetyMonitor{}.checkState(stale, now), "state.stale_feedback");
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(stale, now),
                 "state.joint_stale_feedback");
}

R1_TEST(freshness_feedback_stop_is_detected_at_the_configured_budget) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));
  const SafetyConfig cfg;  // max_feedback_age_s = 0.05

  h.bus->tick(40 * kMs);  // 40 ms: still inside the budget
  RobotState s = h.backend->readState();
  R1_CHECK_OK(SafetyMonitor{cfg}.checkState(s, nowNs()));

  h.bus->tick(20 * kMs);  // now 60 ms: outside the budget
  s = h.backend->readState();
  R1_CHECK_FAULT(SafetyMonitor{cfg}.checkState(s, nowNs()),
                 "state.stale_feedback");

  // A new frame restores freshness.
  for (int i = 0; i < kNumJoints; ++i) {
    h.bus->receive(i);
  }
  s = h.backend->readState();
  R1_CHECK_OK(SafetyMonitor{cfg}.checkState(s, nowNs()));
}

R1_TEST(freshness_partial_joint_loss_names_the_joint) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));
  const SafetyConfig cfg;

  // Joints 0..6 keep reporting; joint 4 goes silent.
  for (int step = 0; step < 4; ++step) {
    h.bus->tick(50 * kMs);
    for (int i = 0; i < kNumJoints; ++i) {
      if (i != 4) {
        h.bus->receive(i);
      }
    }
  }
  const RobotState s = h.backend->readState();
  // The other joints are fine, so the per-joint check is what catches it.
  R1_CHECK(s.valid[4]);  // a frame WAS received at some point (sticky evidence)
  const SafetyReport report = SafetyMonitor{cfg}.checkState(s, nowNs());
  R1_CHECK(!report.ok());
  R1_CHECK_FAULT(report, "state.joint_stale_feedback");
  R1_CHECK_CONTAINS(report.toString(), "joint 4");
}

R1_TEST(freshness_never_received_joint_is_invalid) {
  Harness h;
  h.bus->setAlive(2, false);  // joint 2 never transmits
  R1_CHECK(h.backend->connect(&h.error));

  const RobotState s = h.backend->readState();
  R1_CHECK(!s.valid[2]);
  R1_CHECK_EQ(s.joint_stamp_ns[2], 0u);
  R1_CHECK_EQ(s.invalidCount(), 1);

  const SafetyReport report = SafetyMonitor{}.checkState(s, nowNs());
  R1_CHECK_FAULT(report, "state.joint_invalid");
  R1_CHECK_CONTAINS(report.toString(), "joints: 2");
}

R1_TEST(freshness_corrupted_frames_do_not_refresh_the_timestamp) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));

  h.bus->tick(200 * kMs);
  // Every packet for every joint is rejected by CRC: nothing updates.
  for (int i = 0; i < kNumJoints; ++i) {
    h.bus->receiveCorrupted(i);
  }
  const RobotState s = h.backend->readState();
  R1_CHECK_FAULT(SafetyMonitor{}.checkState(s, nowNs()),
                 "state.stale_feedback");
}

R1_TEST(freshness_motor_error_is_reported) {
  Harness h;
  R1_CHECK(h.backend->connect(&h.error));
  h.bus->setErrorCode(5, 0x8);  // transient undervoltage
  const RobotState s = h.backend->readState();
  const SafetyReport report = SafetyMonitor{}.checkState(s, nowNs());
  R1_CHECK_FAULT(report, "state.motor_error");
  R1_CHECK_CONTAINS(report.toString(), "joint 5");
}

R1_TEST(freshness_port_disconnect_blocks_actuation) {
  Harness h;
  R1ArmConfig arm_cfg;
  arm_cfg.safety.allow_real_hardware = true;
  R1Arm arm(h.backend, arm_cfg);
  R1_CHECK(arm.connect(&h.error));
  R1_CHECK(arm.enableMotion(true, &h.error));

  // The transport dies.
  h.bus->close();

  const RobotState s = arm.state();
  R1_CHECK(!s.connected);
  R1_CHECK_FAULT(arm.checkState(), "state.not_connected");
  // Actuation is refused while the port is down; stop is still allowed.
  R1_CHECK(!arm.sendAction(ArmCommand::position(JointVector{0.1, 0.2, 0.1, 0.2, 0.0, 0.0, 0.0}, 20.0, 3.0),
                           &h.error));
  R1_CHECK_CONTAINS(h.error, "not connected");
  R1_CHECK(arm.sendAction(ArmCommand::stop(), &h.error));
}

R1_TEST(freshness_recovery_never_re_arms_motion) {
  Harness h;
  R1ArmConfig arm_cfg;
  arm_cfg.safety.allow_real_hardware = true;
  R1Arm arm(h.backend, arm_cfg);
  R1_CHECK(arm.connect(&h.error));
  R1_CHECK(arm.enableMotion(true, &h.error));
  R1_CHECK(h.backend->motionArmed());

  // The port drops and comes back.
  h.bus->close();
  arm.disconnect();
  R1_CHECK(!h.backend->motionArmed());
  R1_CHECK(h.backend->connect(&h.error));
  R1_CHECK(!h.backend->motionArmed());  // reconnecting must NOT arm

  R1ArmConfig arm_cfg2;
  arm_cfg2.safety.allow_real_hardware = true;
  R1Arm arm2(h.backend, arm_cfg2);
  R1_CHECK(arm2.connect(&h.error));
  R1_CHECK(!arm2.motionEnabled());
  R1_CHECK(!h.backend->motionArmed());
  // Even a direct actuating write is refused: reconnection re-arms nothing.
  R1_CHECK(!h.backend->writeCommand(
      ArmCommand::position(JointVector{0.1, 0.2, 0.1, 0.2, 0.0, 0.0, 0.0}, 20.0, 3.0)));
  R1_CHECK_EQ(h.bus->last_cmd_[0].mode, ControlMode::kStop);
}

R1_TEST(freshness_open_failure_leaves_the_backend_disarmed) {
  Harness h;
  h.bus->open_succeeds = false;
  std::string error;
  R1_CHECK(!h.backend->connect(&error));
  R1_CHECK_CONTAINS(error, "simulated open failure");
  R1_CHECK(!h.backend->isConnected());
  R1_CHECK(!h.backend->motionArmed());
}

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND

// ---------------------------------------------------------------------------
// The per-joint freshness rule itself (backend-independent).
// ---------------------------------------------------------------------------
R1_TEST(safety_reports_a_stale_joint_by_name) {
  const std::uint64_t now = 10ULL * 1000ULL * 1000ULL * 1000ULL;
  RobotState s;
  s.connected = true;
  s.stamp_ns = now;  // global stamp is fresh...
  s.valid.fill(true);
  s.q.fill(0.2);
  s.voltage.fill(48.0);
  s.joint_stamp_ns.fill(now);
  s.joint_stamp_ns[3] = now - 300ULL * 1000ULL * 1000ULL;  // ...but joint 3 is 300 ms old

  const SafetyReport report = SafetyMonitor{}.checkState(s, now);
  R1_CHECK(!report.ok());
  R1_CHECK_FAULT(report, "state.joint_stale_feedback");
  R1_CHECK_CONTAINS(report.toString(), "joint 3");
}

R1_TEST(safety_without_per_joint_stamps_uses_the_global_one_only) {
  const std::uint64_t now = 10ULL * 1000ULL * 1000ULL * 1000ULL;
  RobotState s;
  s.connected = true;
  s.stamp_ns = now - 1000ULL * 1000ULL * 1000ULL;  // 1 s old
  s.valid.fill(true);
  s.q.fill(0.2);
  s.voltage.fill(48.0);
  // joint_stamp_ns is all zero (backend does not publish per-joint stamps).

  const SafetyReport report = SafetyMonitor{}.checkState(s, now);
  R1_CHECK_FAULT(report, "state.stale_feedback");
  // No spurious per-joint faults for a backend that cannot provide the data.
  for (const SafetyIssue& issue : report.issues) {
    R1_CHECK(issue.code != "state.joint_stale_feedback");
  }
}
