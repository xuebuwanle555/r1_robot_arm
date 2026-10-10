// ---------------------------------------------------------------------------
// r1_robot/sdk_joint_bus.cpp
//
// Adapter: official SDK ArmMotorBus -> r1_robot IJointBus.
//
// Only compiled when the official SDK was found at configure time.
//
// IMPORTANT: this file and the SDK sources it wraps are built from a PATCHED
// COPY in the build tree (see cmake/R1SdkDependency.cmake and
// patches/0001-arm_motor_bus-feedback-stats.patch).  The submodule itself is
// never modified; the patch adds `ArmMotorBus::feedbackStats()`, which reports
// the reception time and packet count per joint.  Without it the upstream API
// only exposes a sticky `feedbackReady()` flag and no reception timestamp, so
// a caller cannot tell a healthy bus from one that stopped delivering frames.
// ---------------------------------------------------------------------------
#include "r1_robot/joint_bus.hpp"

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

#include <array>
#include <utility>

#include "arm_motor_bus.h"

namespace r1_robot {
namespace {

ControlMode modeFromMotor(std::uint8_t motor_mode) {
  return (motor_mode == 1) ? ControlMode::kFoc : ControlMode::kStop;
}

std::uint8_t motorFromMode(ControlMode mode) {
  return (mode == ControlMode::kFoc) ? 1u : 0u;
}

/// Thin wrapper around the official bus.  Deliberately dumb: it converts types
/// and forwards calls.  All policy (calibration pre-check, arming, freshness
/// bookkeeping) lives in UnitreeArmBackend / R1Arm.
class SdkJointBus final : public IJointBus {
 public:
  bool open(const std::string& port, int baudrate,
            const std::string& calib_path, std::string* error) override {
    // ArmMotorBus::open() calls std::exit(1) when the calibration file is
    // missing or incomplete.  UnitreeArmBackend validates the file first, so
    // reaching this point means it is usable; a library must never kill the
    // host process on its own.
    if (!bus_.open(port, baudrate, calib_path)) {
      if (error) {
        *error = "failed to open serial port '" + port + "' at " +
                 std::to_string(baudrate) + " baud";
      }
      return false;
    }
    return true;
  }

  void close() override { bus_.close(); }
  bool isOpen() const override { return bus_.isOpen(); }

  void step(const std::array<JointBusCommand, kNumJoints>& cmds) override {
    std::array<ArmJointCmd, kArmJointCount> translated{};
    for (int i = 0; i < kNumJoints; ++i) {
      const JointBusCommand& src = cmds[static_cast<std::size_t>(i)];
      ArmJointCmd& dst = translated[static_cast<std::size_t>(i)];
      dst.mode = motorFromMode(src.mode);
      dst.q = static_cast<float>(src.q);  // URDF frame: converted inside the SDK
      dst.dq = static_cast<float>(src.dq);
      dst.tau = static_cast<float>(src.tau);
      dst.kp = static_cast<float>(src.kp);
      dst.kd = static_cast<float>(src.kd);
      dst.timed_out = src.timed_out;
    }
    bus_.step(translated);
  }

  std::array<JointBusFeedback, kNumJoints> feedback() const override {
    std::array<JointBusFeedback, kNumJoints> out{};
    const std::array<ArmJointState, kArmJointCount> states = bus_.states();
    for (int i = 0; i < kNumJoints; ++i) {
      const ArmJointState& s = states[static_cast<std::size_t>(i)];
      JointBusFeedback& dst = out[static_cast<std::size_t>(i)];
      dst.q = s.q;
      dst.dq = s.dq;
      dst.tau_est = s.tau_est;
      dst.mode = modeFromMotor(s.mode);
      dst.timeout = (s.timeout != 0);
      dst.error_code = s.error_code;
    }
    return out;
  }

  std::array<JointBusFeedbackStats, kNumJoints> feedbackStats() const override {
    // PATCHED SDK API: real per-joint reception time and packet count.
    std::array<JointBusFeedbackStats, kNumJoints> out{};
    const std::array<ArmMotorBus::FeedbackStats, kArmJointCount> stats =
        bus_.feedbackStats();
    for (int i = 0; i < kNumJoints; ++i) {
      const ArmMotorBus::FeedbackStats& s = stats[static_cast<std::size_t>(i)];
      JointBusFeedbackStats& dst = out[static_cast<std::size_t>(i)];
      dst.received = s.received;
      dst.packets = s.packets;
      dst.age_ns = s.age_ns;
      dst.now_ns = s.now_ns;
    }
    return out;
  }

  /// Atomic: values + reception evidence from a SINGLE patched-SDK lock pass,
  /// so a stale joint value can never be paired with a freshly-computed age.
  JointBusSnapshot snapshot() const override {
    const ArmMotorBus::FeedbackSnapshot raw = bus_.feedbackSnapshot();
    JointBusSnapshot out;
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      const ArmJointState& s = raw.states[si];
      JointBusFeedback& fb = out.feedback[si];
      fb.q = s.q;
      fb.dq = s.dq;
      fb.tau_est = s.tau_est;
      fb.mode = modeFromMotor(s.mode);
      fb.timeout = (s.timeout != 0);
      fb.error_code = s.error_code;

      const ArmMotorBus::FeedbackStats& st = raw.stats[si];
      JointBusFeedbackStats& dst = out.stats[si];
      dst.received = st.received;
      dst.packets = st.packets;
      dst.age_ns = st.age_ns;
      dst.now_ns = st.now_ns;
    }
    return out;
  }

  void stop() override { bus_.stop(); }
  void clearFaults() override { bus_.clearFaults(); }
  void resetMotors() override { bus_.resetMotors(); }

 private:
  ArmMotorBus bus_;
};

}  // namespace

bool hasSdkJointBus() { return true; }

JointBusPtr makeSdkJointBus() { return std::make_shared<SdkJointBus>(); }

}  // namespace r1_robot

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND
