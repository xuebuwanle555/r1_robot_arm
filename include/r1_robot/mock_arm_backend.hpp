#pragma once

// ---------------------------------------------------------------------------
// r1_robot/mock_arm_backend.hpp
//
// A real-hardware-STYLE backend for offline tests: it reports isRealHardware()
// == true and enforces the EXACT same MotionGate + MotionControl contract as
// UnitreeArmBackend, but drives a MockJointBus instead of a serial port.
//
// Why: ControlService and the feedback-consistency tests must exercise the
// production gating (arm only via a token, refuse actuation while disarmed,
// atomic reception-stamped snapshots, reconnect never re-arms) without the SDK
// or hardware.  Keeping this in a header lets every test binary reuse it.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "r1_robot/backend.hpp"
#include "r1_robot/joint_bus.hpp"
#include "r1_robot/mock_joint_bus.hpp"
#include "r1_robot/motion_gate.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

/// Shared implementation of the reception-stamped snapshot->RobotState mapping.
/// Both UnitreeArmBackend and this mock use the same rules; factored here so the
/// offline test path and the hardware path cannot drift.
inline RobotState robotStateFromSnapshot(bool is_open,
                                         const JointBusSnapshot& snap,
                                         std::uint64_t read_now_ns) {
  RobotState out;
  if (!is_open) {
    out.connected = false;
    return out;
  }
  out.connected = true;
  std::uint64_t oldest = 0;
  bool any = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    const JointBusFeedback& fb = snap.feedback[si];
    const JointBusFeedbackStats& stats = snap.stats[si];
    out.q[si] = fb.q;
    out.dq[si] = fb.dq;
    out.tau[si] = fb.tau_est;
    out.mode[si] = fb.mode;
    out.error_code[si] = fb.error_code;
    out.valid[si] = stats.received;
    out.voltage[si] = 0.0;
    out.temperature_c[si] = 0.0;
    std::uint64_t stamp = 0;
    if (stats.received) {
      const std::uint64_t ref = stats.now_ns != 0 ? stats.now_ns : read_now_ns;
      stamp = (ref > stats.age_ns) ? (ref - stats.age_ns) : ref;
    }
    out.joint_stamp_ns[si] = stamp;
    if (stats.received && (!any || stamp < oldest)) {
      oldest = stamp;
      any = true;
    }
  }
  out.stamp_ns = any ? oldest : 0;
  return out;
}

class MockArmBackend final : public IArmBackend {
 public:
  explicit MockArmBackend(std::shared_ptr<MockJointBus> bus)
      : bus_(std::move(bus)) {}

  const char* name() const override { return "mock-real"; }
  bool isRealHardware() const override { return true; }

  bool connect(std::string* error) override {
    gate_.disarm();  // Fail-closed: a freshly connected backend is never armed.
    std::string err;
    if (!bus_->open("mock", 6000000, "mock_calib", &err)) {
      if (error) {
        *error = err;
      }
      return false;
    }
    connected_ = true;
    return true;
  }
  void disconnect() override {
    if (!connected_) {
      return;
    }
    gate_.disarm();
    bus_->stop();
    bus_->close();
    connected_ = false;
  }
  bool isConnected() const override { return connected_ && bus_->isOpen(); }

  bool hasValidCalibration(std::string* detail) const override {
    if (detail) {
      *detail = "mock calibration";
    }
    return true;
  }

  RobotLimits limits() const override { return r1DefaultLimits(); }

  RobotState readState() const override {
    const std::uint64_t rn = clock_ ? clock_() : nowNs();
    return robotStateFromSnapshot(bus_->isOpen(), bus_->snapshot(), rn);
  }

  bool writeCommand(const ArmCommand& cmd) override {
    if (!connected_) {
      return false;
    }
    if (!gate_.allows(cmd)) {  // Refuse actuation while disarmed (bypass guard).
      return false;
    }
    if (bus_->failWrite()) {  // wedged transport: the frame never reaches the arm
      return false;
    }
    std::array<JointBusCommand, kNumJoints> translated{};
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      const JointCommand& src = cmd.joints[si];
      JointBusCommand& dst = translated[si];
      dst.mode = src.mode;
      dst.q = src.q;
      dst.dq = src.dq;
      dst.tau = src.tau;
      dst.kp = src.kp;
      dst.kd = src.kd;
      dst.timed_out = cmd.timed_out;
    }
    bus_->step(translated);
    return true;
  }

  void stop() override {
    if (connected_) {
      bus_->stop();
    }
  }

  void setMotionArmed(bool armed, const MotionControl& control) override {
    (void)control;
    gate_.set(armed);
  }
  bool motionArmed() const override { return gate_.armed(); }

  bool probeFeedback(std::string* error, const MotionControl& control) override {
    (void)control;
    if (!connected_) {
      if (error) {
        *error = "refused: not connected";
      }
      return false;
    }
    std::array<JointBusCommand, kNumJoints> zero{};
    for (int i = 0; i < kNumJoints; ++i) {
      zero[static_cast<std::size_t>(i)].mode = ControlMode::kFoc;  // bridges on
    }
    bus_->step(zero);
    return true;
  }

  MockJointBus& bus() { return *bus_; }

  /// Align the backend's reception stamps with an external (virtual) clock so a
  /// ControlService sharing the same clock sees a consistent monotonic epoch.
  void setClock(std::function<std::uint64_t()> clock) {
    clock_ = std::move(clock);
  }

 private:
  std::shared_ptr<MockJointBus> bus_;
  MotionGate gate_;
  bool connected_ = false;
  std::function<std::uint64_t()> clock_;
};

}  // namespace r1_robot
