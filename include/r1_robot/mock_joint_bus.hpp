#pragma once

// ---------------------------------------------------------------------------
// r1_robot/mock_joint_bus.hpp
//
// A deterministic, in-process IJointBus used to exercise the safety-relevant
// transport behaviour OFFLINE - no hardware, no serial port, no sleeping.
//
// It models, in the units that matter, every failure mode of the real bus:
//   * a dropped / half packet   -> receive() simply is not called that cycle
//   * a CRC-rejected frame      -> receiveCorrupted() changes nothing
//   * a dead joint              -> setAlive(j, false): it never receives
//   * a dead port               -> close()
//   * recovery / reconnect       -> open() again (does NOT re-arm anything)
//
// It backs a GateBackedBusArm (mock_arm_backend.hpp), i.e. a real-hardware-style
// backend whose ONLY write path is a MotionGate, so ControlService's watchdog and
// fault-latching can be driven against a backend that behaves like the Unitree
// one but is fully testable.
//
// "Time" is a virtual monotonic clock advanced only by tick(); snapshots capture
// values + reception evidence together under one lock, so a stale value can never
// be paired with a fresh age.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include "r1_robot/joint_bus.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

class MockJointBus final : public IJointBus {
 public:
  static constexpr std::uint64_t kInitialClockNs = 10ULL * 1000ULL * 1000ULL * 1000ULL;

  bool open(const std::string& port, int baudrate, const std::string& calib_path,
            std::string* error) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fail_open_) {
      if (error) {
        *error = "simulated open failure";
      }
      return false;
    }
    port_ = port;
    baud_ = baudrate;
    calib_ = calib_path;
    is_open_ = true;
    last_rx_.fill(0);
    packets_.fill(0);
    fb_ = {};
    // A freshly opened bus is healthy unless a joint was marked dead: give every
    // alive joint one initial frame at the current virtual time.
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      if (alive_[si]) {
        last_rx_[si] = clock_ns_;
        packets_[si] = 1;
      }
    }
    return true;
  }

  void close() override {
    std::lock_guard<std::mutex> lock(mutex_);
    is_open_ = false;
  }
  bool isOpen() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_open_;
  }

  void step(const std::array<JointBusCommand, kNumJoints>& cmds) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_open_) {
      return;
    }
    ++step_count_;
    last_cmds_ = cmds;
    // Echo the commanded position back as feedback (a real motor would report
    // its measured position; for the mock a plain echo is enough to let the
    // ControlService move toward a setpoint).  mode follows the command.
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      fb_[si].mode = cmds[si].mode;
      fb_[si].q = cmds[si].q;
      fb_[si].dq = cmds[si].dq;
      fb_[si].tau_est = cmds[si].tau;
      fb_[si].timeout = cmds[si].timed_out;
    }
  }

  std::array<JointBusFeedback, kNumJoints> feedback() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return fb_;
  }

  std::array<JointBusFeedbackStats, kNumJoints> feedbackStats() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return statsLocked();
  }

  /// Atomic: values + evidence from ONE clock reading (mirrors the patched SDK).
  JointBusSnapshot snapshot() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    JointBusSnapshot snap;
    snap.feedback = fb_;
    snap.stats = statsLocked();
    return snap;
  }

  void stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stop_count_;
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      fb_[si].mode = ControlMode::kStop;
      fb_[si].dq = 0.0;
    }
  }
  void clearFaults() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++clear_count_;
    for (int i = 0; i < kNumJoints; ++i) {
      fb_[static_cast<std::size_t>(i)].error_code = 0;
    }
  }
  void resetMotors() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++reset_count_;
  }

  // -- test control ---------------------------------------------------------
  /// Advance the virtual monotonic clock (no frames arrive by themselves).
  void tick(std::uint64_t dt_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    clock_ns_ += dt_ns;
  }

  std::uint64_t nowNs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clock_ns_;
  }

  /// A valid (CRC-checked) frame for `joint` arrives at the current time.
  void receive(int joint) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t si = static_cast<std::size_t>(joint);
    if (si < kNumJoints && alive_[si] && is_open_) {
      last_rx_[si] = clock_ns_;
      ++packets_[si];
    }
  }

  /// A frame arrives but is rejected (bad CRC): nothing updates.
  void receiveCorrupted(int /*joint*/) {}

  /// Mark a joint dead (it never transmits again).
  void setAlive(int joint, bool alive) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t si = static_cast<std::size_t>(joint);
    alive_[si] = alive;
    if (!alive) {
      last_rx_[si] = 0;
      packets_[si] = 0;
    }
  }

  void setPosition(int joint, double q) {
    std::lock_guard<std::mutex> lock(mutex_);
    fb_[static_cast<std::size_t>(joint)].q = q;
  }
  void setVelocity(int joint, double dq) {
    std::lock_guard<std::mutex> lock(mutex_);
    fb_[static_cast<std::size_t>(joint)].dq = dq;
  }
  void setErrorCode(int joint, std::uint32_t code) {
    std::lock_guard<std::mutex> lock(mutex_);
    fb_[static_cast<std::size_t>(joint)].error_code = code;
  }

  void setFailOpen(bool fail) { fail_open_ = fail; }

  std::uint64_t stepCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return step_count_;
  }
  std::uint64_t stopCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_count_;
  }
  std::array<JointBusCommand, kNumJoints> lastCommands() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_cmds_;
  }

 private:
  std::array<JointBusFeedbackStats, kNumJoints> statsLocked() const {
    std::array<JointBusFeedbackStats, kNumJoints> out{};
    for (int i = 0; i < kNumJoints; ++i) {
      const std::size_t si = static_cast<std::size_t>(i);
      out[si].received = last_rx_[si] != 0;
      out[si].packets = packets_[si];
      out[si].now_ns = clock_ns_;
      out[si].age_ns = (out[si].received && clock_ns_ >= last_rx_[si])
                           ? (clock_ns_ - last_rx_[si])
                           : 0;
    }
    return out;
  }

  mutable std::mutex mutex_;
  bool is_open_ = false;
  bool fail_open_ = false;
  std::string port_, calib_;
  int baud_ = 0;
  std::uint64_t clock_ns_ = kInitialClockNs;
  std::uint64_t step_count_ = 0;
  std::uint64_t stop_count_ = 0;
  std::uint64_t clear_count_ = 0;
  std::uint64_t reset_count_ = 0;
  std::array<JointBusFeedback, kNumJoints> fb_{};
  std::array<JointBusCommand, kNumJoints> last_cmds_{};
  std::array<std::uint64_t, kNumJoints> last_rx_{};
  std::array<std::uint64_t, kNumJoints> packets_{};
  std::array<bool, kNumJoints> alive_ = {true, true, true, true, true, true, true};
};

}  // namespace r1_robot
