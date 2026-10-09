#pragma once

// ---------------------------------------------------------------------------
// r1_robot/motion_gate.hpp
//
// Fail-closed arming latch for a real-hardware backend.
//
// WHY THIS EXISTS
//   R1Arm is the only place that is *supposed* to talk to a backend, but a
//   caller that holds the backend pointer (R1Arm::backend()) could otherwise
//   call writeCommand() directly and drive motors while bypassing every gate in
//   R1Arm/SafetyMonitor.  Defense in depth: a backend that drives physical
//   motors must refuse every *actuating* command until it has been explicitly
//   armed, and must return to the disarmed state on stop/disconnect.
//
// CONTRACT
//   * Constructed DISARMED.
//   * A command that does not actuate (a pure mode-0 stop) is always allowed:
//     it can only ever remove energy, so it must never be blocked.
//   * Every actuating command is refused while disarmed.
//
// The latch is a single atomic word, so arming/disarming and the per-command
// check are safe from different threads.
// ---------------------------------------------------------------------------

#include <atomic>

#include "r1_robot/types.hpp"

namespace r1_robot {

class MotionGate {
 public:
  MotionGate() = default;
  MotionGate(const MotionGate&) = delete;
  MotionGate& operator=(const MotionGate&) = delete;

  /// Arm the motion path.  Must only be called by R1Arm after every precondition
  /// (human confirmation, allow_real_hardware, calibration, healthy state) holds.
  void arm() { armed_.store(true, std::memory_order_release); }

  /// Disarm the motion path.  Idempotent and safe to call at any time.
  void disarm() { armed_.store(false, std::memory_order_release); }

  /// Set the latch to `armed`.
  void set(bool armed) {
    armed_.store(armed, std::memory_order_release);
  }

  bool armed() const { return armed_.load(std::memory_order_acquire); }

  /// True when `cmd` may be written through this gate.  Stop commands pass even
  /// while disarmed; anything that actuates requires an armed gate.
  bool allows(const ArmCommand& cmd) const {
    return armed() || !cmd.hasActuation();
  }

 private:
  std::atomic<bool> armed_{false};
};

}  // namespace r1_robot
