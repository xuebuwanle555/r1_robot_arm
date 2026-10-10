#pragma once

// ---------------------------------------------------------------------------
// r1_robot/motion_control.hpp
//
// An unforgeable capability token.  Holding one proves you are the component
// that is allowed to open the hardware write path (arm the motion gate, run a
// zero-torque feedback probe).  Ordinary business code CANNOT create one, so it
// CANNOT disarm the motion lock for a real backend - the guarantee is enforced
// by the compiler, not by a documentation convention.
//
// WHY THIS EXISTS
//   R1Arm::backend() used to hand out a raw, *armable* backend pointer, so any
//   caller could do `arm.backend()->setMotionArmed(true)` and drive motors while
//   bypassing enableMotion()/SafetyMonitor.  Arming now requires this token,
//   whose only constructor is private and whose only friends are the two classes
//   that legitimately own the bus: R1Arm and ControlService.
//
// The token is copyable (so it can be passed by const& through the backend
// interface) but not constructible outside the friend set.  There is no public
// factory, no static mint(), and it is not an aggregate (the private user-declared
// constructor makes `{}`-initialisation ill-formed), so a non-friend cannot
// obtain the first instance by any means.
// ---------------------------------------------------------------------------

namespace r1_robot {

class R1Arm;
class ControlService;

class MotionControl {
 public:
  MotionControl(const MotionControl&) = default;
  MotionControl& operator=(const MotionControl&) = default;

 private:
  // Only the components that own the hardware write path may mint a token.
  friend class R1Arm;
  friend class ControlService;

  MotionControl() = default;
};

}  // namespace r1_robot
