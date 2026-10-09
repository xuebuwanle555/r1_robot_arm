#pragma once

// ---------------------------------------------------------------------------
// r1_robot/limits.hpp
//
// Joint limits for the R1-7A.
//
// PROVENANCE
//   The default table below was read from the official URDF shipped with the
//   SDK:  urdf/R1_Robotic_Arm_V1.1-0915/.../urdf/R1_Robotic_Arm.urdf
//   and is therefore *URDF-verified*, not hardware-verified.
//
// TODO(bringup) 待验证:
//   * `max_torque` here is the URDF `effort` attribute.  Confirm it matches the
//     real actuator/PSU capability before using it as a safety envelope.
//   * `max_velocity` is the URDF `velocity` attribute.  Confirm against the
//     physical joint speed.
//   * Direction sign of each joint under gravity is NOT verified.
// ---------------------------------------------------------------------------

#include <array>
#include <string>

#include "r1_robot/types.hpp"

namespace r1_robot {

struct JointLimits {
  double lower = 0.0;         ///< [rad] URDF lower bound
  double upper = 0.0;         ///< [rad] URDF upper bound
  double max_velocity = 0.0;  ///< [rad/s]
  double max_torque = 0.0;    ///< [Nm]
};

struct RobotLimits {
  std::array<JointLimits, kNumJoints> joints{};
  std::array<std::string, kNumJoints> names{};
  std::string source;  ///< provenance string for logs / reports

  bool withinLimits(int joint, double q, double margin = 0.0) const;
};

/// Limits read from the official URDF (see provenance note above).
RobotLimits r1DefaultLimits();

/// True if `q` is inside [lower - margin, upper - margin] for every joint.
/// On failure `error` (when non-null) receives a joint-qualified message.
bool checkLimits(const JointVector& q, const RobotLimits& limits,
                 double margin, std::string* error);

/// Clamp every joint into the limits (in place).  Returns true if anything
/// had to be modified - callers should treat that as a safety event.
bool clampToLimits(JointVector& q, const RobotLimits& limits, double margin = 0.0);

}  // namespace r1_robot
