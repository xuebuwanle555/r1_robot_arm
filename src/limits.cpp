#include "r1_robot/limits.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace r1_robot {
namespace {

// ---------------------------------------------------------------------------
// Source: urdf/R1_Robotic_Arm_V1.1-0915/.../urdf/R1_Robotic_Arm.urdf
//   joint J1 (idx 0) lower=-2.723 upper= 2.723 effort=60  velocity=18.7
//   joint J2 (idx 1) lower= 0     upper= 3.1416 effort=60  velocity=18.7
//   joint J3 (idx 2) lower=-0.349 upper= 3.491 effort=33  velocity=33.4
//   joint J4 (idx 3) lower= 0     upper= 3.1415 effort=33  velocity=33.4
//   joint J5 (idx 4) lower=-2.269 upper= 2.269 effort=33  velocity=33.4
//   joint J6 (idx 5) lower=-0.524 upper= 0.524 effort=10  velocity=37.7
//   joint J7 (idx 6) lower=-1.571 upper= 1.571 effort=10  velocity=37.7
//
// TODO(bringup) 待验证: confirm effort/velocity against the real actuators and
// the chosen power supply before using them as a torque/speed envelope.
// ---------------------------------------------------------------------------
struct RawLimit {
  double lower;
  double upper;
  double velocity;
  double effort;
  const char* name;
};

constexpr RawLimit kRawLimits[kNumJoints] = {
    {-2.723, 2.723, 18.7, 60.0, "J1"},
    {0.0, 3.1416, 18.7, 60.0, "J2"},
    {-0.349, 3.491, 33.4, 33.0, "J3"},
    {0.0, 3.1415, 33.4, 33.0, "J4"},
    {-2.269, 2.269, 33.4, 33.0, "J5"},
    {-0.524, 0.524, 37.7, 10.0, "J6"},
    {-1.571, 1.571, 37.7, 10.0, "J7"},
};

}  // namespace

bool RobotLimits::withinLimits(int joint, double q, double margin) const {
  if (joint < 0 || joint >= kNumJoints) {
    return false;
  }
  const JointLimits& l = joints[static_cast<std::size_t>(joint)];
  return q >= (l.lower - margin) && q <= (l.upper + margin);
}

RobotLimits r1DefaultLimits() {
  RobotLimits limits;
  for (int i = 0; i < kNumJoints; ++i) {
    const RawLimit& raw = kRawLimits[i];
    JointLimits& l = limits.joints[static_cast<std::size_t>(i)];
    l.lower = raw.lower;
    l.upper = raw.upper;
    l.max_velocity = raw.velocity;
    l.max_torque = raw.effort;
    limits.names[static_cast<std::size_t>(i)] = raw.name;
  }
  limits.source =
      "urdf/R1_Robotic_Arm_V1.1-0915/.../R1_Robotic_Arm.urdf (URDF-verified, "
      "hardware-UNVERIFIED)";
  return limits;
}

bool checkLimits(const JointVector& q, const RobotLimits& limits, double margin,
                 std::string* error) {
  for (int i = 0; i < kNumJoints; ++i) {
    const double qi = q[static_cast<std::size_t>(i)];
    const JointLimits& l = limits.joints[static_cast<std::size_t>(i)];
    if (!std::isfinite(qi)) {
      if (error) {
        std::ostringstream oss;
        oss << "joint " << i << " value is not finite";
        *error = oss.str();
      }
      return false;
    }
    if (qi < l.lower - margin || qi > l.upper + margin) {
      if (error) {
        std::ostringstream oss;
        oss << "joint " << i << " (" << limits.names[static_cast<std::size_t>(i)]
            << ") q=" << qi << " rad outside [" << l.lower << ", " << l.upper
            << "] (margin " << margin << ")";
        *error = oss.str();
      }
      return false;
    }
  }
  return true;
}

bool clampToLimits(JointVector& q, const RobotLimits& limits, double margin) {
  bool modified = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const JointLimits& l = limits.joints[static_cast<std::size_t>(i)];
    const double lo = l.lower + margin;
    const double hi = l.upper - margin;
    double& qi = q[static_cast<std::size_t>(i)];
    if (!std::isfinite(qi)) {
      qi = 0.0;
      modified = true;
      continue;
    }
    // Degenerate limits (margin wider than the range) collapse to the middle.
    const double clamped = (lo <= hi) ? std::min(std::max(qi, lo), hi)
                                      : 0.5 * (l.lower + l.upper);
    if (clamped != qi) {
      qi = clamped;
      modified = true;
    }
  }
  return modified;
}

}  // namespace r1_robot
