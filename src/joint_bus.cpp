// ---------------------------------------------------------------------------
// r1_robot/joint_bus.cpp
//
// Fallback definitions for builds WITHOUT the official SDK backend.  In such a
// build the SDK-backed bus cannot exist, and the factory must fail closed
// instead of silently returning something unusable.
// ---------------------------------------------------------------------------
#include "r1_robot/joint_bus.hpp"

#if !defined(R1_ROBOT_WITH_UNITREE_BACKEND)

namespace r1_robot {

bool hasSdkJointBus() { return false; }

JointBusPtr makeSdkJointBus() { return nullptr; }

}  // namespace r1_robot

#endif  // !R1_ROBOT_WITH_UNITREE_BACKEND
