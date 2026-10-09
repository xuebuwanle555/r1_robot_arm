#pragma once

// ---------------------------------------------------------------------------
// r1_robot/backend_factory.hpp
//
// Only compiled when R1_ROBOT_WITH_UNITREE_BACKEND is defined.
// The factory NEVER creates a real-hardware backend implicitly:
//   * "mock"    -> always available,
//   * "unitree" -> requires an explicit request AND, at connect time,
//                  SafetyConfig::allow_real_hardware.
// ---------------------------------------------------------------------------

#include <string>

#include "r1_robot/backend.hpp"
#include "r1_robot/config.hpp"

namespace r1_robot {

/// Create a backend from a selection.  Returns nullptr and fills `error` for an
/// unknown kind or when the requested backend was not compiled in.
ArmBackendPtr makeBackend(const BackendSelection& selection, std::string* error);

/// True when this build contains the real-hardware backend.
bool hasUnitreeBackend();

/// Human readable description of the compiled-in SDK pin, for logs / reports.
std::string unitreeBackendInfo();

}  // namespace r1_robot
