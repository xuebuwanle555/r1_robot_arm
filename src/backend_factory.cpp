#include "r1_robot/backend_factory.hpp"

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)

#include <sstream>

#include "r1_robot/mock_arm.hpp"
#include "r1_robot/unitree_backend.hpp"

#ifndef R1_ROBOT_SDK_VERSION
#define R1_ROBOT_SDK_VERSION "unknown"
#endif
#ifndef R1_ROBOT_SDK_COMMIT
#define R1_ROBOT_SDK_COMMIT "unknown"
#endif

namespace r1_robot {

ArmBackendPtr makeBackend(const BackendSelection& selection, std::string* error) {
  if (selection.kind == "mock") {
    return std::make_shared<MockArm>(MockArmConfig{});
  }

  if (selection.kind == "unitree") {
    UnitreeBackendConfig cfg;
    cfg.port = selection.port.empty() ? "/dev/ttyUSB3" : selection.port;
    cfg.baudrate = (selection.baudrate > 0) ? selection.baudrate : 6000000;
    cfg.calib_path = selection.calib_path.empty() ? "config/joint_calib.txt"
                                                  : selection.calib_path;
    // NOTE: creating the object does not touch the hardware.  Connecting is a
    // separate, explicitly gated step (R1Arm::connect).
    return std::make_shared<UnitreeArmBackend>(cfg);
  }

  if (error) {
    *error = "unknown backend '" + selection.kind +
             "' (expected 'mock' or 'unitree')";
  }
  return nullptr;
}

bool hasUnitreeBackend() { return true; }

std::string unitreeBackendInfo() {
  std::ostringstream oss;
  oss << "official unitreerobotics/r1-7a_sdk"
      << " version=" << R1_ROBOT_SDK_VERSION
      << " commit=" << R1_ROBOT_SDK_COMMIT;
  return oss.str();
}

}  // namespace r1_robot

#endif  // R1_ROBOT_WITH_UNITREE_BACKEND
