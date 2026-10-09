#pragma once

// ---------------------------------------------------------------------------
// r1_robot/backend.hpp
//
// Hardware backend abstraction.  A backend is a thin, dumb I/O adapter:
//   * it does NOT validate commands (R1Arm + SafetyMonitor do that),
//   * it does NOT decide whether real hardware may move (R1Arm does that),
//   * it MUST report honestly whether it is real hardware.
//
// Implementations:
//   MockArm             - offline simulation (always available)
//   UnitreeArmBackend   - real R1-7A via the official SDK (optional build)
// ---------------------------------------------------------------------------

#include <memory>
#include <string>

#include "r1_robot/limits.hpp"
#include "r1_robot/types.hpp"

namespace r1_robot {

class IArmBackend {
 public:
  virtual ~IArmBackend() = default;

  /// Stable identifier used by factory / logs, e.g. "mock", "unitree".
  virtual const char* name() const = 0;

  /// True only for backends that can drive physical motors.  R1Arm refuses to
  /// connect such a backend unless SafetyConfig::allow_real_hardware is set.
  virtual bool isRealHardware() const = 0;

  /// Open the transport.  Returns false and fills `error` on failure.
  virtual bool connect(std::string* error) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;

  /// True when a usable joint calibration is available.  Real motion is
  /// forbidden otherwise.
  virtual bool hasValidCalibration(std::string* detail) const = 0;

  /// Limits advertised by this backend (URDF frame, SI units).
  virtual RobotLimits limits() const = 0;

  /// Latest feedback snapshot.  `connected == false` when not connected.
  virtual RobotState readState() const = 0;

  /// Push one command frame.  Pre-validated by R1Arm; backend only translates
  /// it (frame conversion, packing, bus write) and may still refuse.
  ///
  /// A backend that drives real hardware MUST additionally refuse every
  /// actuating command while the motion gate is disarmed (defense in depth
  /// against a caller that bypasses R1Arm).  A pure stop command must always be
  /// accepted.  See r1_robot/motion_gate.hpp.
  virtual bool writeCommand(const ArmCommand& cmd) = 0;

  /// Best-effort mode-0 stop.  NOTE: this is NOT a brake - it cannot hold an
  /// arm that is loaded by gravity (documented in HARDWARE_BRINGUP.md).
  virtual void stop() = 0;

  /// Arm/disarm the backend-level motion gate.  R1Arm is the only legitimate
  /// caller: it arms in enableMotion() and disarms in
  /// disableMotion()/stop()/emergencyStop()/disconnect().
  ///
  /// Default implementation is a no-op; backends that do not drive hardware do
  /// not need a gate (see motionArmed()).
  virtual void setMotionArmed(bool armed) { (void)armed; }

  /// True when this backend will accept an *actuating* command.
  ///
  /// Default: ``!isRealHardware()`` - a backend that does not drive real
  /// motors is always "armed" so R1Arm needs no special casing, while a real
  /// backend that does not override this reports `false` (fail-closed).
  virtual bool motionArmed() const { return !isRealHardware(); }
};

using ArmBackendPtr = std::shared_ptr<IArmBackend>;

/// Always-available, hardware-independent backend (see mock_arm.hpp).
ArmBackendPtr makeMockBackend();

}  // namespace r1_robot
