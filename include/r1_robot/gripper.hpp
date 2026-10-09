#pragma once

// ---------------------------------------------------------------------------
// r1_robot/gripper.hpp
//
// Gripper abstraction.
//
// IMPORTANT: the reBot-style dual rack-and-pinion gripper is driven by a
// DAMIAO DM4310 over CAN.  It is a *separate device* from the R1-7A arm and
// MUST NOT be assumed to be reachable through the Unitree serial bus.
// Phase 1 therefore ships only:
//   * IGripper        - the interface upper layers program against,
//   * MockGripper     - offline implementation for tests,
//   * Dm4310Gripper   - a fail-closed placeholder (NOT implemented).
//
// 待验证 (see HARDWARE_BRINGUP.md) - none of these is confirmed yet:
//   * CAN interface / adapter (can0? socketcan? vendor USB-CAN?) and bitrate
//   * DM4310 CAN id(s), master id, MIT-mode parameter ranges
//   * gear ratio between motor and rack travel (position units!)
//   * travel limits (fully closed / fully open), current and velocity limits
//   * homing procedure, whether an absolute encoder zero exists
//   * thermal model and stall-current protection
// Positions are therefore kept as a *normalized* opening in [0, 1] and every
// limit defaults to a conservative placeholder.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>

namespace r1_robot {

enum class GripperCommandMode {
  kStop = 0,     ///< release output
  kPosition = 1, ///< go to `position`
  kCurrent = 2,  ///< apply `current` (force control)
};

const char* toString(GripperCommandMode mode);

struct GripperLimits {
  double position_min = 0.0;  ///< normalized opening, 0 = closed
  double position_max = 1.0;  ///< normalized opening, 1 = open
  double max_current = 0.0;   ///< [A]  TODO(bringup): verify
  double max_velocity = 0.0;  ///< [1/s] TODO(bringup): verify
  double default_current_limit = 0.0;  ///< [A] TODO(bringup): verify
  std::string source;         ///< provenance of the numbers above
};

struct GripperCommand {
  GripperCommandMode mode = GripperCommandMode::kStop;
  double position = 0.0;       ///< normalized [0, 1], used by kPosition
  double current = 0.0;        ///< [A], used by kCurrent
  double velocity_limit = 0.0; ///< [1/s], 0 = backend default
  double current_limit = 0.0;  ///< [A], 0 = backend default
};

struct GripperState {
  double position = 0.0;       ///< normalized [0, 1]
  double velocity = 0.0;       ///< [1/s]
  double current = 0.0;        ///< [A]
  double temperature_c = 0.0;  ///< [degC]
  bool valid = false;
  bool connected = false;
  std::uint32_t error_code = 0;
  std::uint64_t stamp_ns = 0;
};

class IGripper {
 public:
  virtual ~IGripper() = default;

  virtual const char* name() const = 0;
  virtual bool isRealHardware() const = 0;

  virtual bool connect(std::string* error) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;

  virtual GripperLimits limits() const = 0;
  virtual GripperState state() const = 0;

  /// Validates against `limits()` and refuses (returns false) on violation.
  virtual bool writeCommand(const GripperCommand& cmd, std::string* error) = 0;
  virtual void stop() = 0;
};

/// Offline gripper: no CAN, no hardware.  Useful for tests and for pipelines
/// that must run on a workstation.
class MockGripper final : public IGripper {
 public:
  explicit MockGripper(GripperLimits limits = GripperLimits{});

  const char* name() const override { return "mock-gripper"; }
  bool isRealHardware() const override { return false; }

  bool connect(std::string* error) override;
  void disconnect() override;
  bool isConnected() const override;

  GripperLimits limits() const override { return limits_; }
  GripperState state() const override;

  bool writeCommand(const GripperCommand& cmd, std::string* error) override;
  void stop() override;

  // test helpers
  void setState(const GripperState& state);
  GripperCommand lastCommand() const;
  std::uint64_t commandCount() const;

 private:
  GripperLimits limits_;
  GripperState state_{};
  GripperCommand last_cmd_{};
  std::uint64_t command_count_ = 0;
  bool connected_ = false;
};

/// DM4310 + reBot-style rack-and-pinion gripper.
///
/// PHASE 1 PLACEHOLDER - FAILS CLOSED.
/// Every method refuses; connect() reports exactly what must be verified
/// before this class may be implemented.  It exists so that upper layers can
/// already be written against a stable interface without any risk of driving
/// the physical gripper.
class Dm4310Gripper final : public IGripper {
 public:
  const char* name() const override { return "dm4310-gripper"; }
  bool isRealHardware() const override { return true; }

  bool connect(std::string* error) override;
  void disconnect() override;
  bool isConnected() const override { return false; }

  GripperLimits limits() const override;
  GripperState state() const override;

  bool writeCommand(const GripperCommand& cmd, std::string* error) override;
  void stop() override;
};

}  // namespace r1_robot
