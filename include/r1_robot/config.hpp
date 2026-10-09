#pragma once

// ---------------------------------------------------------------------------
// r1_robot/config.hpp
//
// Minimal, dependency-free configuration reader.
//
// Format:  "key = value"  with '#' comments and blank lines.
// Deliberately not YAML: phase 1 must build and run offline with no extra
// packages, and the config surface is tiny.  If the project later moves to
// YAML/ROS2 parameters, only this file and its test need to change.
// ---------------------------------------------------------------------------

#include <map>
#include <string>
#include <vector>

#include "r1_robot/r1_arm.hpp"

namespace r1_robot {

class Config {
 public:
  Config() = default;

  /// Parse a file.  Returns false and fills `error` on I/O or syntax failure.
  static bool fromFile(const std::string& path, Config* out, std::string* error);

  /// Parse from a string (used by tests).
  static bool fromString(const std::string& text, Config* out,
                         std::string* error);

  bool has(const std::string& key) const;

  std::string getString(const std::string& key, const std::string& fallback) const;
  double getDouble(const std::string& key, double fallback) const;
  int getInt(const std::string& key, int fallback) const;
  bool getBool(const std::string& key, bool fallback) const;

  /// All key/value pairs, sorted, for logging.
  std::map<std::string, std::string> values() const { return values_; }
  std::string dump() const;

 private:
  std::map<std::string, std::string> values_;
};

/// Build R1ArmConfig from a Config, starting from safe defaults.
/// Unknown keys are ignored; malformed values fall back to the default and are
/// reported through `warnings`.
R1ArmConfig armConfigFromConfig(const Config& cfg,
                                std::vector<std::string>* warnings = nullptr);

/// Backend selection: "mock" (default and always allowed) or "unitree".
struct BackendSelection {
  std::string kind = "mock";
  std::string port = "/dev/ttyUSB3";
  int baudrate = 6000000;
  std::string calib_path;
};

BackendSelection backendSelectionFromConfig(const Config& cfg);

}  // namespace r1_robot
