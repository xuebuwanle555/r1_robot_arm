#include "r1_robot/config.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace r1_robot {
namespace {

std::string trim(const std::string& s) {
  const char* ws = " \t\r\n";
  const auto begin = s.find_first_not_of(ws);
  if (begin == std::string::npos) {
    return std::string();
  }
  const auto end = s.find_last_not_of(ws);
  return s.substr(begin, end - begin + 1);
}

std::string lower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

bool parseBool(const std::string& raw, bool* out) {
  const std::string v = lower(trim(raw));
  if (v == "true" || v == "1" || v == "yes" || v == "on") {
    *out = true;
    return true;
  }
  if (v == "false" || v == "0" || v == "no" || v == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool parseNumber(const std::string& raw, double* out) {
  const std::string v = trim(raw);
  if (v.empty()) {
    return false;
  }
  char* end = nullptr;
  const double value = std::strtod(v.c_str(), &end);
  if (end == v.c_str() || *end != '\0') {
    return false;
  }
  *out = value;
  return true;
}

/** Strip a trailing "# comment" and surrounding whitespace. */
std::string stripInlineComment(const std::string& value) {
  const auto pos = value.find('#');
  if (pos == std::string::npos) {
    return trim(value);
  }
  return trim(value.substr(0, pos));
}

}  // namespace

bool Config::fromString(const std::string& text, Config* out,
                        std::string* error) {
  if (out == nullptr) {
    if (error) {
      *error = "Config::fromString: null output";
    }
    return false;
  }

  Config cfg;
  std::istringstream iss(text);
  std::string line;
  int lineno = 0;

  while (std::getline(iss, line)) {
    ++lineno;
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }
    const auto eq = stripped.find('=');
    if (eq == std::string::npos) {
      if (error) {
        std::ostringstream oss;
        oss << "line " << lineno << ": expected 'key = value', got '" << stripped
            << "'";
        *error = oss.str();
      }
      return false;
    }
    const std::string key = trim(stripped.substr(0, eq));
    const std::string value = stripInlineComment(stripped.substr(eq + 1));
    if (key.empty()) {
      if (error) {
        std::ostringstream oss;
        oss << "line " << lineno << ": empty key";
        *error = oss.str();
      }
      return false;
    }
    cfg.values_[key] = value;
  }

  *out = cfg;
  return true;
}

bool Config::fromFile(const std::string& path, Config* out, std::string* error) {
  std::ifstream ifs(path);
  if (!ifs) {
    if (error) {
      *error = "cannot open config file: " + path;
    }
    return false;
  }
  std::ostringstream buffer;
  buffer << ifs.rdbuf();
  if (!Config::fromString(buffer.str(), out, error)) {
    if (error) {
      *error = path + ": " + *error;
    }
    return false;
  }
  return true;
}

bool Config::has(const std::string& key) const {
  return values_.find(key) != values_.end();
}

std::string Config::getString(const std::string& key,
                              const std::string& fallback) const {
  const auto it = values_.find(key);
  return (it == values_.end()) ? fallback : it->second;
}

double Config::getDouble(const std::string& key, double fallback) const {
  double value = 0.0;
  if (!has(key) || !parseNumber(getString(key, ""), &value)) {
    return fallback;
  }
  return value;
}

int Config::getInt(const std::string& key, int fallback) const {
  double value = 0.0;
  if (!has(key) || !parseNumber(getString(key, ""), &value)) {
    return fallback;
  }
  return static_cast<int>(value);
}

bool Config::getBool(const std::string& key, bool fallback) const {
  bool value = false;
  if (!has(key) || !parseBool(getString(key, ""), &value)) {
    return fallback;
  }
  return value;
}

std::string Config::dump() const {
  std::ostringstream oss;
  for (const auto& kv : values_) {
    oss << kv.first << " = " << kv.second << "\n";
  }
  return oss.str();
}

R1ArmConfig armConfigFromConfig(const Config& cfg,
                                std::vector<std::string>* warnings) {
  const SafetyConfig defaults;
  R1ArmConfig out;

  auto number = [&](const char* key, double fallback) {
    if (!cfg.has(key)) {
      return fallback;
    }
    double value = 0.0;
    const std::string raw = cfg.getString(key, "");
    if (!parseNumber(raw, &value)) {
      if (warnings) {
        warnings->push_back(std::string("'") + key + "' is not a number ('" +
                            raw + "'), using default");
      }
      return fallback;
    }
    return value;
  };

  auto boolean = [&](const char* key, bool fallback) {
    if (!cfg.has(key)) {
      return fallback;
    }
    bool value = false;
    const std::string raw = cfg.getString(key, "");
    if (!parseBool(raw, &value)) {
      if (warnings) {
        warnings->push_back(std::string("'") + key + "' is not a boolean ('" +
                            raw + "'), using default");
      }
      return fallback;
    }
    return value;
  };

  out.safety.allow_real_hardware =
      boolean("safety.allow_real_hardware", defaults.allow_real_hardware);
  out.safety.require_all_joints_valid =
      boolean("safety.require_all_joints_valid", defaults.require_all_joints_valid);
  out.safety.require_calibration =
      boolean("safety.require_calibration", defaults.require_calibration);
  out.safety.max_feedback_age_s =
      number("safety.max_feedback_age_s", defaults.max_feedback_age_s);
  out.safety.soft_limit_margin =
      number("safety.soft_limit_margin", defaults.soft_limit_margin);
  out.safety.max_joint_step_rad =
      number("safety.max_joint_step_rad", defaults.max_joint_step_rad);
  out.safety.max_kp = number("safety.max_kp", defaults.max_kp);
  out.safety.max_kd = number("safety.max_kd", defaults.max_kd);
  out.safety.max_tau = number("safety.max_tau", defaults.max_tau);
  out.safety.max_abs_velocity =
      number("safety.max_abs_velocity", defaults.max_abs_velocity);
  out.safety.min_supply_voltage =
      number("safety.min_supply_voltage", defaults.min_supply_voltage);

  out.calib_path = cfg.getString("hardware.calib_path", "");
  return out;
}

BackendSelection backendSelectionFromConfig(const Config& cfg) {
  BackendSelection sel;
  sel.kind = cfg.getString("backend", "mock");
  sel.port = cfg.getString("hardware.port", "/dev/ttyUSB3");
  sel.baudrate = cfg.getInt("hardware.baudrate", 6000000);
  sel.calib_path = cfg.getString("hardware.calib_path", "");
  return sel;
}

}  // namespace r1_robot
