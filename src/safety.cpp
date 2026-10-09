#include "r1_robot/safety.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace r1_robot {
namespace {

SafetyIssue makeIssue(SafetyLevel level, std::string code, std::string message) {
  SafetyIssue issue;
  issue.level = level;
  issue.code = std::move(code);
  issue.message = std::move(message);
  return issue;
}

std::string hexCode(std::uint32_t code) {
  std::ostringstream oss;
  oss << "0x" << std::hex << code;
  return oss.str();
}

}  // namespace

const char* toString(SafetyLevel level) {
  switch (level) {
    case SafetyLevel::kOk:
      return "ok";
    case SafetyLevel::kWarning:
      return "warning";
    case SafetyLevel::kFault:
      return "fault";
  }
  return "unknown";
}

SafetyLevel safetyLevelFromString(const std::string& s, SafetyLevel fallback) {
  if (s == "ok") return SafetyLevel::kOk;
  if (s == "warning" || s == "warn") return SafetyLevel::kWarning;
  if (s == "fault" || s == "error") return SafetyLevel::kFault;
  return fallback;
}

bool SafetyReport::ok() const {
  for (const SafetyIssue& i : issues) {
    if (i.level == SafetyLevel::kFault) {
      return false;
    }
  }
  return true;
}

bool SafetyReport::hasWarnings() const {
  for (const SafetyIssue& i : issues) {
    if (i.level == SafetyLevel::kWarning) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> SafetyReport::faultCodes() const {
  std::vector<std::string> codes;
  for (const SafetyIssue& i : issues) {
    if (i.level == SafetyLevel::kFault) {
      codes.push_back(i.code);
    }
  }
  return codes;
}

std::string SafetyReport::toString() const {
  std::ostringstream oss;
  if (issues.empty()) {
    return "ok";
  }
  for (std::size_t i = 0; i < issues.size(); ++i) {
    const SafetyIssue& issue = issues[i];
    // Qualified: the member name would otherwise hide the free function.
    oss << r1_robot::toString(issue.level) << " [" << issue.code << "] "
        << issue.message;
    if (i + 1 < issues.size()) {
      oss << "; ";
    }
  }
  return oss.str();
}

SafetyReport mergeReports(const SafetyReport& a, const SafetyReport& b) {
  SafetyReport merged;
  merged.issues.reserve(a.issues.size() + b.issues.size());
  merged.issues.insert(merged.issues.end(), a.issues.begin(), a.issues.end());
  merged.issues.insert(merged.issues.end(), b.issues.begin(), b.issues.end());
  return merged;
}

SafetyReport SafetyMonitor::emptyOk() { return SafetyReport{}; }

SafetyMonitor::SafetyMonitor(SafetyConfig cfg) : cfg_(cfg) {}

SafetyReport SafetyMonitor::checkState(const RobotState& state,
                                       std::uint64_t now_ns) const {
  SafetyReport report;

  if (!state.connected) {
    report.issues.push_back(makeIssue(SafetyLevel::kFault, "state.not_connected",
                                      "backend is not connected"));
    return report;
  }

  const std::uint64_t now = (now_ns != 0) ? now_ns : nowNs();

  if (state.stamp_ns == 0) {
    report.issues.push_back(makeIssue(SafetyLevel::kFault, "state.no_timestamp",
                                      "state carries no timestamp"));
  } else if (state.stamp_ns > now) {
    report.issues.push_back(makeIssue(
        SafetyLevel::kFault, "state.timestamp_in_future",
        "state timestamp is in the future (clock or backend bug)"));
  } else {
    const double age_s =
        static_cast<double>(now - state.stamp_ns) * 1e-9;
    if (age_s > cfg_.max_feedback_age_s) {
      std::ostringstream oss;
      oss << std::setprecision(3) << "feedback age " << age_s
          << " s exceeds max_feedback_age_s=" << cfg_.max_feedback_age_s;
      report.issues.push_back(
          makeIssue(SafetyLevel::kFault, "state.stale_feedback", oss.str()));
    }
  }

  if (!state.allFinite()) {
    report.issues.push_back(makeIssue(
        SafetyLevel::kFault, "state.non_finite",
        "state contains NaN/Inf (q, dq or tau)"));
  }

  int invalid = 0;
  std::ostringstream invalid_joints;
  for (int i = 0; i < kNumJoints; ++i) {
    if (!state.valid[static_cast<std::size_t>(i)]) {
      if (invalid > 0) {
        invalid_joints << ", ";
      }
      invalid_joints << i;
      ++invalid;
    }
  }
  if (invalid > 0) {
    const SafetyLevel level = cfg_.require_all_joints_valid ? SafetyLevel::kFault
                                                            : SafetyLevel::kWarning;
    std::ostringstream oss;
    oss << invalid << "/" << kNumJoints
        << " joints have no valid feedback (joints: " << invalid_joints.str()
        << ")";
    report.issues.push_back(
        makeIssue(level, "state.joint_invalid", oss.str()));
  }

  for (int i = 0; i < kNumJoints; ++i) {
    const std::uint32_t code = state.error_code[static_cast<std::size_t>(i)];
    if (code != 0) {
      std::ostringstream oss;
      oss << "joint " << i << " reports motor error " << hexCode(code)
          << " (0x8 = transient undervoltage, see HARDWARE_BRINGUP.md)";
      report.issues.push_back(
          makeIssue(SafetyLevel::kFault, "state.motor_error", oss.str()));
    }
  }

  if (cfg_.min_supply_voltage > 0.0) {
    for (int i = 0; i < kNumJoints; ++i) {
      const double v = state.voltage[static_cast<std::size_t>(i)];
      // A non-positive value means "not reported by this backend"; the check is
      // then skipped rather than guessed.
      if (v > 0.0 && v < cfg_.min_supply_voltage) {
        std::ostringstream oss;
        oss << std::setprecision(3) << "joint " << i << " supply voltage " << v
            << " V below min_supply_voltage=" << cfg_.min_supply_voltage << " V";
        report.issues.push_back(
            makeIssue(SafetyLevel::kFault, "state.undervoltage", oss.str()));
      }
    }
  }

  return report;
}

SafetyReport SafetyMonitor::checkCommand(const ArmCommand& cmd,
                                         const RobotState& state,
                                         const RobotLimits& limits,
                                         std::uint64_t now_ns,
                                         bool calibration_ok) const {
  SafetyReport report;

  if (!cmd.allFinite()) {
    report.issues.push_back(makeIssue(SafetyLevel::kFault, "cmd.non_finite",
                                      "command contains NaN/Inf"));
    return report;
  }

  // A stop command can only ever remove energy: always allowed.
  if (!cmd.hasActuation()) {
    return report;
  }

  if (cfg_.require_calibration && !calibration_ok) {
    report.issues.push_back(makeIssue(
        SafetyLevel::kFault, "cmd.no_calibration",
        "actuating command refused: no valid joint calibration"));
  }

  // The command is only as trustworthy as the state it is applied to.
  report = mergeReports(report, checkState(state, now_ns));

  bool has_output = false;

  for (int i = 0; i < kNumJoints; ++i) {
    const JointCommand& j = cmd.joints[static_cast<std::size_t>(i)];
    const std::size_t si = static_cast<std::size_t>(i);

    if (j.mode == ControlMode::kStop) {
      continue;  // stopped joints are exempt from the envelope checks
    }

    const JointLimits& lim = limits.joints[si];
    const std::string name =
        limits.names[si].empty() ? std::to_string(i) : limits.names[si];

    auto fault = [&](const char* code, const std::string& detail) {
      std::ostringstream oss;
      oss << "joint " << i << " (" << name << ") " << detail;
      report.issues.push_back(makeIssue(SafetyLevel::kFault, code, oss.str()));
    };

    // Fail closed per joint, independently of `require_all_joints_valid`.
    // That flag only governs how the *state report* classifies a missing
    // frame; it must never let an actuating command through for a joint whose
    // position is unknown, because that would also skip the anti-step check
    // below (the two together are a real hole).
    if (j.kp > 0.0 && !state.valid[si]) {
      fault("cmd.no_feedback_for_joint",
            "actuating command for a joint with no valid feedback "
            "(fail-closed)");
    }

    if (j.kp < 0.0 || j.kp > cfg_.max_kp) {
      std::ostringstream oss;
      oss << "kp=" << j.kp << " outside [0, " << cfg_.max_kp << "] Nm/rad";
      fault("cmd.gain_out_of_range", oss.str());
    }
    if (j.kd < 0.0 || j.kd > cfg_.max_kd) {
      std::ostringstream oss;
      oss << "kd=" << j.kd << " outside [0, " << cfg_.max_kd << "] Nm/(rad/s)";
      fault("cmd.gain_out_of_range", oss.str());
    }
    if (std::fabs(j.tau) > cfg_.max_tau) {
      std::ostringstream oss;
      oss << "tau=" << j.tau << " Nm exceeds max_tau=" << cfg_.max_tau;
      fault("cmd.torque_out_of_range", oss.str());
    }
    if (std::fabs(j.dq) > cfg_.max_abs_velocity) {
      std::ostringstream oss;
      oss << "dq=" << j.dq << " rad/s exceeds max_abs_velocity="
          << cfg_.max_abs_velocity;
      fault("cmd.velocity_out_of_range", oss.str());
    }

    // Position-derived checks only apply when the joint actually has position
    // authority (kp > 0).  A compliant FOC command (kp == 0, e.g. the
    // zero-torque probe) carries a meaningless `q` field and must not be
    // rejected because of it.
    if (j.kp > 0.0) {
      // Soft joint limits.  If the margin is wider than half the range, fall
      // back to the joint centre instead of producing a guaranteed fault.
      const double centre = 0.5 * (lim.lower + lim.upper);
      const double lo = std::min(lim.lower + cfg_.soft_limit_margin, centre);
      const double hi = std::max(lim.upper - cfg_.soft_limit_margin, centre);
      if (j.q < lo || j.q > hi) {
        std::ostringstream oss;
        oss << "q=" << j.q << " rad outside soft limits [" << lo << ", " << hi
            << "] rad";
        fault("cmd.out_of_limits", oss.str());
      }

      // Anti-step: never jump further than max_joint_step_rad per cycle from
      // the measured position.
      if (state.valid[si]) {
        const double step = std::fabs(j.q - state.q[si]);
        if (step > cfg_.max_joint_step_rad) {
          std::ostringstream oss;
          oss << "q jump " << step << " rad from measured " << state.q[si]
              << " exceeds max_joint_step_rad=" << cfg_.max_joint_step_rad;
          fault("cmd.step_too_large", oss.str());
        }
      }
    }

    if (j.kp != 0.0 || j.kd != 0.0 || j.tau != 0.0) {
      has_output = true;
    }
  }

  if (!has_output) {
    report.issues.push_back(makeIssue(
        SafetyLevel::kWarning, "cmd.compliant_foc",
        "every actuated joint has zero gains and zero torque: the arm will be "
        "compliant and will NOT hold against gravity"));
  }

  return report;
}

SafetyReport SafetyMonitor::checkMotionEnable(const RobotState& state,
                                              std::uint64_t now_ns,
                                              bool calibration_ok) const {
  SafetyReport report = checkState(state, now_ns);

  if (cfg_.require_calibration && !calibration_ok) {
    report.issues.push_back(makeIssue(
        SafetyLevel::kFault, "enable.no_calibration",
        "motion enable refused: no valid joint calibration"));
  }

  for (int i = 0; i < kNumJoints; ++i) {
    if (state.mode[static_cast<std::size_t>(i)] != ControlMode::kStop) {
      std::ostringstream oss;
      oss << "joint " << i
          << " already reports FOC before enabling; the enable path expects a "
             "stopped arm";
      report.issues.push_back(makeIssue(SafetyLevel::kWarning,
                                        "enable.already_actuated", oss.str()));
      break;
    }
  }

  return report;
}

}  // namespace r1_robot
