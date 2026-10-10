// ---------------------------------------------------------------------------
// arm_status - read-only bring-up / diagnostic tool.
//
// This tool deliberately CANNOT move the arm:
//   * it never enables motion,
//   * it never sends a position, torque or home command,
//   * the only actuating call it can make is --probe, which must be requested
//     twice (a flag plus an explicit acknowledgement) because it powers the
//     motor bridges.
//
// Real motion bring-up is a staged manual procedure - see HARDWARE_BRINGUP.md.
// ---------------------------------------------------------------------------
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include "r1_robot/backend_factory.hpp"
#include "r1_robot/config.hpp"
#include "r1_robot/mock_arm.hpp"
#include "r1_robot/r1_arm.hpp"

#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)
#include "r1_robot/unitree_backend.hpp"
#endif

namespace {

const char* kProbeAck = "--yes-i-know-this-powers-the-motors";

void printUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "Read-only diagnostics for the R1-7A stack. Never moves the arm.\n\n"
      << "Options:\n"
      << "  --config PATH   configuration file (default: ./config/r1_robot.conf\n"
      << "                  when it exists, otherwise built-in defaults)\n"
      << "  --backend KIND  mock | unitree        (default: from config, 'mock')\n"
      << "  --port PATH     serial port for the unitree backend\n"
      << "  --calib PATH    joint_calib.txt for the unitree backend\n"
      << "  --status        connect and print state + safety report (default)\n"
      << "  --safety        print the effective safety configuration and exit\n"
      << "  --probe         send ONE zero-torque FOC frame so the bus starts\n"
      << "                  reporting feedback. REQUIRES " << kProbeAck << "\n"
      << "                  because it powers the motor bridges.\n"
      << "  -h, --help\n\n"
      << "NOT available here (on purpose): home, goto, zero-torque hold, goto,\n"
      << "any motion command, and any auto-enable.\n";
}

std::string describeIssue(const r1_robot::SafetyReport& report) {
  return report.toString();
}

void printState(const r1_robot::RobotState& state) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "state: connected=" << (state.connected ? "yes" : "no")
            << " stamp=" << r1_robot::formatTimeNs(state.stamp_ns) << "\n";
  for (int i = 0; i < r1_robot::kNumJoints; ++i) {
    const std::size_t si = static_cast<std::size_t>(i);
    std::cout << "  j" << i << " q=" << state.q[si] << " dq=" << state.dq[si]
              << " tau=" << state.tau[si]
              << " mode=" << r1_robot::toString(state.mode[si])
              << " valid=" << (state.valid[si] ? "yes" : "NO")
              << " err=0x" << std::hex << state.error_code[si] << std::dec
              << "\n";
  }
}

void printSafety(const r1_robot::SafetyConfig& s) {
  std::cout << "safety configuration:\n"
            << "  allow_real_hardware     = "
            << (s.allow_real_hardware ? "true" : "false") << "\n"
            << "  require_all_joints_valid= "
            << (s.require_all_joints_valid ? "true" : "false") << "\n"
            << "  require_calibration     = "
            << (s.require_calibration ? "true" : "false") << "\n"
            << "  max_feedback_age_s      = " << s.max_feedback_age_s << "\n"
            << "  soft_limit_margin       = " << s.soft_limit_margin << " rad\n"
            << "  max_joint_step_rad      = " << s.max_joint_step_rad << " rad\n"
            << "  max_kp / max_kd         = " << s.max_kp << " / " << s.max_kd
            << "\n"
            << "  max_tau                 = " << s.max_tau << " Nm\n"
            << "  max_abs_velocity        = " << s.max_abs_velocity << " rad/s\n"
            << "  min_supply_voltage      = " << s.min_supply_voltage << " V\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "config/r1_robot.conf";
  std::string backend_kind;
  std::string port;
  std::string calib;
  bool do_probe = false;
  bool probe_ack = false;
  bool print_safety_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](std::string& dst) {
      if (i + 1 < argc) {
        dst = argv[++i];
      }
    };
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--config") {
      next(config_path);
    } else if (arg == "--backend") {
      next(backend_kind);
    } else if (arg == "--port") {
      next(port);
    } else if (arg == "--calib") {
      next(calib);
    } else if (arg == "--status") {
      do_probe = false;
    } else if (arg == "--safety") {
      print_safety_only = true;
    } else if (arg == "--probe") {
      do_probe = true;
    } else if (arg == kProbeAck) {
      probe_ack = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  // -- configuration --------------------------------------------------------
  r1_robot::Config config;
  std::string error;
  {
    // A missing config file is fine: the built-in defaults are already safe.
    r1_robot::Config loaded;
    if (r1_robot::Config::fromFile(config_path, &loaded, &error)) {
      config = loaded;
      std::cout << "config: loaded " << config_path << "\n";
    } else {
      std::cout << "config: using built-in defaults (" << error << ")\n";
    }
  }

  std::vector<std::string> warnings;
  r1_robot::R1ArmConfig arm_config =
      r1_robot::armConfigFromConfig(config, &warnings);
  for (const std::string& w : warnings) {
    std::cout << "config warning: " << w << "\n";
  }

  const r1_robot::BackendSelection selection =
      r1_robot::backendSelectionFromConfig(config);
  const std::string kind = backend_kind.empty() ? selection.kind : backend_kind;

  if (print_safety_only) {
    printSafety(arm_config.safety);
    std::cout << "selected backend: " << kind << "\n"
              << "limits source   : " << arm_config.limits.source << "\n";
    return 0;
  }

  // -- backend --------------------------------------------------------------
  r1_robot::ArmBackendPtr backend;
  if (kind == "mock") {
    backend = r1_robot::makeMockBackend();
  } else if (kind == "unitree") {
#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)
    // Report the exact SDK pin this binary was built against.
    std::cout << "official SDK: " << r1_robot::unitreeBackendInfo() << "\n";
    r1_robot::UnitreeBackendConfig ucfg;
    ucfg.port = port.empty() ? selection.port : port;
    ucfg.baudrate = selection.baudrate;
    ucfg.calib_path = calib.empty() ? (selection.calib_path.empty()
                                           ? "config/joint_calib.txt"
                                           : selection.calib_path)
                                    : calib;
    backend = std::make_shared<r1_robot::UnitreeArmBackend>(ucfg);
#else
    std::cerr << "This build has no unitree backend. Reconfigure with "
                 "-DR1_ROBOT_SDK_ROOT=/path/to/r1-7a_sdk\n";
    return 1;
#endif
  } else {
    std::cerr << "Unknown backend: " << kind << " (expected mock|unitree)\n";
    return 1;
  }

  r1_robot::R1Arm arm(backend, arm_config);
  std::cout << arm.describe() << "\n";

  if (!arm.connect(&error)) {
    std::cerr << "connect failed: " << error << "\n";
    return 1;
  }
  std::cout << "connected: " << arm.describe() << "\n";

  if (do_probe) {
    if (!probe_ack) {
      std::cerr << "--probe refused: this powers the motor bridges. Re-run with "
                << kProbeAck << " only if the arm is supported and you have read "
                   "HARDWARE_BRINGUP.md.\n";
      return 1;
    }
#if defined(R1_ROBOT_WITH_UNITREE_BACKEND)
    // The bring-up tool drives motors only through R1Arm's gated path: the
    // backend itself refuses a probe to any caller that cannot present an
    // unforgeable MotionControl token, which only R1Arm can mint.
    if (!arm.probeFeedbackForBringup(/*human_confirmed=*/true, &error)) {
      std::cerr << "probe failed: " << error << "\n";
      return 1;
    }
    std::cout << "probe: one zero-torque FOC frame sent (arm is COMPLIANT)\n";
#else
    std::cerr << "--probe requires the unitree backend\n";
    return 1;
#endif
  }

  const r1_robot::RobotState state = arm.state();
  printState(state);

  const r1_robot::SafetyReport report = arm.checkState();
  std::cout << "\nsafety: " << (report.ok() ? "PASS" : "FAIL") << "\n"
            << "  " << describeIssue(report) << "\n";

  arm.disconnect();
  std::cout << "\ndone (no motion command was ever sent).\n";
  return report.ok() ? 0 : 2;
}
