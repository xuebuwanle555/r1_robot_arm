#include <cstdio>
#include <fstream>
#include <string>

#include "r1_robot/config.hpp"
#include "test_framework.hpp"

using namespace r1_robot;

namespace {

const char* kSampleConfig = R"(
# r1_robot sample config
backend = unitree          # inline comment must be stripped

safety.allow_real_hardware = TRUE
safety.max_kp = 42.5
safety.max_feedback_age_s = not-a-number
hardware.port = /dev/ttyUSB0
hardware.baudrate = 6000000
)";

}  // namespace

R1_TEST(config_defaults_are_the_safe_values) {
  // This is a phase-1 safety guarantee: the default configuration must never
  // permit real hardware.
  const R1ArmConfig arm_config;
  R1_CHECK(!arm_config.safety.allow_real_hardware);
  R1_CHECK(arm_config.safety.require_calibration);
  R1_CHECK(arm_config.safety.require_all_joints_valid);

  const BackendSelection selection;
  R1_CHECK_EQ(selection.kind, std::string("mock"));
}

R1_TEST(config_parses_keys_and_strips_comments) {
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromString(kSampleConfig, &cfg, &error));
  R1_CHECK_CONTAINS(cfg.dump(), "backend = unitree");
  R1_CHECK_EQ(cfg.getString("hardware.port", ""), std::string("/dev/ttyUSB0"));
  R1_CHECK_EQ(cfg.getInt("hardware.baudrate", 0), 6000000);
  R1_CHECK(cfg.getBool("safety.allow_real_hardware", false));
}

R1_TEST(config_bool_spellings) {
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromString("a = true\nb = 1\nc = yes\nd = off\ne = 0\n",
                              &cfg, &error));
  R1_CHECK(cfg.getBool("a", false));
  R1_CHECK(cfg.getBool("b", false));
  R1_CHECK(cfg.getBool("c", false));
  R1_CHECK(!cfg.getBool("d", true));
  R1_CHECK(!cfg.getBool("e", true));
  R1_CHECK(cfg.getBool("missing", true));  // fallback
}

R1_TEST(config_falls_back_on_malformed_values) {
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromString("n = banana\n", &cfg, &error));
  R1_CHECK_NEAR(cfg.getDouble("n", 3.5), 3.5, 1e-12);
  R1_CHECK_EQ(cfg.getInt("n", 7), 7);
  R1_CHECK(cfg.getBool("n", true));
}

R1_TEST(config_rejects_a_line_without_an_equals_sign) {
  Config cfg;
  std::string error;
  R1_CHECK(!Config::fromString("backend unitree\n", &cfg, &error));
  R1_CHECK_CONTAINS(error, "line 1");
}

R1_TEST(config_from_file_reports_io_errors) {
  Config cfg;
  std::string error;
  R1_CHECK(!Config::fromFile("/nonexistent/r1_robot.conf", &cfg, &error));
  R1_CHECK_CONTAINS(error, "cannot open config file");
}

R1_TEST(config_from_file_reads_a_real_file) {
  const std::string path = "/tmp/r1_robot_test_config.conf";
  {
    std::ofstream ofs(path);
    ofs << "backend = mock\n\n# comment only\nsafety.max_tau = 12.5\n";
  }
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromFile(path, &cfg, &error));
  R1_CHECK_EQ(cfg.getString("backend", ""), std::string("mock"));
  R1_CHECK_NEAR(cfg.getDouble("safety.max_tau", 0.0), 12.5, 1e-12);
  std::remove(path.c_str());
}

R1_TEST(config_maps_onto_arm_config_and_warns_about_bad_values) {
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromString(kSampleConfig, &cfg, &error));

  std::vector<std::string> warnings;
  const R1ArmConfig arm_config = armConfigFromConfig(cfg, &warnings);

  R1_CHECK(arm_config.safety.allow_real_hardware);
  R1_CHECK_NEAR(arm_config.safety.max_kp, 42.5, 1e-12);
  // 'not-a-number' must fall back to the conservative default, with a warning.
  R1_CHECK_NEAR(arm_config.safety.max_feedback_age_s, SafetyConfig{}.max_feedback_age_s,
                1e-12);
  R1_CHECK_EQ(warnings.size(), 1u);
  R1_CHECK_CONTAINS(warnings[0], "safety.max_feedback_age_s");
}

R1_TEST(config_backend_selection) {
  Config cfg;
  std::string error;
  R1_CHECK(Config::fromString(kSampleConfig, &cfg, &error));

  const BackendSelection sel = backendSelectionFromConfig(cfg);
  R1_CHECK_EQ(sel.kind, std::string("unitree"));
  R1_CHECK_EQ(sel.port, std::string("/dev/ttyUSB0"));
  R1_CHECK_EQ(sel.baudrate, 6000000);

  Config empty;
  const BackendSelection default_sel = backendSelectionFromConfig(empty);
  R1_CHECK_EQ(default_sel.kind, std::string("mock"));
  R1_CHECK_EQ(default_sel.port, std::string("/dev/ttyUSB3"));
}
