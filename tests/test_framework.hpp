// ---------------------------------------------------------------------------
// Minimal, dependency-free test framework.
//
// Why not GoogleTest?  Phase 1 must configure, build and run fully offline.
// Registration is static, so adding a test file requires no central list.
// ---------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "r1_robot/gripper.hpp"
#include "r1_robot/safety.hpp"

namespace r1_robot {
// Stream operators so that checkEq()/fail() can print the enums.
inline std::ostream& operator<<(std::ostream& os, const ControlMode& m) {
  return os << toString(m);
}
inline std::ostream& operator<<(std::ostream& os, const SafetyLevel& l) {
  return os << toString(l);
}
inline std::ostream& operator<<(std::ostream& os, const GripperCommandMode& m) {
  return os << toString(m);
}
}  // namespace r1_robot

namespace r1_test {

struct AssertionFailure : std::runtime_error {
  explicit AssertionFailure(const std::string& what)
      : std::runtime_error(what) {}
};

struct TestCase {
  std::string name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> cases;
  return cases;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) {
    registry().push_back(TestCase{name, std::move(fn)});
  }
};

[[noreturn]] inline void fail(const std::string& message, const char* file,
                             int line) {
  std::ostringstream oss;
  oss << file << ":" << line << ": " << message;
  throw AssertionFailure(oss.str());
}

inline int runAll(const std::string& filter) {
  int passed = 0;
  int failed = 0;
  int filtered = 0;

  for (const TestCase& test : registry()) {
    if (!filter.empty() &&
        test.name.find(filter) == std::string::npos) {
      ++filtered;
      continue;
    }
    try {
      test.fn();
      std::cout << "[ PASS ] " << test.name << "\n";
      ++passed;
    } catch (const std::exception& e) {
      std::cout << "[ FAIL ] " << test.name << "\n            " << e.what()
                << "\n";
      ++failed;
    } catch (...) {
      std::cout << "[ FAIL ] " << test.name << " (unknown exception)\n";
      ++failed;
    }
  }

  std::cout << "\n"
            << passed << " passed, " << failed << " failed";
  if (filtered > 0) {
    std::cout << ", " << filtered << " filtered out";
  }
  std::cout << "\n";
  return (failed == 0) ? 0 : 1;
}

template <typename A, typename B>
inline void checkEq(const A& a, const B& b, const char* ea, const char* eb,
                    const char* file, int line) {
  if (!(a == b)) {
    std::ostringstream oss;
    oss << "expected " << ea << " == " << eb << " (got " << a << " vs " << b
        << ")";
    fail(oss.str(), file, line);
  }
}

template <typename A, typename B>
inline void checkNear(A a, B b, double eps, const char* ea, const char* eb,
                      const char* file, int line) {
  const double da = static_cast<double>(a);
  const double db = static_cast<double>(b);
  if (!(std::fabs(da - db) <= eps)) {
    std::ostringstream oss;
    oss << "expected |" << ea << " - " << eb << "| <= " << eps << " (got " << da
        << " vs " << db << ")";
    fail(oss.str(), file, line);
  }
}

inline void checkContains(const std::string& haystack, const std::string& needle,
                          const char* eh, const char* en, const char* file,
                          int line) {
  if (haystack.find(needle) == std::string::npos) {
    std::ostringstream oss;
    oss << "expected " << eh << " to contain " << en << " (" << needle
        << "), got: " << haystack;
    fail(oss.str(), file, line);
  }
}

inline void checkFault(const r1_robot::SafetyReport& report,
                       const std::string& code, const char* file, int line) {
  for (const r1_robot::SafetyIssue& issue : report.issues) {
    if (issue.level == r1_robot::SafetyLevel::kFault && issue.code == code) {
      return;
    }
  }
  fail("expected fault '" + code + "', report was: " + report.toString(), file,
       line);
}

inline void checkOk(const r1_robot::SafetyReport& report, const char* file,
                    int line) {
  if (!report.ok()) {
    fail("expected a passing report, got: " + report.toString(), file, line);
  }
}

}  // namespace r1_test

#define R1_TEST(name)                                        \
  static void name();                                        \
  static ::r1_test::Registrar r1_test_reg_##name(#name, name); \
  static void name()

#define R1_CHECK(cond)                                             \
  do {                                                             \
    if (!(cond)) {                                                 \
      ::r1_test::fail("CHECK failed: " #cond, __FILE__, __LINE__);  \
    }                                                              \
  } while (0)

#define R1_CHECK_EQ(a, b) ::r1_test::checkEq((a), (b), #a, #b, __FILE__, __LINE__)

#define R1_CHECK_NEAR(a, b, eps) \
  ::r1_test::checkNear((a), (b), (eps), #a, #b, __FILE__, __LINE__)

#define R1_CHECK_CONTAINS(haystack, needle) \
  ::r1_test::checkContains((haystack), (needle), #haystack, #needle, __FILE__, __LINE__)

#define R1_CHECK_FAULT(report, code) \
  ::r1_test::checkFault((report), (code), __FILE__, __LINE__)

#define R1_CHECK_OK(report) ::r1_test::checkOk((report), __FILE__, __LINE__)
