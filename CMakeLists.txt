cmake_minimum_required(VERSION 3.16)

project(r1_robot
  VERSION 0.1.0
  DESCRIPTION "Research control stack for the Unitree R1-7A 7-DOF arm (phase 1: architecture + offline driver)"
  LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Options.  NOTE: every option defaults to the *safe* value.
# ---------------------------------------------------------------------------
option(R1_ROBOT_BUILD_TESTS "Build the unit tests" ON)
option(R1_ROBOT_WITH_UNITREE_BACKEND
  "Build the real-hardware backend that links the official R1 SDK" ON)
option(R1_ROBOT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(R1_ROBOT_SDK_ROOT "" CACHE PATH
  "Explicit path to a pinned r1-7a_sdk checkout. Leave EMPTY to use the \
in-project submodule third_party/r1-7a_sdk (the strict default). \
See cmake/R1SdkDependency.cmake.")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(R1SdkDependency)

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
function(r1_robot_target_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(R1_ROBOT_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# ---------------------------------------------------------------------------
# r1_robot_core : hardware-independent code (types, limits, safety, MockArm)
# ---------------------------------------------------------------------------
add_library(r1_robot_core STATIC
  src/config.cpp
  src/gripper.cpp
  src/joint_bus.cpp
  src/limits.cpp
  src/mock_arm.cpp
  src/r1_arm.cpp
  src/safety.cpp
  src/types.cpp
)
add_library(r1_robot::core ALIAS r1_robot_core)

target_include_directories(r1_robot_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(r1_robot_core PUBLIC Threads::Threads)
r1_robot_target_warnings(r1_robot_core)

target_compile_definitions(r1_robot_core PUBLIC
  R1_ROBOT_PHASE=1
  R1_ROBOT_VERSION="${PROJECT_VERSION}")

# ---------------------------------------------------------------------------
# r1_robot_unitree_backend : optional real-hardware backend.
# Never enabled at runtime without an explicit human opt-in (see R1Arm).
# ---------------------------------------------------------------------------
if(R1_ROBOT_WITH_UNITREE_BACKEND AND R1_SDK_FOUND)
  # Build the SDK's vendored motor_controller library OUT-OF-TREE.  Its own
  # CMakeLists hard-codes the output directory to its source lib/ directory,
  # so r1_sdk_redirect_outputs() overrides that: nothing may be written into
  # the read-only SDK checkout.
  add_subdirectory(
    "${R1_SDK_ROOT}/thirdparty/motor_controller"
    "${CMAKE_BINARY_DIR}/_deps/motor_controller"
    EXCLUDE_FROM_ALL)

  r1_sdk_redirect_outputs(motor_controller)
  # Fails the build if the checkout ends up dirty anyway.
  r1_sdk_add_cleanliness_check(motor_controller)

  # The SDK is compiled from a build-tree copy with the minimal, documented
  # feedback-stats patch applied; the submodule itself stays read-only.
  r1_sdk_prepare_patched_sources()

  add_library(r1_robot_unitree_backend STATIC
    src/backend_factory.cpp
    src/sdk_joint_bus.cpp
    src/unitree_backend.cpp
    "${R1_SDK_PATCHED_DIR}/src/arm_motor_bus.cpp"
  )
  add_library(r1_robot::unitree ALIAS r1_robot_unitree_backend)

  # The PATCHED copy must come first so `#include "arm_motor_bus.h"` picks up
  # the version that provides feedbackStats().
  target_include_directories(r1_robot_unitree_backend
    PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    PRIVATE "${R1_SDK_PATCHED_DIR}/include"
    PRIVATE "${R1_SDK_ROOT}/include")

  target_link_libraries(r1_robot_unitree_backend
    PUBLIC  r1_robot_core
    PRIVATE motor_controller)

  target_compile_definitions(r1_robot_unitree_backend PUBLIC
    R1_ROBOT_WITH_UNITREE_BACKEND=1
    R1_ROBOT_SDK_VERSION="${R1_SDK_VERSION}"
    R1_ROBOT_SDK_COMMIT="${R1_SDK_COMMIT}")

  r1_robot_target_warnings(r1_robot_unitree_backend)
endif()

# ---------------------------------------------------------------------------
# Examples / tools
# ---------------------------------------------------------------------------
add_executable(r1_arm_status examples/arm_status.cpp)
target_link_libraries(r1_arm_status PRIVATE r1_robot_core)
r1_robot_target_warnings(r1_arm_status)

if(R1_ROBOT_WITH_UNITREE_BACKEND)
  target_link_libraries(r1_arm_status PRIVATE r1_robot_unitree_backend)
  set_target_properties(r1_arm_status PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib"
    INSTALL_RPATH "$ORIGIN/../lib")
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(R1_ROBOT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "r1_robot ${PROJECT_VERSION} configuration")
message(STATUS "  build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "  sdk root            : ${R1_SDK_ROOT}")
message(STATUS "  sdk commit          : ${R1_SDK_COMMIT}")
message(STATUS "  unitree backend     : ${R1_ROBOT_WITH_UNITREE_BACKEND}")
message(STATUS "  tests               : ${R1_ROBOT_BUILD_TESTS}")
message(STATUS "  default backend     : mock (real hardware requires explicit opt-in)")
message(STATUS "")
