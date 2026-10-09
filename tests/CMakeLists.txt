# ---------------------------------------------------------------------------
# Unit tests.  Everything here runs against MockArm only - no hardware, no
# serial port, no sleeping.  Real-hardware bring-up is a separate, manual
# procedure (see HARDWARE_BRINGUP.md).
# ---------------------------------------------------------------------------

add_executable(r1_robot_tests
  test_main.cpp
  test_config.cpp
  test_feedback_freshness.cpp
  test_gripper.cpp
  test_hardware_gate.cpp
  test_mock_arm.cpp
  test_r1_arm.cpp
  test_safety.cpp
  test_types_and_limits.cpp
)

target_link_libraries(r1_robot_tests PRIVATE r1_robot_core)
target_include_directories(r1_robot_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
r1_robot_target_warnings(r1_robot_tests)

if(R1_ROBOT_WITH_UNITREE_BACKEND)
  # Link the real-hardware backend so its translation units are compiled and
  # checked, even though no test touches the hardware.
  target_link_libraries(r1_robot_tests PRIVATE r1_robot_unitree_backend)
  set_target_properties(r1_robot_tests PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib"
    INSTALL_RPATH "$ORIGIN/../lib")
endif()

add_test(NAME r1_robot_all COMMAND r1_robot_tests)

# Per-suite entries so `ctest -R safety` works.
foreach(suite limits safety mock_arm r1_arm gripper config motion_gate freshness)
  add_test(NAME r1_robot_${suite} COMMAND r1_robot_tests --filter ${suite})
endforeach()
