# ---------------------------------------------------------------------------
# python/binding/CMakeLists.txt
#
# Optional pybind11 module that lets the production C++ ControlService drive the
# Python MuJoCo SimArm.  Built only when R1_ROBOT_BUILD_PYTHON_BINDING=ON (which
# the default configure and CI both leave OFF), so the core / CI build is
# completely unaffected when the binding is not requested.
# ---------------------------------------------------------------------------

# Locate pybind11's CMake package, preferring the one shipped by the active
# Python interpreter (`python -m pybind11 --cmakedir`).
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
  OUTPUT_VARIABLE _pybind11_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _pybind11_rc)
if(NOT _pybind11_rc EQUAL 0)
  message(FATAL_ERROR
    "R1_ROBOT_BUILD_PYTHON_BINDING=ON but pybind11 is not importable in "
    "${Python3_EXECUTABLE}.  Install it with `python -m pip install pybind11`, "
    "or configure with -DR1_ROBOT_BUILD_PYTHON_BINDING=OFF.")
endif()
list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_control_binding r1_control_binding.cpp)
target_link_libraries(_control_binding PRIVATE r1_robot_core)
target_include_directories(_control_binding PRIVATE
  "${PROJECT_SOURCE_DIR}/include")
set_target_properties(_control_binding PROPERTIES
  # Land the .so inside the r1_robot package so `from r1_robot import
  # _control_binding` works straight after a build, with no path juggling.
  LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/python/r1_robot")
r1_robot_target_warnings(_control_binding)
