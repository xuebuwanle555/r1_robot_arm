# ===========================================================================
# CheckSdkUntouched.cmake
#
# Post-build guard for the read-only SDK policy.   Run as a CMake script:
#
#   cmake -DR1_SDK_SRC=<sdk-root> -P CheckSdkUntouched.cmake
#
# Fails the build when the SDK checkout has local modifications, which would
# mean r1_robot (or something else) wrote into a dependency it must treat as
# immutable.  Also runnable by hand at any time.
# ===========================================================================

if(NOT DEFINED R1_SDK_SRC)
  message(FATAL_ERROR "CheckSdkUntouched.cmake: -DR1_SDK_SRC=<sdk-root> is required")
endif()

if(NOT EXISTS "${R1_SDK_SRC}")
  message(FATAL_ERROR "CheckSdkUntouched.cmake: '${R1_SDK_SRC}' does not exist")
endif()

# A submodule checkout has a .git *file* (gitdir pointer), a normal clone a
# .git directory; both satisfy EXISTS().
if(NOT EXISTS "${R1_SDK_SRC}/.git")
  message(STATUS "r1_sdk: '${R1_SDK_SRC}' is not a git checkout "
                 "(e.g. a tarball) - cleanliness cannot be verified")
  return()
endif()

find_package(Git QUIET)
if(NOT GIT_FOUND)
  message(STATUS "r1_sdk: git not available - skipping the cleanliness check")
  return()
endif()

execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${R1_SDK_SRC}" status --porcelain
  OUTPUT_VARIABLE _dirty
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
  RESULT_VARIABLE _rc)

if(NOT _rc EQUAL 0)
  message(STATUS "r1_sdk: 'git status' failed - skipping the cleanliness check")
  return()
endif()

if(_dirty)
  message(FATAL_ERROR
"The official R1 SDK checkout was MODIFIED:\n"
"  ${R1_SDK_SRC}\n\n"
"${_dirty}\n\n"
"r1_robot must consume the SDK read-only:\n"
"  * the vendored motor_controller is built out-of-tree into <build>/lib\n"
"    (see r1_sdk_redirect_outputs() in cmake/R1SdkDependency.cmake);\n"
"  * no artifact of this project may land inside the checkout.\n\n"
"Inspect with:\n"
"  git -C ${R1_SDK_SRC} status\n"
"  git -C ${R1_SDK_SRC} diff\n\n"
"If the change is unwanted, restore it with:\n"
"  git -C ${R1_SDK_SRC} checkout -- .\n")
endif()

message(STATUS "r1_sdk: checkout is clean - read-only policy honoured")
