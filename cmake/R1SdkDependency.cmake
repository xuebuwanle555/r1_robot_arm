# ===========================================================================
# R1SdkDependency.cmake
#
# Resolves the official Unitree R1-7A SDK
# (https://github.com/unitreerobotics/r1-7a_sdk) as a pinned, read-only
# third-party dependency.
#
# ---------------------------------------------------------------------------
# POLICY
# ---------------------------------------------------------------------------
#  1. STRICT DEFAULT PATH
#     The only path used by default is the in-project submodule:
#
#         ${PROJECT_SOURCE_DIR}/third_party/r1-7a_sdk
#
#     There is deliberately NO search of $HOME, of sibling directories
#     ("../r1-7a_sdk-main", ...) or of any other checkout on the machine.  A
#     missing submodule is a hard configuration error, never a silent
#     downgrade: an unexpected SDK would change the motor protocol underneath
#     the safety layer without anyone noticing.
#
#  2. EXPLICIT OVERRIDE ONLY
#     -DR1_ROBOT_SDK_ROOT=<path> may be used for unusual setups (tarball,
#     out-of-tree checkout, CI cache).  An explicit flag is intentional, which
#     is what distinguishes it from a silent fallback.
#
#  3. READ-ONLY SOURCES
#     r1_robot consumes exactly:
#         include/arm_motor_bus.h        (public interface)
#         src/arm_motor_bus.cpp          (compiled unchanged into our backend)
#         thirdparty/motor_controller/**  (built OUT-OF-TREE)
#     It never patches, rewrites or generates anything inside the checkout.
#     The vendored motor_controller CMakeLists hard-codes its output directory
#     to its own source lib/ directory, so r1_sdk_redirect_outputs() MUST be
#     called on that target; r1_sdk_add_cleanliness_check() then fails the
#     build if the checkout still ends up dirty.
#
#  4. PINNED AND REPORTED
#     The real git commit of the checkout is read at configure time and is
#     reported both in the configure log and at runtime through
#     unitreeBackendInfo().
#
# ---------------------------------------------------------------------------
# PROVIDES
# ---------------------------------------------------------------------------
#   R1_SDK_FOUND            TRUE when a usable SDK was resolved
#   R1_SDK_ROOT             resolved SDK root
#   R1_SDK_VERSION          first line of the SDK VERSION file
#   R1_SDK_COMMIT           12-char git commit (or "unknown")
#   R1_SDK_COMMIT_FULL      full 40-char git commit (or "unknown")
#   R1_SDK_COMMIT_SUBJECT   "<date>:<subject>" of the checked-out commit
#   R1_SDK_DIRTY            TRUE if the checkout had local modifications
#   R1_SDK_MODULE_DIR       directory of this module
#   r1_sdk_redirect_outputs(<target>)       keep artifacts out of the SDK
#   r1_sdk_add_cleanliness_check(<target>)  fail the build if the SDK is dirty
# ===========================================================================

set(R1_SDK_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(R1_SDK_DEFAULT_ROOT "${PROJECT_SOURCE_DIR}/third_party/r1-7a_sdk")

# Files that must exist for a checkout to be a usable R1 SDK.
set(R1_SDK_REQUIRED_FILES
  "include/arm_motor_bus.h"
  "src/arm_motor_bus.cpp"
  "thirdparty/motor_controller/CMakeLists.txt"
  "thirdparty/motor_controller/include/MotorProtocol.h"
  "thirdparty/motor_controller/src/MotorProtocol.cpp")

# Result variables: always defined, so the top-level summary never breaks.
set(R1_SDK_FOUND FALSE)
set(R1_SDK_ROOT "n/a")
set(R1_SDK_VERSION "unknown")
set(R1_SDK_COMMIT "unknown")
set(R1_SDK_COMMIT_FULL "unknown")
set(R1_SDK_COMMIT_SUBJECT "unknown")
set(R1_SDK_DIRTY FALSE)

# ---------------------------------------------------------------------------
# Helper: one clear, actionable error instead of a silent fallback.
#
# The guidance is printed as STATUS lines first; the FATAL_ERROR message itself
# stays a single line so CMake's own error formatting remains readable.
# ---------------------------------------------------------------------------
function(r1_sdk_report_missing_sdk reason)
  message(STATUS "")
  message(STATUS "================= r1_robot: R1 SDK is not usable =================")
  message(STATUS "reason: ${reason}")
  message(STATUS "")
  message(STATUS "r1_robot consumes the official Unitree R1-7A SDK as an IN-PROJECT,")
  message(STATUS "PINNED git submodule.  It never falls back to another checkout on")
  message(STATUS "this machine: an unexpected SDK would silently change the motor")
  message(STATUS "protocol underneath the safety layer.")
  message(STATUS "")
  message(STATUS "  expected location : ${R1_SDK_DEFAULT_ROOT}")
  message(STATUS "")
  message(STATUS "Initialise the submodule (from the r1_robot repository root):")
  message(STATUS "    git submodule update --init --recursive third_party/r1-7a_sdk")
  message(STATUS "")
  message(STATUS "For a fresh clone:")
  message(STATUS "    git clone --recurse-submodules <r1_robot-url>")
  message(STATUS "")
  message(STATUS "Only if you deliberately want a different, pinned checkout:")
  message(STATUS "    cmake -S . -B build -DR1_ROBOT_SDK_ROOT=/path/to/r1-7a_sdk")
  message(STATUS "")
  message(STATUS "To build without any real-hardware backend (MockArm only, no SDK):")
  message(STATUS "    cmake -S . -B build -DR1_ROBOT_WITH_UNITREE_BACKEND=OFF")
  message(STATUS "==================================================================")
  message(FATAL_ERROR
    "r1_robot: the official R1 SDK is not usable - see the guidance above.")
endfunction()

# ---------------------------------------------------------------------------
# Helper: force build artifacts out of the SDK checkout.
#
# The vendored motor_controller target sets LIBRARY_OUTPUT_DIRECTORY to
# <sdk>/thirdparty/motor_controller/lib, which would write a freshly built
# libmotor_controller.so* into the read-only checkout.  Override it after
# add_subdirectory() so the shared library lands in <build>/lib instead.
# ---------------------------------------------------------------------------
function(r1_sdk_redirect_outputs target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR
      "r1_sdk_redirect_outputs(): target '${target}' does not exist. Call it "
      "after add_subdirectory(<sdk>/thirdparty/motor_controller ...).")
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

  # Multi-config generators append a per-config subdirectory by default.
  foreach(_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${_cfg}" _CFG)
    set_target_properties(${target} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib"
      ARCHIVE_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib"
      RUNTIME_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib")
  endforeach()
endfunction()

# ---------------------------------------------------------------------------
# Helper: fail the build if the SDK checkout was modified.
#
# NOTE: add_custom_command(TARGET ...) only works in the directory that created
# the target, and motor_controller is created inside the SDK subdirectory.  A
# custom target that DEPENDS ON the SDK target is used instead, which also
# guarantees the check runs after the library has been produced.
# ---------------------------------------------------------------------------
function(r1_sdk_add_cleanliness_check target)
  if(NOT R1_SDK_FOUND)
    return()
  endif()
  if(NOT TARGET ${target})
    message(FATAL_ERROR
      "r1_sdk_add_cleanliness_check(): target '${target}' does not exist")
  endif()
  if(NOT EXISTS "${R1_SDK_MODULE_DIR}/CheckSdkUntouched.cmake")
    message(WARNING "CheckSdkUntouched.cmake is missing; the read-only SDK "
                    "guarantee is not verified for target '${target}'")
    return()
  endif()

  set(_r1_check_target "r1_sdk_cleanliness_check")
  if(NOT TARGET ${_r1_check_target})
    add_custom_target(${_r1_check_target} ALL
      COMMAND "${CMAKE_COMMAND}"
              "-DR1_SDK_SRC=${R1_SDK_ROOT}"
              -P "${R1_SDK_MODULE_DIR}/CheckSdkUntouched.cmake"
      COMMENT "Verifying the official R1 SDK checkout is untouched"
      VERBATIM)
  endif()

  # Runs the check after the vendored SDK library has been built.
  add_dependencies(${_r1_check_target} ${target})
endfunction()

# ---------------------------------------------------------------------------
# Helper: build the SDK from a PATCHED COPY, never from the submodule itself.
#
# WHY
#   Upstream ArmMotorBus exposes only a sticky feedbackReady() flag and no
#   reception timestamp, so stale feedback cannot be detected: a bus that died
#   after the first frame looks identical to a healthy one.  A minimal,
#   documented patch (patches/0001-arm_motor_bus-feedback-stats.patch) adds
#   per-joint reception time + packet count.
#
#   The read-only guarantee is preserved: the submodule is never modified.
#   The two touched files are copied into <build>/sdk_patched and the patch is
#   applied there, so the difference against the official pin is exactly the
#   patch file, which is versioned with r1_robot.
#
# PROVIDES
#   R1_SDK_PATCHED_DIR   build-tree copy to compile instead of the submodule
# ---------------------------------------------------------------------------
function(r1_sdk_prepare_patched_sources)
  set(_patch
      "${R1_SDK_MODULE_DIR}/../patches/0001-arm_motor_bus-feedback-stats.patch")
  if(NOT EXISTS "${_patch}")
    message(FATAL_ERROR
      "r1_robot: the SDK patch is missing: ${_patch}\n"
      "          It is required to obtain real feedback reception timestamps.")
  endif()

  find_program(R1_PATCH_EXECUTABLE patch)
  if(NOT R1_PATCH_EXECUTABLE)
    message(FATAL_ERROR
      "r1_robot: 'patch' is required to build the patched SDK copy "
      "(install it, or use -DR1_ROBOT_WITH_UNITREE_BACKEND=OFF for a mock-only "
      "build).")
  endif()

  set(_dir "${CMAKE_BINARY_DIR}/sdk_patched")
  # Refresh the copies every configure so the patch always applies to pristine
  # sources (idempotent across re-configures).
  file(COPY "${R1_SDK_ROOT}/include/arm_motor_bus.h" DESTINATION "${_dir}/include")
  file(COPY "${R1_SDK_ROOT}/src/arm_motor_bus.cpp" DESTINATION "${_dir}/src")

  execute_process(
    COMMAND "${R1_PATCH_EXECUTABLE}" -p1 --batch --forward --silent
            -i "${_patch}"
    WORKING_DIRECTORY "${_dir}"
    RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR
      "r1_robot: could not apply the minimal SDK patch to the build-tree copy.\n"
      "  patch  : ${_patch}\n"
      "  target : ${_dir}\n"
      "  output : ${_out}${_err}\n"
      "The SDK pin most likely changed.  Re-derive the patch against the new "
      "pin; do NOT patch the submodule in place.")
  endif()

  # Verify the API the backend relies on really exists after patching.
  file(READ "${_dir}/include/arm_motor_bus.h" _patched_header)
  string(FIND "${_patched_header}" "feedbackSnapshot()" _missing)
  if(_missing EQUAL -1)
    message(FATAL_ERROR
      "r1_robot: the patch reported success but feedbackSnapshot() is missing "
      "from ${_dir}/include/arm_motor_bus.h.  The backend needs the atomic "
      "value+reception-evidence snapshot (patch v2); a value must never be read "
      "in a separate lock pass from its age.")
  endif()

  file(SHA256 "${_patch}" _sha)
  message(STATUS "  patch     : 0001-arm_motor_bus-feedback-stats.patch")
  message(STATUS "              sha256 ${_sha}")
  message(STATUS "              applied to a build-tree copy (submodule untouched)")

  set(R1_SDK_PATCHED_DIR "${_dir}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# Resolve the dependency
# ===========================================================================
if(NOT R1_ROBOT_WITH_UNITREE_BACKEND)
  message(STATUS "Official R1 SDK : not required "
                 "(R1_ROBOT_WITH_UNITREE_BACKEND=OFF)")
  message(STATUS "                  -> mock-only build; no real-hardware backend")
  set(R1_SDK_ROOT "n/a (unitree backend disabled)")
else()

  # ---- 1. resolve the root ---------------------------------------------
  if(R1_ROBOT_SDK_ROOT)
    set(R1_SDK_ROOT "${R1_ROBOT_SDK_ROOT}")
    set(_r1_sdk_origin "explicit -DR1_ROBOT_SDK_ROOT")
  else()
    set(R1_SDK_ROOT "${R1_SDK_DEFAULT_ROOT}")
    set(_r1_sdk_origin "in-project submodule (default)")
  endif()

  # ---- 2. validate -----------------------------------------------------
  # Reasons are kept to a single line so the failure output stays readable.
  set(_r1_problem "")

  if(NOT EXISTS "${R1_SDK_ROOT}")
    set(_r1_problem "the SDK directory does not exist: ${R1_SDK_ROOT}")
  elseif(NOT EXISTS "${R1_SDK_ROOT}/include/arm_motor_bus.h")
    file(GLOB _r1_entries "${R1_SDK_ROOT}/*")
    if(NOT _r1_entries)
      set(_r1_problem
          "the submodule directory exists but is EMPTY: ${R1_SDK_ROOT}")
    else()
      set(_r1_problem
          "${R1_SDK_ROOT} is not an R1 SDK checkout "
          "(include/arm_motor_bus.h is missing)")
    endif()
  else()
    set(_r1_missing "")
    foreach(_f IN LISTS R1_SDK_REQUIRED_FILES)
      if(NOT EXISTS "${R1_SDK_ROOT}/${_f}")
        if(_r1_missing)
          string(APPEND _r1_missing ", ")
        endif()
        string(APPEND _r1_missing "${_f}")
      endif()
    endforeach()
    if(_r1_missing)
      set(_r1_problem
          "the SDK at ${R1_SDK_ROOT} is incomplete; missing: ${_r1_missing}")
    endif()
  endif()

  if(_r1_problem)
    r1_sdk_report_missing_sdk("${_r1_problem}")
  endif()

  set(R1_SDK_FOUND TRUE)

  # ---- 3. version ------------------------------------------------------
  if(EXISTS "${R1_SDK_ROOT}/VERSION")
    file(READ "${R1_SDK_ROOT}/VERSION" _r1_version_raw)
    # The upstream VERSION file is a multi-line changelog; only the first line
    # ("Version: x.y.z") is safe to embed in a compile definition.
    string(REGEX REPLACE "[\r\n].*" "" _r1_version_first "${_r1_version_raw}")
    string(REGEX REPLACE "^[Vv]ersion:[ \t]*" "" _r1_version_first "${_r1_version_first}")
    string(STRIP "${_r1_version_first}" R1_SDK_VERSION)
    if(NOT R1_SDK_VERSION)
      set(R1_SDK_VERSION "unknown")
    endif()
  endif()

  # ---- 4. real commit (the actual pin) ---------------------------------
  set(R1_SDK_PIN_PROBLEM "")
  find_package(Git QUIET)

  if(NOT GIT_FOUND)
    set(R1_SDK_PIN_PROBLEM "git was not found, so the SDK commit cannot be read")
  else()
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${R1_SDK_ROOT}" rev-parse HEAD
      OUTPUT_VARIABLE _r1_head
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_VARIABLE _r1_head_err
      RESULT_VARIABLE _r1_head_rc)

    if(NOT _r1_head_rc EQUAL 0 OR NOT _r1_head)
      set(R1_SDK_PIN_PROBLEM
          "'${R1_SDK_ROOT}' is not a git checkout, so the SDK is NOT pinned")
    else()
      set(R1_SDK_COMMIT_FULL "${_r1_head}")
      string(SUBSTRING "${_r1_head}" 0 12 R1_SDK_COMMIT)

      execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${R1_SDK_ROOT}" log -1
                "--format=%ad:%s" --date=short
        OUTPUT_VARIABLE R1_SDK_COMMIT_SUBJECT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)

      execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${R1_SDK_ROOT}" status --porcelain
        OUTPUT_VARIABLE _r1_dirty
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _r1_dirty_rc)
      if(_r1_dirty_rc EQUAL 0 AND _r1_dirty)
        set(R1_SDK_DIRTY TRUE)
      endif()
    endif()
  endif()

  # ---- 5. report -------------------------------------------------------
  message(STATUS "Official R1 SDK")
  message(STATUS "  root      : ${R1_SDK_ROOT}")
  message(STATUS "  origin    : ${_r1_sdk_origin}")
  message(STATUS "  version   : ${R1_SDK_VERSION}")

  if(R1_SDK_PIN_PROBLEM)
    message(WARNING
      "  commit    : UNKNOWN - ${R1_SDK_PIN_PROBLEM}.\n"
      "              r1_robot cannot tell which motor protocol it was built "
      "against.\n"
      "              Use the pinned submodule (third_party/r1-7a_sdk) for any "
      "build you intend to keep.")
  else()
    message(STATUS "  commit    : ${R1_SDK_COMMIT} (${R1_SDK_COMMIT_SUBJECT})")
    message(STATUS "  full hash : ${R1_SDK_COMMIT_FULL}")
  endif()

  if(R1_SDK_DIRTY)
    message(WARNING
      "  state     : the SDK checkout HAS LOCAL MODIFICATIONS.\n"
      "              r1_robot requires a read-only SDK; the motor protocol "
      "may no longer\n"
      "              match the reported commit. Inspect with:\n"
      "                git -C ${R1_SDK_ROOT} status")
  elseif(NOT R1_SDK_PIN_PROBLEM)
    message(STATUS "  state     : clean (read-only)")
  endif()

endif()
