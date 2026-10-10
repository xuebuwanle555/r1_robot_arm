# r1_robot

Research control stack for the **Unitree R1-7A** 7-DOF robotic arm.

> **Status: phase 1 (architecture + offline-testable driver abstraction), now
> hardened through the P0/P1 safety review.** In addition to the phase-1 stack
> this branch adds an unforgeable hardware control-permission boundary, a
> standalone C++ `ControlService` (periodic thread, command watchdog, fault
> latching), an atomic feedback snapshot, a real trajectory success predicate,
> and working MuJoCo collision detection.
> Phase 1 does not implement ROS2, MoveIt2, LeRobot, ACT, Diffusion Policy or
> VLA training. It provides the foundation those will be built on.

| | |
|---|---|
| Robot | Unitree R1-7A, 7 DOF, IM6014 serial bus |
| Official SDK | [`unitreerobotics/r1-7a_sdk`](https://github.com/unitreerobotics/r1-7a_sdk) — pinned third-party dependency, never patched |
| Gripper | custom reBot-style dual rack-and-pinion, **DAMIAO DM4310 over CAN** (interface only in phase 1) |
| Cameras | RealSense D435, Orbbec Astra Pro (out of scope for phase 1) |
| Host | Ubuntu 22.04, x86_64, NVIDIA RTX 5070 Ti, PyTorch 2.9.1+cu130 |

---

## 1. Hard safety guarantees

These are enforced in code, not just in documentation:

| # | Guarantee | Where |
|---|---|---|
| 1 | **The default backend is `MockArm`.** Real hardware is never selected implicitly. | `python/r1_robot/robot.py`, `src/r1_arm.cpp` |
| 2 | **`SafetyConfig::allow_real_hardware` defaults to `false`.** While false, `connect()` refuses any backend that reports `isRealHardware()`. | `include/r1_robot/safety.hpp` |
| 3 | **Motion requires an explicit human confirmation** — `enableMotion(true)` / `enable_motion(human_confirmed=True)`. Nothing enables motors automatically, including on connect, on error, or on state read. | `src/r1_arm.cpp`, `python/r1_robot/robot.py` |
| 4 | **Actuating commands are refused without**: a valid calibration, valid feedback for *all* joints, a fresh timestamp, no motor error codes and a supply voltage above the floor. | `src/safety.cpp` |
| 5 | **No automatic home / zero-torque / high-gain control.** `home()` returns `false` / raises; the only zero-torque path is an explicitly acknowledged `--probe` in `r1_arm_status`. | `src/r1_arm.cpp`, `examples/arm_status.cpp` |
| 6 | **Unverifiable parameters are labelled, not guessed.** Search the tree for `TODO(bringup)` and `待验证`. | throughout |
| 7 | **`stop()` is not a brake.** It sends mode 0 (motor output disabled) and *cannot* hold an arm loaded by gravity. | `HARDWARE_BRINGUP.md` §6 |
| 8 | **The motion lock cannot be disarmed by ordinary business code.** `setMotionArmed()`/`probeFeedback()` require an unforgeable `MotionControl` token that only `R1Arm`/`ControlService` can mint (private ctor + friend); `R1Arm::backend()` returns a `const` read-only handle so actuating calls will not compile. | `include/r1_robot/motion_control.hpp`, `tests/test_hardware_gate.cpp` |
| 9 | **Exactly one owner writes to the hardware bus.** `ControlService` owns the backend, runs the periodic loop and a command-timeout watchdog, latches faults (upper-layer silence / stale feedback / motor error) and never auto-re-arms after a timeout. | `src/control_service.cpp`, `tests/test_control_service.cpp` |

A pure **stop** command is always accepted — even in a failed state — so a fault
can never block the way back to a safe state.

---

## 2. Layout

```
r1_robot/
├── CMakeLists.txt                 top-level build
├── cmake/
│   ├── R1SdkDependency.cmake      resolves + pins the official SDK (strict)
│   └── CheckSdkUntouched.cmake    build-time guard: the SDK must stay read-only
├── third_party/r1-7a_sdk/         the official SDK as a pinned git submodule
├── include/r1_robot/
│   ├── types.hpp                  JointCommand / RobotState / ArmCommand (URDF + SI)
│   ├── limits.hpp                 JointLimits + URDF-derived default table
│   ├── safety.hpp                 SafetyConfig / SafetyMonitor / SafetyReport
│   ├── backend.hpp                IArmBackend abstraction
│   ├── mock_arm.hpp               offline backend (deterministic)
│   ├── r1_arm.hpp                 R1Arm hardware abstraction layer
│   ├── unitree_backend.hpp        real hardware via the official SDK
│   ├── backend_factory.hpp        "mock" | "unitree" factory
│   ├── gripper.hpp                IGripper / MockGripper / Dm4310Gripper
│   └── config.hpp                 "key = value" config reader
├── src/                           implementations (mirrors include/)
├── examples/arm_status.cpp        read-only bring-up / diagnostic tool
├── tests/                         67 offline unit tests (no hardware, no sleeps)
├── python/
│   ├── pyproject.toml
│   ├── r1_robot/                  Python API (types, limits, safety, backends, robot, gripper, config)
│   └── tests/                     33 offline unit tests
├── config/
│   ├── r1_robot.conf              safety thresholds + backend selection
│   └── joint_calib.example.txt    calibration format (real file must be produced on the arm)
├── HARDWARE_BRINGUP.md            staged, manual real-hardware procedure
└── README.md
```

`config/joint_calib.txt` is git-ignored: a real calibration is machine-specific
and must be produced with the official `calibrate_joints` tool on the arm.

---
, and the SDK submodule.
Nothing else — the build is fully offline (no FetchContent, no GoogleTest).

```bash
cd r1_robot
git submodule update --init --recursive third_party/r1-7a_sdk   # once per clone
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

### The SDK dependency is strict by design

The **only** path used by default is the in-project submodule:

```
${PROJECT_SOURCE_DIR}/third_party/r1-7a_sdk
```

There is deliberately **no** search of `$HOME`, of sibling directories
(`../r1-7a_sdk`, `../r1-7a_sdk-main`, …) or of any other checkout on the
machine. A missing or incomplete submodule is a **hard configuration error**,
never a silent downgrade — an unexpected SDK would change the motor protocol
underneath the safety layer without anyone noticing. The failure prints the
exact `git submodule update --init --recursive` command to run.

Two deliberate escape hatches, both requiring an explicit flag:

```bash
# a different, pinned checkout on purpose (tarball, CI cache, ...)
cmake -S . -B build -DR1_ROBOT_SDK_ROOT=/path/to/r1-7a_sdk

# mock-only build: no real-hardware backend, no SDK required
cmake -S . -B build -DR1_ROBOT_WITH_UNITREE_BACKEND=OFF
```

### The report you should expect

```
-- Official R1 SDK
--   root      : /home/jesse/r1_robot/third_party/r1-7a_sdk
--   origin    : in-project submodule (default)
--   version   : 0.1.0
--   commit    : 480cfc8c031a (2026-09-28:add step file)
--   full hash : 480cfc8c031aaeccddf562b4d789d46b64c6c543
--   state     : clean (read-only)
   ...
--   sdk commit          : 480cfc8c031a
```

The same pin is baked into the binary and printed by `r1_arm_status`:

```
official SDK: official unitreerobotics/r1-7a_sdk version=0.1.0 commit=480cfc8c031a
```

If the checkout is **not** a git repository (e.g. a tarball), configure ends
with a loud warning that the SDK is *not pinned* instead of pretending.

### Read-only enforcement

The vendored `motor_controller` CMakeLists hard-codes its output directory to
its own source `lib/` directory. `r1_sdk_redirect_outputs()` overrides that so
the shared library is written to `build/lib`, and `r1_sdk_cleanliness_check`
(a build target that always runs after `motor_controller`) fails the build if
the checkout is dirty for any reason:

```
-- r1_sdk: checkout is clean - read-only policy honoured
```

Verified: a full build leaves `git -C third_party/r1-7a_sdk status --porcelain`
empty, and planting a stray file inside the submodule makes the build fail.

### Options

| Option | Default | Meaning |
|---|---|---|
| `R1_ROBOT_BUILD_TESTS` | `ON` | build the unit tests |
| `R1_ROBOT_WITH_UNITREE_BACKEND` | `ON` | build the real-hardware backend; set `OFF` for a mock-only build that needs no SDK |
| `R1_ROBOT_SDK_ROOT` | *empty* | explicit override for the SDK path; empty means "use the in-project submodule" |
| `R1_ROBOT_WARNINGS_AS_ERRORS` | `OFF` | treat compiler warnings as errors |

### Pinning

The submodule commit recorded in the r1_robot repository *is* the pin. To move
to a different SDK revision, check out the submodule explicitly and commit the
new gitlink:

```bash
git -C third_party/r1-7a_sdk fetch
git -C third_party/r1-7a_sdk checkout <known-good-commit>
git add third_party/r1-7a_sdk && git commit -m "bump R1 SDK pin"
```

Never edit files inside `third_party/r1-7a_sdk`: `r1_robot` is written against
the pinned upstream layout, and interface differences must be resolved by
choosing the right pin, not by patching the SDK

The vendored `motor_controller` library is built **out-of-tree** into
`build/lib`, so the official checkout stays byte-for-byte untouched.

### SDK dependency audit (this round)

The official SDK was previously available only as an unpacked ZIP
(`/home/jesse/r1-7a_sdk-main`, not a git repository). It is now a **git
submodule** at `third_party/r1-7a_sdk`; the ZIP directory is left in place and
untouched.

| Item | Result |
|---|---|
| Submodule URL | `https://github.com/unitreerobotics/r1-7a_sdk.git` |
| Pinned commit | `480cfc8c031aaeccddf562b4d789d46b64c6c543` (`480cfc8c031a`) |
| Commit subject | `2026-09-28:add step file` |
| SDK `VERSION` | `Version: 0.1.0` |
| Diff vs the ZIP | **none** — `diff -rq` (excluding `.git`/`.vscode`) is empty, i.e. the ZIP already corresponded to the pinned commit |
| Build-time patch | `patches/0001-arm_motor_bus-feedback-stats.patch` (sha256 printed at configure time). Applied to a **build-tree copy** only — the submodule is never modified |

Core sources (`src/arm_motor_bus.cpp`, `include/arm_motor_bus.h`), the
third-party motor driver (`thirdparty/motor_controller/**`) and the URDF/mesh
assets all match the ZIP exactly, so the submodule introduces no interface
change. The build resolves the submodule by default, reports the commit at
configure time and at runtime, and never falls back to the ZIP path.

The single build-time patch exists because upstream exposes no feedback
reception time or counter (only a sticky `ready` flag), which makes stale
feedback undetectable. It adds `ArmMotorBus::feedbackStats()`; the copied files
are patched in `<build>/sdk_patched` and the official checkout stays clean
(enforced by `r1_sdk_cleanliness_check`). See `SAFETY_REVIEW.md` §3b.

### Documentation map

| File | Contents |
|---|---|
| `SIMULATION.md` | Pinocchio kinematics, MuJoCo model, `SimArm`, trajectories, viewer |
| `SAFETY_REVIEW.md` | Independent safety review, fixes, `HARDWARE_UNVERIFIED` risks, and §9 on the Python/C++ control boundary |
| `JOINT_MAPPING.md` | Joint map (URDF ↔ SDK index ↔ motor id ↔ gear ratio ↔ calibration), the three reference poses, unit chain and source contradictions |
| `HARDWARE_BRINGUP.md` | Staged real-arm bring-up procedure (manual, explicit) |

### Test summary

| Suite | Count | Command |
|---|---|---|
| C++ | 103 unit cases / 11 ctest suites (SDK build); 94 / 11 without the SDK | `ctest --test-dir build --output-on-failure` |
| Python | 103 | `cd python && python -m pytest -q` |

Python simulation/dynamics tests skip automatically when MuJoCo or Pinocchio is
not installed. The C++ suite is validated in CI both with the pinned SDK and with
`-DR1_ROBOT_WITH_UNITREE_BACKEND=OFF` (mock only, warnings-as-errors).

### Simulation (MuJoCo + Pinocchio)

Phase-2 simulation groundwork lives in `SIMULATION.md`: Pinocchio FK/Jacobian,
a generated MuJoCo model with a `SimArm` backend, minimum-jerk trajectories
with outcome classification, and `examples/sim_viewer.py`. The model is
cross-validated (MuJoCo FK vs Pinocchio FK) and runs headless.

The independent safety re-review of the phase-1 motion stack — including the
backend-bypass and per-joint feedback holes it found and fixed, and the open
`HARDWARE_UNVERIFIED` items — is in `SAFETY_REVIEW.md`.

### Test

```bash
cd build
ctest --output-on-failure      # 11 suites
./tests/r1_robot_tests --list  # every case
./tests/r1_robot_tests --filter control_service
./tests/r1_robot_tests --filter motion_gate
```

---

## 4. Quick start (offline, no hardware)

```bash
./build/r1_arm_status --safety     # print the effective safety configuration
./build/r1_arm_status --status     # connect the mock, print state + a safety report
```

```python
import r1_robot as rr

robot = rr.R1Robot()                 # MockArm, offline, deterministic
robot.connect()
robot.backend.set_joint_positions([0.18] * 7)
robot.refresh()

robot.enable_motion(human_confirmed=True)          # explicit opt-in
robot.send_action(rr.ArmCommand.position([0.2] * 7, kp=20.0, kd=3.0))
robot.emergency_stop()
```

### C++ example

```cpp
#include "r1_robot/mock_arm.hpp"
#include "r1_robot/r1_arm.hpp"

r1_robot::R1Arm arm(r1_robot::makeMockBackend());
std::string error;
if (!arm.connect(&error)) { /* handle */ }

auto* mock = /* keep your own MockArm pointer if you need to drive time */;
arm.enableMotion(/*human_confirmed=*/true, &error);

r1_robot::JointVector q{};
q.fill(0.2);
arm.sendAction(r1_robot::ArmCommand::position(q, 20.0, 3.0), &error);
arm.emergencyStop();
```

Python tests:

```bash
cd python
python -m pytest -q                                   # or:
python -m unittest discover -s tests -t .
```

### Reproduce everything offline (deterministic, no hardware, no network)

Every safety-critical behaviour on this branch is verifiable without an arm. From
a fresh clone (or after `git submodule update --init --recursive`):

```bash
# 1. C++ WITH the pinned SDK: compiles the real backend + the patched SDK copy,
#    enforces the submodule stays read-only, runs 11 ctest suites.
cmake -S . -B build -DR1_ROBOT_WITH_UNITREE_BACKEND=ON -DR1_ROBOT_WARNINGS_AS_ERRORS=ON
cmake --build build
ctest --test-dir build --output-on-failure            # 103 cases, all pass

# 2. C++ WITHOUT the SDK (offline core only): proves ControlService, the motion
#    gate and the snapshot tests need no vendor dependency.
cmake -S . -B build-mock -DR1_ROBOT_WITH_UNITREE_BACKEND=OFF -DR1_ROBOT_WARNINGS_AS_ERRORS=ON
cmake --build build-mock && ctest --test-dir build-mock  # 94 cases, all pass

# 3. Python: offline logic always runs; MuJoCo/Pinocchio suites skip when absent.
cd python && python -m pytest -q                      # 103 passed

# 4. The two headline demonstrations:
./build/tests/r1_robot_tests --filter control_service  # comm-loss -> fault latch -> refuse motion
./build/tests/r1_robot_tests --filter motion_gate      # bypass attempts are refused
cd python && python -m pytest tests/test_dynamics.py -k collision  # MuJoCo detects a designed collision
```

The same three configurations run in CI (`.github/workflows/ci.yml`), including a
negative test that a stray file written into the SDK checkout fails the build.

---

## 5. Design rules

* **URDF frame, SI units everywhere above the backend.** `q [rad]`, `dq [rad/s]`,
  `tau [Nm]`, `kp [Nm/rad]`, `kd [Nm/(rad/s)]`, joint order `J1..J7`. The motor
  frame (`dir * scale * motor + offset`) never leaves the C++ hardware backend.
* **The backend is a dumb I/O adapter.** It does not validate commands and does
  not decide what may move.
* **`R1Arm` is the choke point for the single-shot command path.** It owns
  validation, motion gating and emergency stop, and hands out only a `const`
  read-only backend handle.
* **`ControlService` owns the periodic, real-time write path.** For continuous
  operation there is exactly one owner of the hardware bus: a `ControlService`
  that runs its own thread, applies the command-timeout watchdog, latches faults
  and buffers trajectories. Upper layers (Python / VLA) submit setpoints and
  heartbeats to it; they never touch the backend. Arming either entry point needs
  the unforgeable `MotionControl` token.
* **Python does not own the real-time loop.** It is for planning, datasets,
  policy inference and analysis. The single documented bridge is a pybind11
  module or a ROS2/ZMQ service in front of the C++ `R1Arm` (phase 3), keeping
  the safety gates on the C++ side.
* **The gripper is a separate CAN device.** It is intentionally *not* wired into
  the arm's serial bus abstraction.

---

## 6. Phase 1 status

See `HARDWARE_BRINGUP.md` §7 for the full checklist. Summary:

### Verified (offline, in this repository)

| Item | Evidence |
|---|---|
| Official SDK public interface understood | `arm_motor_bus.h` read and mirrored; `ArmMotorBus::open()` calling `std::exit(1)` on bad calibration documented and pre-empted |
| SDK pinned as an in-project submodule at a known commit | configure reports `commit`/`full hash`; `unitreeBackendInfo()` prints the same pin at runtime |
| SDK sources stay read-only across a full build | `r1_sdk_redirect_outputs()` + the `r1_sdk_cleanliness_check` build target; verified with a negative test (stray file → build fails) |
| Missing submodule fails loudly with the exact fix command | configure `FATAL_ERROR`; no `$HOME`/sibling fallback |
| Joint limits match the official URDF | `default_limits_match_official_urdf`, `test_limits_match_the_official_urdf` |
| Safety gates reject disconnected / stale / invalid / faulty / undervoltage / out-of-envelope commands | 20 safety tests (C++), 12 (Python) |
| Motion cannot be enabled implicitly | `r1_arm_requires_an_explicit_human_confirmation_to_enable`, `test_actuation_is_refused_before_enable` |
| Real hardware refused without the master switch | `r1_arm_refuses_a_real_backend_without_the_master_switch` |
| `R1Arm` never sends a command that did not pass the envelope | `r1_arm_still_validates_commands_after_enabling` |
| MockArm is deterministic and offline | 11 mock tests, no sleeps, no wall-clock dependence |
| Motion-lock bypass is closed at compile time (not by convention) | `r1_arm_backend_handle_is_read_only`, `enabled_gate_still_bypasses_safety_via_writecommand_so_backend_is_hidden`, `feedback_probe_does_not_arm_motion` |
| `ControlService` watchdog latches on upper-layer silence and never auto-re-arms | `control_service_watchdog_latches_on_silence_then_refuses_motion` (+ stale-feedback / motor-error / port-drop variants) |
| Feedback values + reception evidence come from one atomic snapshot | `snapshot_atomic_all_joints_share_one_clock`, `snapshot_*` suite |
| MuJoCo detects a designed collision scenario | `test_environment_mode_detects_a_designed_tabletop_collision`, `test_self_mode_nominal_is_clean_but_a_fold_is_detected` |
| 103 C++ + 103 Python tests pass (SDK build; 94 C++ without the SDK) | `ctest` / `pytest` |

### To be verified on real hardware (`TODO(bringup)` / 待验证)

* Real bus voltage and the correct `min_supply_voltage` floor.
* Real achievable control rate, feedback latency and a realistic
  `max_feedback_age_s` on the 6 Mbaud bus. The patched SDK now reports per-frame
  reception time via `feedbackSnapshot()` and `UnitreeArmBackend` stamps states at
  the **reception** time (not read time); the accuracy of that reception clock vs.
  the real bus still needs hardware confirmation (`HARDWARE_UNVERIFIED`).
* Real timing of the `ControlService` periodic loop and the command-timeout
  watchdog under a genuinely stalled Python/VLA process.
* Whether `timed_out` should be `true` or `false` for each application.
* Actual torque/velocity capability vs. the URDF `effort`/`velocity` attributes.
* Sign of each joint under gravity (needed before any gravity-holding control).
* A safe `max_joint_step_rad` and the lead-limited creep behaviour of the real
  arm (the SDK examples creep with a 0.2 rad lead; that path is **not** exposed
  in phase 1).
* Whether the arm can be moved safely at all without a brake/gravity-comp
  strategy — this blocks phase 2.
* Gripper (DM4310): CAN interface/bitrate, motor id, MIT parameter ranges, rack
  gear ratio, travel limits, current/velocity limits, homing, thermal model.

---

## 7. Roadmap

| Phase | Content |
|---|---|
| **1 (done)** | Architecture, `R1Arm`, safety framework, `MockArm`, gripper interface, C++ + Python APIs, tests, docs |
| **1.5 (done, this branch)** | P0 hardware control-permission boundary (unforgeable `MotionControl`), standalone C++ `ControlService` (periodic loop, watchdog, fault latch), atomic feedback snapshot (`feedbackSnapshot()` patch v2), real trajectory success predicate, working MuJoCo collision detection, CI + license |
| 2 | Hardware bring-up; gravity compensation; joint-space trajectories with feed-forward on the real arm |
| 3 | Real-time bridge (pybind11 or ROS2/ZMQ) so Python can command the C++ `R1Arm` without owning the loop |
| 4 | ROS2 + MoveIt2, URDF/mesh integration, camera drivers (D435, Astra Pro) |
| 5 | LeRobot datasets, ACT, Diffusion Policy, π₀/π₀.₅ evaluation |

---

## 8. Documentation

* `README.md` — this file.
* `HARDWARE_BRINGUP.md` — staged, manual real-hardware procedure, fault table and
  abort criteria. **Read before powering the arm.**
* `python/README.md` — Python API notes and the C++/Python boundary.

## License

MIT for this project (see [`LICENSE`](LICENSE)). The vendored official SDK keeps
its own BSD-3-Clause license and the required attribution is reproduced in
[`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md), along with the open
(`NOASSERTION`) license questions for the prebuilt `motor_controller` library and
the URDF/STEP/STL design assets.
