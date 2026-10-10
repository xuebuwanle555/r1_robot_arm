# SAFETY_REVIEW.md — independent review of the phase-1 motion stack

Scope: an adversarial re-review of `SafetyMonitor` / `R1Arm` / `MockArm` /
`UnitreeArmBackend` beyond "the tests pass". Every claim below is either
(i) verified offline by a new regression test, or (ii) marked
**`HARDWARE_UNVERIFIED`** and left open on purpose.

No safety check was removed or weakened to make anything pass.

---

## Summary

| # | Question | Verdict |
|---|----------|---------|
| 1 | Can a motion command bypass `SafetyMonitor`? | **2 holes found → fixed**; further closed this round (§1c) |
| 2 | Do calibration / direction / gearing / Kp-Kd-tau units match the SDK? | Mapping verified; realized gains `HARDWARE_UNVERIFIED` |
| 3 | Can state feedback be stale? Timestamps & thread safety? | **Real gap → fixed**; now an atomic `feedbackSnapshot()` (value+evidence under one lock, one clock); R1Arm race **fixed** |
| 4 | Does the low-level controller watchdog on its own? | **`ControlService` added**: periodic thread + command-timeout watchdog + fault latch; timing `HARDWARE_UNVERIFIED` |
| 5 | Are stop / FOC-hold / zero-torque / disable conflated? | No; distinguished + tested, now incl. software-hold / controlled-stop / motor-disabled |
| 6 | Are trajectory timeout / not-reached / motor-fault detected? | **Strengthened**: REACHED needs position + velocity + sustained dwell; timeout can no longer grant REACHED |
| 7 | Does the read-only diagnostic tool change motor modes? | No — `probeFeedback` now needs the `MotionControl` token via `R1Arm` |
| 8 | Can the "no real hardware by default" rule be bypassed via the backend? | Yes before → fixed (backend gate) → **this round: arming/probing needs an unforgeable token; `backend()` is `const`** |
| 9 | Can Python drive real hardware? Where does safety authority live? | No Python hardware path; authority is C++; **guarded by a test** |
| 10 | Can the simulator confirm the arm stops at / collides with obstacles? | **Fixed**: collisions were a silent no-op; environment + self modes now authored in XML and tested |

---

## 1. Bypass paths around `SafetyMonitor`

### 1a. Direct backend access (found, fixed)

`R1Arm::backend()` returned the raw backend pointer, and no backend enforced
arming. A caller could therefore do:

```cpp
auto b = makeBackend({"unitree"});
b->connect("...");                        // bypasses R1Arm entirely
b->writeCommand(ArmCommand::position(q, 50, 5));   // drives real motors
```

**Fix.** A fail-closed `MotionGate` (`include/r1_robot/motion_gate.hpp`) is now
embedded in `UnitreeArmBackend`. `writeCommand()` refuses every *actuating*
command while the gate is disarmed; a stop command always passes. Only
`R1Arm::enableMotion(human_confirmed=true)` arms the gate; `disableMotion`,
`emergencyStop`, `disconnect`, the destructor and `connect` disarm it.
`IArmBackend` gained `setMotionArmed()` / `motionArmed()` (safe defaults for
non-hardware backends), and `R1Arm::sendAction` additionally requires
`backend_->motionArmed()`.

Regression tests (`tests/test_hardware_gate.cpp`): a gated real backend refuses
direct actuating writes while disarmed, accepts them once armed, and returns to
refusing after `disableMotion()`/`emergencyStop()`.

### 1b. Per-joint feedback hole in `checkCommand` (found, fixed)

With `require_all_joints_valid = false` (a supported setting), a joint with no
valid feedback produced only a warning in `checkState`. In `checkCommand` the
anti-step check was then **skipped** for that joint (`if (state.valid[si])`), and
the soft-limit check is about the target — so an arbitrarily large jump on an
unmeasured joint was accepted.

**Fix.** `checkCommand` now emits a fault `cmd.no_feedback_for_joint` for any
actuating joint (`kp > 0`) without valid feedback, independent of
`require_all_joints_valid`. Regression test
`safety_rejects_actuation_for_a_joint_without_feedback` fails without the fix.

### 1c. probeFeedback and the backend handle (found, fixed this round)

Earlier this review accepted two things "by design" that this round closes, per
the request that the control-permission boundary must not rely on convention:

* `UnitreeArmBackend::probeFeedback()` used to be callable directly on the
  concrete backend and bypassed `MotionGate`. It now takes an unforgeable
  `MotionControl` token and is reachable only through
  `R1Arm::probeFeedbackForBringup(human_confirmed, …)`, which additionally
  requires `allow_real_hardware` for a real backend. Ordinary business code
  cannot construct the token, so it cannot power the motor bridges.
* `R1Arm::backend()` used to hand out a writable `ArmBackendPtr`; a caller could
  invoke `setMotionArmed(true)` on it. It now returns
  `std::shared_ptr<const IArmBackend>`, so `writeCommand` / `stop` /
  `setMotionArmed` / `probeFeedback` (all non-const) are ill-formed through it —
  a read-only status handle by construction. Arming needs the token; the token's
  only constructor is private and befriends `R1Arm` and `ControlService`.

Negative regressions: `r1_arm_backend_handle_is_read_only`,
`enabled_gate_still_bypasses_safety_via_writecommand_so_backend_is_hidden`,
`feedback_probe_does_not_arm_motion`, `enable_motion_is_the_only_armed_transition`
(`tests/test_hardware_gate.cpp`).

### 1d. Still accepted by design

* A pure **stop** command is accepted in every state, on purpose: a fault must
  never block the way back to a safe state. It only removes energy.
* With motion enabled the backend gate is armed, so a caller who *somehow* holds
  a non-const backend could push an out-of-envelope frame past `SafetyMonitor`
  via `writeCommand`. This is the hole §1c's `const` handle closes: `R1Arm`
  no longer exposes a writable backend, and `ControlService` never exposes one at
  all (it is the sole owner). The regression documents the residual hazard so the
  handle contract cannot quietly regress.

---

## 2. Calibration, direction, gearing, units (vs the official SDK)

Verified by reading the SDK sources (`third_party/r1-7a_sdk/src/arm_motor_bus.cpp`,
`include/arm_motor_bus.h`):

* The bus edge converts `q`, `dq`, `tau` with `dir * scale` and `q` additionally
  with `offset`: `urdf = dir*scale*motor + offset`. `r1_robot` passes URDF-frame
  values in and receives URDF-frame values out — the motor frame never leaks
  above the backend. ✔
* `kp`/`kd` are **passed through unchanged by `ArmMotorBus`**, but the wire
  encoding in `MotorProtocol::buildControlPacket()` divides them by `ratio²`
  (and scales position by `ratio`, velocity by `ratio`, torque by `1/ratio`).
  The gear ratio is applied exactly **once**, inside the protocol layer; `scale`
  is a calibration scale on top of the output-side value, not a second ratio.
  What the motor firmware does with the `kp/(ratio²)` number is **not in this
  repository**, so the *realized* joint stiffness is **`HARDWARE_UNVERIFIED`**.
  Do not assume the value handed to `ArmJointCmd.kp` equals the stiffness the
  joint exhibits; the configured `max_kp`/`max_kd` bound the *commanded* value
  only.  (This corrects a too-optimistic claim made earlier in this document.)
* `scale` is `1.0` in the SDK example calibration and `dir` is ±1; the IM6014
  reduction ratio is handled inside the motor firmware and is **not** exposed by
  the SDK, so `r1_robot` cannot (and must not) apply it again. ✔
* `calibrationFileUsable()` pre-checks the file because `ArmMotorBus::open()`
  calls `std::exit(1)` on failure — the library must not kill the host. ✔

**`HARDWARE_UNVERIFIED`:** the real `dir/scale/offset` values must come from
`calibrate_joints` on the actual arm; joint gravity-load direction sign; real
bus voltage; achievable control rate; feedback latency; whether `timed_out`
should be true/false for a given application. These require the bring-up
procedure and must not be assumed safe from simulation.

---

## 3. Stale feedback, timestamps, thread safety

### 3a. R1Arm was not thread-safe (found, fixed)

`state_`, `status_`, `motion_enabled_` were read/written from multiple methods
with no synchronization, so a control thread and a UI/status thread could race.

**Fix.** `R1Arm` now holds a `std::mutex`; `status()`, `limits()`,
`safetyConfig()` and `backend()` return **by value** (no dangling reference
under lock), and all mutating paths lock. A multithreaded stress test was added.

### 3b. Stale feedback on the real backend (FIXED, with a documented SDK patch)

The original gap: upstream `ArmMotorBus` exposes only a **sticky** `fb_ready[]`
flag (set once, never cleared) and **no reception timestamp**, and the backend
stamped the state at read time — so a bus that died after the first frame looked
identical to a healthy one and the freshness check was vacuous.

**Confirmed by reading the SDK:** `MotorProtocol::parseFeedbackPacket()` returns
only parsed values; `SerialPort`'s receive callback carries no timestamp;
`ArmMotorBus::Impl::onPacket()` only sets `fb_ready[joint] = true`.  There is no
per-joint reception time, sequence number or counter anywhere.

**Fix.** A minimal, versioned patch
(`patches/0001-arm_motor_bus-feedback-stats.patch`, sha256 printed at configure
time) adds `ArmMotorBus::FeedbackStats { packets, received, age_ns }` and
`feedbackStats()`, recording `steady_clock` reception time + packet count in
`onPacket`.  The submodule is **never modified**: CMake copies the two touched
files into `<build>/sdk_patched`, applies the patch there, verifies
`feedbackStats()` exists, and compiles the copy.  `SdkJointBus` adapts it to the
SDK-free `IJointBus` interface.

`UnitreeArmBackend::readState()` now derives each joint's timestamp as
`now - age_ns` (the real reception time, **never the read time**) and publishes
`RobotState::joint_stamp_ns[i]`; the global `stamp_ns` becomes the stalest
joint's, so the existing global check still catches a single silent joint.
`SafetyMonitor` adds a per-joint fault `state.joint_stale_feedback` naming the
joint — but only for backends that actually publish per-joint stamps, so those
that cannot are not double-reported.

**Tests** (`tests/test_feedback_freshness.cpp`, fake bus injected through
`UnitreeBackendConfig::bus_factory`): reception-time stamping (and that it does
*not* advance when no frame arrives), the freshness budget boundary, partial
joint loss (named joint), a never-received joint, CRC-rejected frames, motor
errors, port disconnect (reported as not-connected, actuation refused, stop still
allowed), and **recovery never re-arms motion**.

**Still `HARDWARE_UNVERIFIED`:** the real frame rate, latency and the correct
`max_feedback_age_s` for a 6 Mbaud half-duplex bus must be measured on hardware.

---

## 4. Independent watchdog at the controller

The SDK's `ArmMotorBus::step()` builds every FOC packet with `timeout = true`
(verified in `src/arm_motor_bus.cpp`), so each frame carries the **motor's own**
communication-timeout protection. If the host stops calling `step()` (e.g. the
Python layer stops sending), the motors time out on their own rather than
holding the last command indefinitely. `r1_robot` never sets
`ArmCommand::timed_out = true`, so it never silently downgrades a real command
into the SDK's zero-torque branch.

**`HARDWARE_UNVERIFIED`:** the timeout duration, and whether it degrades to
compliant/disabled, must be confirmed on the real arm.

---

## 5. stop / FOC-hold / zero-torque / motor-disable are distinct

* `ArmCommand::stop()` → mode 0 on every joint: motor output **disabled**. It is
  **not a brake** and will not hold an arm against gravity (documented
  throughout, incl. `HARDWARE_BRINGUP.md`).
* `ArmCommand::zeroTorque()` → mode 1 with `kp = kd = tau = 0`: **compliant**;
  the bridges are powered and the arm will not hold itself up. It counts as
  actuation (`hasActuation() == true`) and is warned about
  (`cmd.compliant_foc`).
* `ArmCommand::position(q, kp>0, kd)` → mode 1 with position **authority**: this
  can hold.

New test `stop_zero_torque_and_position_are_distinct` asserts all of the above,
including that only the compliant command produces the warning.

---

## 6. Trajectory outcome detection (added)

There was no trajectory layer. `python/r1_robot/trajectory.py` adds
`JointTrajectory` (minimum-jerk) and `TrajectoryExecutor`, which classifies the
result as `REACHED` / `NOT_REACHED` / `TIMEOUT` / `MOTOR_FAULT` / `REJECTED`.
`MOTOR_FAULT` is checked *before* the safety layer rejects a command so the
diagnosis stays specific. Each outcome has a dedicated test
(`python/tests/test_trajectory.py`), including that a fault present before
enabling blocks motion at the gate.

---

## 7. Read-only diagnostic tool

`examples/arm_status.cpp` never enables motion and never sends a position,
torque or home command. Its only actuating capability is `--probe`, which
requires the explicit second flag `--yes-i-know-this-powers-the-motors` and
sends a single zero-torque FOC frame (compliant). Reading state uses the SDK's
cached `states()` (no bus writes). ✔ No change needed.

---

## 8. "No real hardware by default" cannot be bypassed

* Default `SafetyConfig::allow_real_hardware == false`; `R1Arm::connect()`
  refuses a backend whose `isRealHardware()` is true. ✔
* The remaining direct-object route is now closed by the backend-level
  `MotionGate` (§1a): even with the raw pointer, actuation is impossible until
  something explicitly arms it, and only `R1Arm` does. ✔

---

## 9. Python / C++ control boundary (P2 review)

**Where safety authority lives.** All motion gating, the arming latch, the
safety monitor and the bus I/O are **C++** (`r1_robot_core` +
`r1_robot_unitree_backend`). Python owns *planning*: it produces joint targets
and trajectories; it does not own the hardware control loop or its timing.

* **Python cannot drive real hardware today.** The only Python backends are
  `MockArm` (simulation) and `SimArm` (MuJoCo, `is_real_hardware == False`), plus
  `NullBackend`, whose `is_real_hardware` is `True` but whose every method
  raises. There is no Python serial/SDK path at all. Guarded by
  `tests/test_trajectory.py::TestNoPythonHardwarePath`.
* **`TrajectoryExecutor` never talks to a backend directly.** It takes an
  `R1Robot` and sends every frame through `R1Robot.send_action()`, so the soft
  limits, gain/torque envelope, anti-step rule, freshness and motor-error checks
  all apply. It cannot arm motion: the caller must call
  `enable_motion(human_confirmed=True)`.
* **Interface parity.** `MockArm`, `SimArm` and the C++ `IArmBackend` expose the
  same logical surface (`name`, `is_real_hardware`, `connect`, `read_state`,
  `write_command`, `stop`, calibration, `advance` for deterministic time). The
  differences that matter: `SimArm` uses MuJoCo physics while `MockArm` uses a
  one-line-per-joint model; `MockArm`/`SimArm` report voltage and temperature,
  the real SDK does not; only the real backend has an arming gate and a serial
  transport.
* **Real-hardware mode remains human-gated.** `allow_real_hardware` defaults to
  `false`; `enableMotion(human_confirmed=true)` is required; the backend gate is
  armed only there. Nothing in Python can bypass this — see §1a and §8.

**Required for real hardware (not implemented here).** A real device must be
driven by a **dedicated C++ control service** that owns the real-time loop, the
watchdog policy and the fault arbitration; Python becomes a client that submits
targets and trajectories over a link that cannot command motors directly. Phase
1/2 deliberately stops at "the C++ library can do this correctly"; the service
is phase-3 work.

**Timing.** The MuJoCo timestep (2 ms / 500 Hz) is a *simulation* choice and says
nothing about the real command rate. `ArmMotorBus::step()` sleeps 115 µs after
every joint write — a ≥ 805 µs floor per 7-joint frame before serial time and
motor turnaround — so a 500 Hz real loop must not be assumed; it must be
measured (`test_dynamics.py::TestControlFrequencyAssumptions` documents the
arithmetic).

---

## Open risks (`HARDWARE_UNVERIFIED`)

1. **Realized joint gains.** `kp`/`kd` reach the wire divided by `ratio²` inside
   `MotorProtocol`; the firmware's interpretation is not in the repository, so
   the effective stiffness/damping must be measured on hardware (§2, §3 of
   `JOINT_MAPPING.md`).
2. **Calibration values** (`dir/scale/offset`) and joint direction signs. `dir`
   cannot be derived from the URDF axis sign — it must come from
   `calibrate_joints`; do not guess (`JOINT_MAPPING.md` §4).
3. **Freshness budget / real bus timing.** The mechanism is now correct, but
   frame rate, latency and a safe `max_feedback_age_s` for the 6 Mbaud
   half-duplex bus must be measured (§3b).
4. **Effort / velocity limits** are the URDF values, not verified against the
   actuators and power supply; the envelopes `max_kp/kd/tau/velocity`,
   `min_supply_voltage` and `max_feedback_age_s` are conservative placeholders.
5. **Watchdog timing** (§4).
6. **Voltage/temperature** are parsed by `MotorProtocol` but dropped by
   `ArmMotorBus`, so the undervoltage check is inert on real hardware (only motor
   error bit `0x8` is usable). Closing this needs a further SDK patch.
7. **Self-collision is now detectable but only approximately.** This round found
   the earlier "no self-contact in MuJoCo" result was an artifact: the old
   `enable_collision` wrote `geom_contype` *after* the model compiled, which is a
   silent no-op in MuJoCo 3.x. Collision masks are now authored in the generated
   XML, and `test_dynamics.py::test_runtime_mask_write_does_not_enable_contacts`
   pins that fact. `environment` mode reliably detects a designed tabletop/floor
   collision; `self` mode detects a real fold (wrist across the upper arm) after
   excluding the jointless/adjacent pairs and the one concentric `Link5↔Link7`
   convex-hull artifact. Caveat: the collision geometry is the **convex hull of
   each concave visual STL**, so self-collision is conservative — it can
   over-report near-adjacent links and miss thin interpenetrations. Dedicated
   primitive collision meshes would be needed for a tight guarantee.
8. Soft-limit zero pose: J2/J4 lower limits are `0`, so under the default
   `soft_limit_margin` a commanded `q = 0` is *outside* the soft range and is
   rejected. Trajectory/hold tests must start from an in-range pose (the C++
   `ControlService` tests park every joint at 0.1 rad). Constraint to respect in
   any trajectory (`JOINT_MAPPING.md` §1).
9. `SAFE_HOME` is simulation-validated only; cable routing, fixtures, brake
   behaviour and real self-collision are not covered (`JOINT_MAPPING.md` §2).
