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
  `tests/test_trajectory.py::TestNoPythonHardwarePath`. The opt-in pybind11
  bridge below reaches hardware *semantics* only through the C++
  `ControlService`/`IArmBackend` gates — and it is wired to `SimArm`, never to
  the real backend, so this property is unchanged.
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

**Required for real hardware (partly demonstrated here).** A real device must be
driven by a **dedicated C++ control service** that owns the real-time loop, the
watchdog policy and the fault arbitration; Python becomes a client that submits
targets and trajectories over a link that cannot command motors directly. This
branch adds an **opt-in pybind11 bridge** (`-DR1_ROBOT_BUILD_PYTHON_BINDING=ON`,
off by default) that runs exactly that C++ `ControlService` against the real
MuJoCo `SimArm` plant: Python submits a trajectory and heartbeats, `run_cycle()`
performs the C++ control cycle, and the command-timeout watchdog is exercised
against the **real monotonic clock** (no virtual-clock stubbing of the safety
path). `python/tests/test_control_binding.py` shows the full sequence — physics
tracks the goal, upper-layer silence latches `comm.watchdog_timeout`, motion is
refused until an operator clears and re-enables, then resumes. The bridge is a
*simulation* validation only; a real transport (ROS2/ZMQ) in front of the same
service is still phase-3 work.

**Timing.** The MuJoCo timestep (2 ms / 500 Hz) is a *simulation* choice and says
nothing about the real command rate. `ArmMotorBus::step()` sleeps 115 µs after
every joint write — a ≥ 805 µs floor per 7-joint frame before serial time and
motor turnaround — so a 500 Hz real loop must not be assumed; it must be
measured (`test_dynamics.py::TestControlFrequencyAssumptions` documents the
arithmetic).

---

## 10. P0 review of the stop state machine, admission and loop reliability

This round re-reviewed `ControlService` for the failure modes that a safety
stack actually hits, and found four real defects. Each was reproduced by a test
that failed against the previous code before it was fixed.

**10a - the controlled stop dropped the drive on a timer (§A).**
`kControlledStop` concluded from `|dq| < stop_vel_tol` **plus** `t -
stop_engaged >= 500 ms` that it could `setMotionArmed(false)` and send mode 0.
Both premises are wrong. Velocity is exactly zero at the instant a
gravity-loaded arm begins to fall, so stillness is not evidence that removing
torque is safe; and `connected == false` was mapped to "settled", i.e. losing the
transport was reported as a completed stop. Now:

* the stop holds under active FOC, open-ended;
* `motor_disabled` is reachable only through `confirmMotorDisable()`, an explicit
  operator call that refuses unless feedback is currently valid **and** every
  joint has been observed still for the whole `stop_settle_ns` window without a
  break (a lost-feedback or still-moving joint both withdraw prior verification);
* `StopReason` separates operator stop / upper-layer timeout / feedback lost /
  actuator fault / unsafe state, so "we stopped it" and "we can no longer see it"
  are different facts downstream;
* the hold target is the last **commanded** setpoint, never stale measured `q` —
  re-aiming at feedback that just went bad would command a jump.

MuJoCo evidence (`test_dynamics.py::TestGravityLoadAtDriveRemoval`): settled
under FOC the arm sits at `max|dq| = 3.3e-6` rad/s; 0.3 s after mode 0 it is at
`4.8` rad/s with J3 down **0.75 rad**. Stillness predicted nothing. Removing the
drive remains `HARDWARE_UNVERIFIED` as a way to hold the arm — it is now
disproved by test rather than only disclaimed in a comment.

**10b - trajectories were admitted almost unchecked (§B).**
`submitTrajectory`/`submitCommand` returned `void`; only the soft limits were
examined, there was no finiteness, timing, count, horizon, velocity or
acceleration check, and the buffered plan started at the first waypoint
regardless of where the arm was (`last_target_` was overwritten with it, so the
anti-step ramp could not smooth the jump). An out-of-limit destination *latched a
fault and stopped the arm* for an upload that was never executed. Now both entry
points return `SubmitResult{status, error, waypoint_index}`, full validation runs
before anything is stored, a rejection latches nothing, and the start transition
is a synthesized, velocity-limited **lead-in** that shifts the whole plan later
rather than bending its timings — or is refused if it does not fit the budget.
Submission is also refused while feedback is invalid, so nothing is ever planned
off stale data.

**10c - the loop drifted and swallowed send failures (§C).**
`threadMain` slept the period *after* each cycle, so the real interval was
`period + work`; `writeCommand()` returns `bool` and its result was discarded,
making a frame that never reached the arm indistinguishable from one that did.
Now scheduling is on absolute deadlines with rebase-on-miss (no catch-up burst),
a failed send latches `bus.send_failed`, and persistent lateness latches
`ctrl.deadline_missed` after `max_consecutive_overruns`. `ControlLoopMetrics`
reports cycles, period, overruns, send failures, exec time, worst jitter and
measured rate — **observations, not real-time guarantees**; a `std::thread` on a
general-purpose kernel is not a hard real-time context and nothing here claims it
is.

**10d - the state lock was held across backend I/O (§C/§D).**
`runCycle` took `mutex_` for the whole cycle including `readState()`/
`writeCommand()`. For the pybind bridge those calls enter Python, so a Python
that blocks would also have blocked `status()`, `latestState()` and the
operator's `requestControlledStop()`. `mutex_` now guards state only and
`io_mutex_` serialises the backend, never held simultaneously; `runCycle` is
observe → decide → act. `TestIndependentWatchdog` proves the watchdog fires from
the service's own thread with Python calling nothing at all — the synchronous
`run_cycle()` tests could only ever show that the service reacts when Python
agrees to schedule it.

**Structural limit of the bridge, stated plainly.** Every cycle of a threaded
bridge re-acquires the GIL to enter Python. If the main interpreter is stuck
*while holding the GIL*, the C++ watchdog cannot run. That is why this binding is
a validation instrument and must never be wired to a real arm; the hardware
service needs a backend that does not depend on the Python interpreter yielding.

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
10. **Drive removal after a verified stop is still unproven.** `confirmMotorDisable`
    gates the decision on evidence, but whether mode 0 leaves an R1 arm holding
    its pose under gravity is exactly the thing no test can answer here; the
    MuJoCo result in §10a says it falls, and a real arm with brakes/cabling may
    behave differently in either direction.
11. **Admission thresholds are placeholders.** `max_joint_accel_rad_s2` (5.0),
    `max_consecutive_overruns` (5), `traj_lead_in_max_ns` (5 s),
    `stop_settle_ns` (500 ms) and `overrun_tolerance_ns` (period/2) are
    engineering guesses. The velocity bound reuses the URDF `velocity` fields,
    which are themselves unverified against the actuators (item 4).
12. **The anti-step bound conflicts with a lagging plant.** `checkCommand`
    compares command against *measured* position, so a plant that lags by more
    than one `max_joint_step_rad` cannot be commanded forward at all — observed
    directly while driving MuJoCo physics, and worked around in tests by
    loosening the bound rather than by measuring a real value.
13. **Real loop timing is unmeasured.** `ControlLoopMetrics` now makes rate,
    exec time and jitter observable, but no figure in this repository comes from
    a 6 Mbaud bus with a real `ArmMotorBus::step()` floor (≥805 µs per 7-joint
    frame, §Timing above), so the achievable period is unknown.
14. **Gripper model cannot be driven yet.** The combined prototype model loads
    and its mimic compiles, but it declares no actuators (`nu = 0`) and its
    massless frames — including `prototype_tcp` — are merged away at compile
    time, so there is no addressable TCP body. Grasp-force claims are out of the
    question until both are fixed; `effort = 8` in that URDF is not a measured
    grasp force (`SIMULATION.md` §8, `python/r1_robot/gripper_model.py`).
