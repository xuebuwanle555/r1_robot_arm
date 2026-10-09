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
| 1 | Can a motion command bypass `SafetyMonitor`? | **2 holes found → fixed** |
| 2 | Do calibration / direction / gearing / Kp-Kd-tau units match the SDK? | Logical mapping verified; values `HARDWARE_UNVERIFIED` |
| 3 | Can state feedback be stale? Timestamps & thread safety? | Stale detection is a **real gap** on the unitree backend; R1Arm race **fixed** |
| 4 | Does the low-level controller watchdog on its own? | Yes by design; timing `HARDWARE_UNVERIFIED` |
| 5 | Are stop / FOC-hold / zero-torque / disable conflated? | No; distinguished + tested |
| 6 | Are trajectory timeout / not-reached / motor-fault detected? | No layer existed → **added + tested** |
| 7 | Does the read-only diagnostic tool change motor modes? | No (only an explicit, twice-confirmed `--probe`) |
| 8 | Can the "no real hardware by default" rule be bypassed via the backend? | Yes before → **fixed** (backend-level gate) |

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

### 1c. Remaining, by design

* `UnitreeArmBackend::probeFeedback()` is callable directly. It sends **one
  zero-torque FOC frame** per joint (compliant, bridges powered) and is the
  lowest-energy way to wake up feedback. It is never automatic and is documented
  as a deliberate manual step (`HARDWARE_BRINGUP.md` §2). It is *not* gated by
  `MotionGate` because arming it through `R1Arm` would require enabling motion —
  which is a larger hazard than the probe itself.
* A stop command is accepted in every state, on purpose: a fault must never
  block the way back to a safe state.

---

## 2. Calibration, direction, gearing, units (vs the official SDK)

Verified by reading the SDK sources (`third_party/r1-7a_sdk/src/arm_motor_bus.cpp`,
`include/arm_motor_bus.h`):

* The bus edge converts `q`, `dq`, `tau` with `dir * scale` and `q` additionally
  with `offset`: `urdf = dir*scale*motor + offset`. `r1_robot` passes URDF-frame
  values in and receives URDF-frame values out — the motor frame never leaks
  above the backend. ✔
* `kp`/`kd` are **passed through unchanged**. This is correct for joint-frame
  stiffness/damping: with `τ_motor = kp·(q_err,motor)` and
  `q_err,motor = q_err,urdf / (dir·scale)`, the joint torque is
  `τ_joint = (dir·scale)·τ_motor = kp·q_err,urdf`. The same holds for `kd`. So
  the *joint-frame* gains equal the packet gains; no hidden `scale²` factor. ✔
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

### 3b. Stale feedback on the real backend (real gap — `HARDWARE_UNVERIFIED`)

`MockArm` reports honest timestamps (and `feedback_delay_s` can force staleness);
`SafetyMonitor::checkState` correctly faults on `state.stale_feedback` — this is
covered by tests.

The **unitree** backend cannot currently detect staleness:

* the SDK's `fb_ready[]` flag is set once on the first packet and **never
  cleared**, so `RobotState::valid` is sticky `true` even if the bus dies;
* the SDK exposes no per-frame reception timestamp, so `readState()` stamps the
  state at read time (`nowNs()`), which makes the freshness check vacuous.

**Status: `HARDWARE_UNVERIFIED`, open.** Closing it needs a reception counter or
timestamp from the SDK; the SDK is read-only, so this cannot be fixed in
`r1_robot` alone. Tracked as a TODO in `src/unitree_backend.cpp`. Until then, do
not treat `state.valid == true` on real hardware as proof that feedback is
current.

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

## Open risks (`HARDWARE_UNVERIFIED`)

1. **Stale-feedback detection on the real bus** (§3b) — the most important open
   item; needs SDK support for a reception counter/timestamp.
2. **Calibration values** (`dir/scale/offset`) and joint direction signs (§2).
3. **Effort / velocity limits** are the URDF values, not verified against the
   actuators and power supply (§2); the safety envelopes
   `max_kp/kd/tau/velocity`, `min_supply_voltage` and `max_feedback_age_s` are
   conservative placeholders.
4. **Watchdog timing** (§4).
5. **Voltage/temperature** are not exposed by the SDK, so the undervoltage check
   is inert on real hardware (only the motor error bit `0x8` is usable).
6. Soft-limit zero pose: J2/J4 cannot be commanded to `q = 0` under the default
   margin — a UX constraint to respect in any trajectory (§2 of `SIMULATION.md`).
