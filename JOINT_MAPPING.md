# JOINT_MAPPING.md — joints, SDK indices, calibration and safe home

All facts below are read from the **pinned** SDK checkout
(`480cfc8c031aaeccddf562b4d789d46b64c6c543`) and from `config/joint_calib.example.txt`.
Nothing here is guessed: anything not stated by an official source is marked
**`HARDWARE_UNVERIFIED`**.

Sources used:

| Fact | Source |
|------|--------|
| names, axes, limits, effort, velocity, inertial data | `urdf/.../R1_Robotic_Arm.urdf` |
| same data, independent copy | `urdf/.../R1_Robotic_Arm.csv` (SolidWorks export) |
| joint order | `urdf/.../config/joint_names_R1_Robotic_Arm.yaml` (`['', 'J1'...'J7']`) |
| joint ↔ motor id | `src/arm_motor_bus.cpp` (`motor_ids[i] = i`) |
| gear ratios | `thirdparty/motor_controller/src/MotorProtocol.cpp` (`gearRatio()`) |
| unit conversion at the bus edge | `src/arm_motor_bus.cpp` (`dir*scale`, `+offset`) |
| example calibration | `config/joint_calib.example.txt` (SDK-shipped example) |

## 1. Full joint map

`SDK idx` = 0-based index used by `ArmMotorBus` and by every `r1_robot` array.
`motor id` = the value on the wire (default `motor_ids[i] = i`).
`ratio` = `MotorProtocol::gearRatio(motor_id)` applied **inside** the protocol layer.
`calib (example)` = `dir / scale / offset` from the SDK's example file — **example
values only, not your arm's calibration.**

| Joint | URDF axis | lower | upper | effort [Nm] | velocity [rad/s] | SDK idx | motor id | ratio | calib (example) dir/scale/offset | commandable range (margin 0.05) |
|-------|-----------|-------|-------|-------------|------------------|---------|----------|-------|----------------------------------|---------------------------------|
| J1 | `0 0 -1` | −2.723 | 2.723 | 60 | 18.7 | 0 | 0 | 32 | `-1 / 1 / -2.721` | [−2.673, 2.673] |
| J2 | `1 0 0`  | 0 | 3.1416 | 60 | 18.7 | 1 | 1 | 32 | `+1 / 1 / -0.05` | [0.05, 3.0916] |
| J3 | `0 1 0`  | −0.349 | 3.491 | 33 | 33.4 | 2 | 2 | 38/3 ≈ 12.667 | `+1 / 1 / -0.35011` | [−0.299, 3.441] |
| J4 | `-1 0 0` | 0 | 3.1415 | 33 | 33.4 | 3 | 3 | 38/3 | `-1 / 1 / -0.025` | [0.05, 3.0915] |
| J5 | `0 1 0`  | −2.269 | 2.269 | 33 | 33.4 | 4 | 4 | 38/3 | `-1 / 1 / -2.27451` | [−2.219, 2.219] |
| J6 | `-1 0 0` | −0.524 | 0.524 | 10 | 37.7 | 5 | 5 | 16 | `+1 / 1 / -0.52377` | [−0.474, 0.474] |
| J7 | `0 0 -1` | −1.571 | 1.571 | 10 | 37.7 | 6 | 6 | 16 | `-1 / 1 / -1.57638` | [−1.521, 1.521] |

Buses: default port `/dev/ttyUSB3`, 6 Mbaud, `joint[i] ↔ motor_id = i`.
The URDF/CSV agree on every axis, limit, effort and velocity value (verified).

> ⚠️ **J2 and J4 cannot be commanded to 0.** Their URDF lower bound is 0 and the
> safety layer keeps a 0.05 rad soft margin, so the commandable lower bound is
> 0.05. `KINEMATIC_ZERO` is a FK reference, not a command.

## 2. The three reference poses

Defined in `python/r1_robot/poses.py` and asserted by `python/tests/test_poses.py`.

| Pose | Value [rad] | Purpose | Commandable? |
|------|-------------|---------|--------------|
| `KINEMATIC_ZERO` | `[0,0,0,0,0,0,0]` | FK/Jacobian reference; calibration target | ❌ no (J2/J4) |
| `SIM_INITIAL_POSE` | `[0, 0.3, 0.3, 0.3, 0, 0, 0]` | where simulations start | ✅ yes |
| `SAFE_HOME` | `[0, 0.3, 0.3, 0.3, 0, 0, 0]` | candidate "return to" pose | ✅ yes |

`SAFE_HOME` was chosen to be (a) commandable with margin to spare, (b) near the
middle of each range so the static holding torque stays small — the largest
gravity torque there is ≤ 50 % of the corresponding URDF effort limit
(asserted by a test), and (c) reachable from the simulation start without a
large move.

**`HARDWARE_UNVERIFIED`**: `SAFE_HOME` is validated only against the URDF limits,
the soft margins and the *simulated* gravity load. Cable routing, self-collision,
fixtures, brake behaviour and the real calibration are not covered. Treat it as
the simulation start pose and a *candidate* home, not a proven-safe real-arm pose.

All demo/example waypoints (`SIMULATION.md`, `examples/sim_viewer.py`) are
checked against the soft limits before moving
(`validate_trajectory_poses`, `examples/sim_viewer.py::check_waypoints`).

## 3. Unit chain, and where it is NOT verifiable

```
URDF value  --(dir*scale, +offset; inside ArmMotorBus)-->  output-side value
            --(gear ratio; inside MotorProtocol)-------->  rotor-side wire value
```

* `ArmMotorBus` converts `q`, `dq`, `tau` with `dir*scale`; `q` additionally
  with `offset`. `kp`/`kd` are passed through **unchanged**.
* `MotorProtocol::buildControlPacket()` then converts to wire units, applying
  `position*ratio`, `speed*ratio`, `torque/ratio`, `kp/(ratio²)`, `kd/(ratio²)`.
* `MotorProtocol::parseFeedbackPacket()` applies the inverse
  (`torque*ratio`, `speed/ratio`, ...).

So the gear ratio is applied **once**, inside the protocol; `scale` is a
calibration scale on top of the output-side value, *not* a second gear ratio.

**Finding (corrects an earlier, too-optimistic statement).** The SDK header
documents `kp`/`kd` as `Nm/rad` / `Nm/(rad/s)` in the joint frame, and
`MotorProtocol` divides them by `ratio²` before putting them on the wire. What
the motor firmware then does with that number is **not in this repository**.
Consequently the *realized* joint stiffness/damping cannot be verified offline
and is **`HARDWARE_UNVERIFIED`** — do not assume the value passed in `ArmJointCmd.kp`
is exactly the stiffness the joint exhibits. The safety envelope (`max_kp`,
`max_kd`) is a limit on the *commanded* value only.

## 4. Contradictions / observations between official sources

1. **`dir` cannot be derived from the URDF axis.** J2 (`1 0 0`) and J3 (`0 1 0`)
   have `dir = +1`, while J5 (`0 1 0`) has `dir = -1` and J6 (`-1 0 0`) has
   `dir = +1`. The URDF axis sign therefore does **not** determine `dir`. It is
   a per-arm calibration result (encoder mounting + motor direction) and must
   come from `calibrate_joints`. **Do not guess it.**
2. **Example offsets ≠ URDF limits exactly.** The example offsets track the URDF
   lower bounds but differ by 0.0002–0.006 rad (e.g. J1 −2.721 vs −2.723,
   J3 −0.35011 vs −0.349, J7 −1.57638 vs −1.571). No official document ties the
   two; the example file is explicitly "the values shipped with the SDK example",
   not a calibration for any particular arm.
3. **Default calibration path differs by context.** The header default is
   `../config/joint_calib.txt` (relative to `bin/`), while `r1_robot`'s config
   default is `config/joint_calib.txt` (relative to the process CWD, like the
   official examples). Both resolve relative to CWD once in use; the mismatch is
   a documentation nuance, not a behavioural difference. The backend requires an
   absolute or CWD-relative path and validates it before opening the port.
4. **`MotorFeedback` carries voltage and temperatures that `ArmMotorBus`
   discards.** `MotorFeedback` parses `voltage = data[5]/2` and both temperatures,
   but `ArmJointState` has no such fields, so `SafetyMonitor`'s undervoltage check
   is inert on real hardware (only motor error bit `0x8` is usable). Exposing
   them would need a further SDK patch; deferred (keep the patch minimal).
5. **URDF and CSV agree** on axes, limits, effort, velocity, joint origins and
   inertias — no contradiction there. The CSV's `Safety Soft Upper/Lower` and
   `Safety K Position/Velocity` columns are empty, so they provide no extra
   envelope.

## 5. What is deliberately not claimed

* No real-arm calibration value is produced or invented here.
* No statement that `dir`, `scale`, `offset`, `kp`, `kd`, the torque or the
  velocity limits are correct for the physical arm — all `HARDWARE_UNVERIFIED`.
* The safe home pose is a simulation-validated candidate, not a hardware-verified
  posture.
