# HARDWARE_BRINGUP.md

Staged bring-up procedure for the Unitree R1-7A with `r1_robot`.

> ## STOP — read this first
>
> * `r1_robot` **cannot** home the arm, hold it against gravity, or move it
>   safely out of an arbitrary pose in phase 1. Those capabilities require the
>   model layer and a gravity-compensation strategy planned for phase 2.
> * **`stop()` is not a brake.** It sends mode 0 (motor output disabled). A
>   multi-joint arm under gravity will collapse when you call it. Plan for that
>   before every test: support the arm, or keep it in a pose where gravity
>   cannot accelerate it into a limit or a person.
> * Every step below is **manual and explicit**. Nothing in this repository
>   enables motors, homes the arm or sends zero torque automatically.
> * Do not skip a stage because the previous one "looked fine". The stages exist
>   so that a failure happens with the least possible energy in the system.

Legend: **待验证** = unverified on real hardware; `TODO(bringup)` = the matching
code marker.

---

## 1. Preconditions

### Hardware

- [ ] The arm is mechanically supported (fixture, stand or a second person). It
      must not be able to fall onto anything or anyone if all joints go limp.
- [ ] A working **emergency stop** path that removes motor power physically
      (PSU cut-off / e-stop button). A software stop is *not* an e-stop.
- [ ] Power supply sized for 7 joints pulling torque simultaneously.
      TODO(bringup) 待验证: the required bus voltage/current — read it from the
      vendor documentation and the `R1_LIMITS_SOURCE` numbers, do not guess.
- [ ] Nothing is inside the arm's swept volume.
- [ ] The gripper is **not** attached / not powered. It is a separate DM4310 CAN
      device (see §8) and must not share the arm's bus.

### Host

- [ ] Ubuntu 22.04, x86_64.
- [ ] `ls /dev/ttyUSB*` shows the arm's adapter.
- [ ] Serial access:
  ```bash
  sudo usermod -aG dialout "$USER"     # then log out and back in
  # or, for a quick single session:
  sudo chmod 666 /dev/ttyUSB3
  ```
- [ ] The arm adapter is not claimed by another process
      (`sudo lsof /dev/ttyUSB3`, `ModemManager` disabled if it interferes).

---

## 2. Stage 0 — validate the official SDK on its own

Never bring up your own wrapper before the vendor tool works.

The SDK repository ships **prebuilt** vendor tools in its `bin/` directory, so
try those first — they need no build at all:

```bash
cd third_party/r1-7a_sdk/bin
./test_arm_lowlevel --clear        # clear faults, read error codes
./test_arm_lowlevel --reset        # reset motors
```

If you do need to rebuild them, build in a **scratch copy**, never inside the
submodule:

```bash
cp -a third_party/r1-7a_sdk /tmp/r1_sdk_vendor
cd /tmp/r1_sdk_vendor && mkdir -p build && cd build && cmake .. && make -j
cd /tmp/r1_sdk_vendor/bin
./test_arm_lowlevel --clear
./test_arm_lowlevel --reset
```

> ⚠️ **Why a scratch copy?** The vendor `CMakeLists.txt` writes its outputs into
> its own source tree (`bin/`, `thirdparty/motor_controller/lib/`), and
> `calibrate_joints` writes a *tracked* file (`config/joint_calib.txt`).
> Building or calibrating in place therefore dirties the submodule, and
> `r1_robot`'s build guard would then (correctly) refuse to build. If it
> happens, restore it:
>
> ```bash
> git -C third_party/r1-7a_sdk checkout -- .
> git -C third_party/r1-7a_sdk clean -fd
> ```

Expected: motor feedback is printed for all 7 joints. If not, the problem is in
wiring / port / baud / power — **stop and fix it here**, not in `r1_robot`.

Notes verified from the official sources:

* The default port is `/dev/ttyUSB3` at **6 Mbaud**; joint `i` maps to motor id
  `i`.
* **Motors only transmit feedback after receiving a control packet.**
* `ArmMotorBus::open()` calls `std::exit(1)` when the calibration file is
  missing or incomplete. `UnitreeArmBackend` pre-checks the file so a library
  cannot kill the host process, but the underlying behaviour is the vendor's.

> ⚠️ `--zero-torque`, `--foc`, `--home` and `--goto` from the SDK examples
> **power the motors**. Read §6 before running any of them.

---

## 3. Stage 1 — joint calibration

The calibration maps the motor frame onto the URDF frame:

```
urdf = dir * scale * motor + offset
```

1. Power the arm, then **mechanically hold it at the desired URDF zero pose**.
   With no calibration loaded yet, do not command motion.
2. Run the official tool from the **scratch copy** of §2 — note `--out` points
   *outside* the SDK checkout, straight into `r1_robot`:
   ```bash
   cd /tmp/r1_sdk_vendor/bin
   ./calibrate_joints --port /dev/ttyUSB3 \
       --out /home/jesse/r1_robot/config/joint_calib.txt
   ```
3. Inspect the result:
   ```bash
   cat config/joint_calib.txt
   ```
   The file must contain one line per joint (`0..6`), each
   `joint dir scale offset`. `config/joint_calib.example.txt` shows the format
   and the values shipped with the SDK — **those are not your arm's values**.
4. Sanity check: with the arm held at URDF zero, every `urdf` value computed from
   the calibration must be ≈ 0.
5. Confirm the SDK checkout is still clean — the calibration must not have been
   written inside it:
   ```bash
   git -C third_party/r1-7a_sdk status --porcelain    # must be empty
   ```

> **Preferred zero pose (待验证):** the URDF limits make `q = 0` a valid *URDF*
> pose for all seven joints, so it is the natural calibration target. Confirm
> that it is mechanically reachable and safe for *your* arm before using it.
>
> ⚠️ **`q = 0` is not commandable.** J2 and J4 have URDF lower bound `0`, and the
> safety layer keeps `soft_limit_margin = 0.05 rad` away from every limit, so the
> *commandable* lower bound for J2/J4 is `0.05`. A command to `q = 0` is rejected
> (`cmd.out_of_limits`) even though it is inside the URDF limits. Keep any
> commanded pose at least 0.05 rad inside the limits — see `SAFETY_REVIEW.md`
> §2/§8 and `SIMULATION.md` §1.
>
> Failure mode to expect: a wrong `dir` makes a joint run away from the target.
> Fix it in the calibration file, not in code.

The file is git-ignored on purpose. Never commit a machine-specific calibration
as if it were universal.

---

## 4. Stage 2 — build `r1_robot` against the SDK

The SDK is an in-project submodule, so no path argument is needed:

```bash
cd r1_robot
git submodule update --init --recursive third_party/r1-7a_sdk   # once per clone
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

At configure time you must see the pinned SDK and its real commit:

```
-- Official R1 SDK
--   root      : /home/jesse/r1_robot/third_party/r1-7a_sdk
--   origin    : in-project submodule (default)
--   version   : 0.1.0
--   commit    : 480cfc8c031a (2026-09-28:add step file)
--   full hash : 480cfc8c031aaeccddf562b4d789c...
--   state     : clean (read-only)
...
--   unitree backend     : ON
```

* **`unitree backend : ON` is mandatory** for anything past §5. If it is `OFF`
  you passed `-DR1_ROBOT_WITH_UNITREE_BACKEND=OFF` (mock-only) and **only
  MockArm exists**.
* If configure stops with `r1_robot: the official R1 SDK is not usable`, the
  submodule is missing/empty. Run the `git submodule update --init
  --recursive` command it prints. Do **not** work around it by pointing
  `-DR1_ROBOT_SDK_ROOT` at some other checkout on the machine and then
  continuing with bring-up: confirm the pin is what you intend to fly.
* A warning that the SDK is *NOT pinned* means the checkout is not a git
  repository — only acceptable for throw-away experiments.
* A warning about *LOCAL MODIFICATIONS* means someone edited the SDK. Bring-up
  results from a modified SDK are not reproducible; restore it
  (`git -C third_party/r1-7a_sdk checkout -- .`) before continuing.

Record the `commit` value in your lab notebook: it is the motor-protocol
revision your bring-up results belong to. `r1_arm_status` prints the same pin
at runtime (`official SDK: ... commit=...`).

Verify that the build did not touch the SDK. The build itself enforces this
(`r1_sdk: checkout is clean - read-only policy honoured`), and the same check
can be run by hand:

```bash
git -C third_party/r1-7a_sdk status --porcelain      # must be empty
```

`motor_controller` is built out-of-tree into `build/lib`, deliberately not into
`third_party/r1-7a_sdk/thirdparty/motor_controller/lib` (which ships a prebuilt
`.so` that must stay untouched).

---

## 5. Stage 3 — read-only connectivity (no motion)

`r1_arm_status` never sends a motion command. `--status` only connects, reads
feedback and prints a safety report.

`connect()` refuses real hardware while `safety.allow_real_hardware` is `false`,
so opt in explicitly — and prefer a throw-away config that keeps everything else
at its default:

```bash
cd r1_robot
cat > /tmp/r1_bringup.conf <<'EOF'
backend = unitree
# Real-hardware master switch. Only for supervised bring-up.
safety.allow_real_hardware = true
# Keep the conservative defaults for everything else.
hardware.port = /dev/ttyUSB3
hardware.baudrate = 6000000
hardware.calib_path = config/joint_calib.txt
EOF

./build/r1_arm_status --config /tmp/r1_bringup.conf --status
```

Expected on a healthy arm at rest:

* `connected: ... connected=yes ... calibration=ok ... motion_enabled=no`
* `safety: PASS`
* every joint `valid=yes err=0x0`

Expected failures and their meaning:

| Symptom | Meaning | Action |
|---|---|---|
| `connect failed: refused: unusable joint calibration` | missing/incomplete `joint_calib.txt` | return to §3 |
| `connect failed: failed to open serial port` | wrong port, permissions or the adapter is busy | check §1 |
| `safety: FAIL [state.joint_invalid]` | motors have not reported feedback yet | this is **expected before §6** — the bus is silent until a packet is sent |
| `safety: FAIL [state.motor_error] ... 0x8` | transient undervoltage | check the PSU and the load; see §7 |
| `safety: FAIL [state.undervoltage]` | reported voltage below `min_supply_voltage` | 待验证: is `min_supply_voltage = 40 V` correct for this arm? If the real bus voltage is lower, fix the config — but only after measuring it. |

**Exit criteria for §5:** connection, calibration detection and the diagnostic
report all behave as understood. No motion has happened yet.

---

## 6. Stage 4 — feedback probe (this powers the motors)

Because the motors stay silent until they receive a packet, the first feedback
frame requires sending one. This is the **only** actuating action phase 1
exposes, and it must be requested twice:

```bash
./build/r1_arm_status --config /tmp/r1_bringup.conf \
    --probe --yes-i-know-this-powers-the-motors
```

What happens: one FOC (mode 1) frame with `kp = kd = tau = 0` is sent to all
seven joints.

> ⚠️ **The arm will be compliant and will NOT hold its own weight.** The probe
> does not create a hold. Gravity will take the arm. Support it first.

After the probe, re-run `--status`: `valid=yes` should now be set for all joints
and `safety: PASS` should be reachable.

The equivalent programmatic call is:

```cpp
auto* unitree = dynamic_cast<r1_robot::UnitreeArmBackend*>(backend.get());
unitree->probeFeedback(&error);
```

It is never called from `connect()`, `R1Arm::enableMotion()` or any test.

---

## 7. Stage 5 and beyond — motion (NOT available in phase 1)

Phase 1 deliberately exposes **no** way to command motion on real hardware:

* `R1Arm::home()` returns `false` with an explanatory error; the Python
  `R1Robot.home()` raises `NotImplementedError`.
* `r1_arm_status` has no `goto`/`foc`/`hold` command.
* `enableMotion()` will succeed once the state is healthy, and `sendAction()`
  will then accept commands — but **no tool in this repository calls them**, and
  you should not hand-write motion until the items below are resolved.

`enableMotion()` itself is gated on: explicit human confirmation, a connected
backend, `allow_real_hardware`, valid calibration, valid+fresh feedback for all
seven joints, no motor errors and a healthy voltage. It never turns motors on by
itself — the first frame that powers the joints will be the first command you
send.

Required before any real motion (all **待验证**):

1. Joint limits confirmed against the physical arm (URDF `effort`/`velocity` are
   datasheet-ish values, not measurements).
2. A gravity/hold strategy. `stop()` removes torque; the SDK's own examples
   admit the arm is heavy, which is why they *creep* toward a target with a
   limited lead (default 0.2 rad) instead of stepping to it.
3. `max_joint_step_rad` tuned against the real creep behaviour (0.05 rad is a
   placeholder).
4. `max_feedback_age_s` tuned against the real feedback rate (0.05 s is a
   placeholder; the SDK exposes no per-frame timestamp, so `UnitreeArmBackend`
   stamps at read time).
5. Sign of each joint under gravity.
6. A decision on `timed_out` (the SDK's `ArmJointCmd::timed_out` flag). Phase 1
   sends `false`, matching the vendor examples.
7. A tested abort path: who calls `emergencyStop()`, and how fast can power be
   cut physically?

Until then, treat the real arm as **read-only**.

---

## 8. Gripper (DM4310, CAN) — not implemented

The gripper is a **separate device on a separate bus**. It must not be assumed
reachable through the R1 serial bus. Phase 1 ships `IGripper` / `Gripper` plus
`MockGripper`; `Dm4310Gripper` fails closed (every call refuses) on purpose.

Before implementing it, these must be established (all **待验证**):

- [ ] CAN interface and adapter (`can0`? SocketCAN? vendor USB-CAN?) and bitrate.
- [ ] DM4310 CAN id(s) and master id.
- [ ] MIT-mode parameter ranges for this motor/firmware revision.
- [ ] Rack-and-pinion gear ratio → conversion from normalized opening `[0, 1]`
      to motor angle, plus the fully-closed / fully-open travel limits.
- [ ] Current and velocity limits; stall-current protection.
- [ ] Homing procedure; whether an absolute zero exists.
- [ ] A thermal model or at least a temperature cut-off.

Until then, `Dm4310Gripper` must stay a placeholder, and `MockGripper` is the
only usable implementation.

---

## 9. Fault handling

| Code | Meaning (from the SDK) | First response |
|---|---|---|
| `0x01` | Overcurrent | stop, check mechanics and the current limit |
| `0x02` / `0x04` | Transient / persistent overvoltage | check regeneration and PSU |
| `0x08` | **Transient undervoltage** | the most common failure under load — check PSU sizing and cabling, reduce the commanded torque |
| `0x10` / `0x20` / `0x40` | Chip / MOS overheat, MOS temperature abnormal | stop and let it cool |
| `0x200` | Winding overheat | stop and let it cool |
| `0x400` / `0x1000` | Encoder error (rotor / output) | do not command motion; consult the vendor |
| `0x2000` | Storage data error | re-run calibration |
| `0x4000` | Abnormal reset | investigate power integrity |
| `0x20000` | Calibration mode | exit calibration mode first |

`r1_robot` reports these as `state.motor_error` faults and blocks actuation.
Clearing them (`clearFaults()` / `--clear`) does **not** fix the root cause.

Within `r1_robot`:

| Report code | Meaning |
|---|---|
| `state.not_connected` | backend not connected |
| `state.no_timestamp` | state has no timestamp |
| `state.timestamp_in_future` | clock or backend bug |
| `state.stale_feedback` | feedback older than `max_feedback_age_s` |
| `state.non_finite` | NaN/Inf in the state |
| `state.joint_invalid` | one or more joints have no valid feedback |
| `state.motor_error` | motor error code reported |
| `state.undervoltage` | reported voltage below the floor |
| `cmd.no_calibration` | actuating command without a valid calibration |
| `cmd.non_finite` | NaN/Inf in the command |
| `cmd.gain_out_of_range` | `kp`/`kd` outside the envelope |
| `cmd.torque_out_of_range` | `|tau|` above `max_tau` |
| `cmd.velocity_out_of_range` | `|dq|` above `max_abs_velocity` |
| `cmd.out_of_limits` | target outside the soft joint limits |
| `cmd.step_too_large` | target more than `max_joint_step_rad` from the measurement |
| `cmd.compliant_foc` | *(warning)* zero gains/torque: the arm will not hold against gravity |
| `enable.no_calibration` | motion enable refused: no calibration |
| `enable.already_actuated` | *(warning)* a joint already reports FOC before enabling |

---

## 10. Abort criteria — stop the session immediately if any occur

- Any joint runs away, oscillates or makes an unexpected noise.
- An undervoltage or overcurrent fault appears more than once.
- Feedback for a joint disappears mid-session.
- The calibration is suspected to be wrong (the arm does not go where the URDF
  frame says).
- You cannot physically cut motor power within a couple of seconds.

Rollback: `Ctrl-C` in the SDK tools sends mode 0; in `r1_robot` call
`emergencyStop()` (C++) / `emergency_stop()` (Python), then cut power. Remember
that both only remove motor output.

---

## 11. Checklist to close phase 1

- [ ] §2: vendor tools work, faults clear, feedback readable.
- [ ] §3: `config/joint_calib.txt` produced and sanity-checked.
- [ ] §4: `r1_robot` configured with `unitree backend : ON`, SDK checkout clean.
- [ ] §5: `r1_arm_status --status` prints an understood report.
- [ ] §6: probe performed with the arm supported, `valid=yes` afterwards.
- [ ] §7: every item in the "required before any real motion" list resolved and
      written back into this file with measured numbers instead of placeholders.
- [ ] Every `TODO(bringup)` / **待验证** in the tree either resolved or explicitly
      deferred with a reason.
