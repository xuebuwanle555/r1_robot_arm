# SIMULATION.md — MuJoCo + Pinocchio foundation

Phase-2 simulation ground work for `r1_robot`: kinematics (Pinocchio), physics
(MuJoCo), a physics backend (`SimArm`) that reuses the phase-1 `ArmBackend`
interface, joint-space trajectories with explicit outcome classification, and a
headless-safe viewer example.

Everything here runs **offline, without a GPU and without hardware**. The
official URDF and meshes are only ever **read**.

---

## 1. Official model and meshes

| Item | Path |
|------|------|
| URDF | `third_party/r1-7a_sdk/urdf/R1_Robotic_Arm_V1.1-0915/R1_Robotic_Arm_V1.1_0915/R1_Robotic_Arm/urdf/R1_Robotic_Arm.urdf` |
| Meshes | `.../R1_Robotic_Arm/meshes/` (8 STL, base_link + Link1..Link7) |
| SDK pin | commit `480cfc8c031a` (`480cfc8c031aaeccddf562b4d789d46b64c6c543`), version 0.1.0 |

The Python modules locate the SDK from the repository root
(`python/r1_robot/kinematics.py`); a missing submodule raises a clear
`FileNotFoundError` with the `git submodule update --init` command.

### URDF facts (read from the XML directly)

* 7 **revolute** joints `J1..J7`, fixed base, joint `k` drives `Link_k`.

| Joint | axis (URDF) | lower | upper | effort [Nm] | velocity [rad/s] |
|-------|-------------|-------|-------|-------------|------------------|
| J1 | `0 0 -1` | −2.723 | 2.723 | 60 | 18.7 |
| J2 | `1 0 0`  | 0       | 3.1416 | 60 | 18.7 |
| J3 | `0 1 0`  | −0.349  | 3.491 | 33 | 33.4 |
| J4 | `-1 0 0` | 0       | 3.1415 | 33 | 33.4 |
| J5 | `0 1 0`  | −2.269  | 2.269 | 33 | 33.4 |
| J6 | `-1 0 0` | −0.524  | 0.524 | 10 | 37.7 |
| J7 | `0 0 -1` | −1.571  | 1.571 | 10 | 37.7 |

These match `limits.py` / `limits.cpp` exactly (asserted by a test).

* Link masses `[0.8945, 0.7477, 0.7184, 0.8557, 0.5929, 0.391, 0.1048]` kg
  (arm total **4.3052 kg**; `base_link` 1.7759 kg).
* Inertias are full 3×3 tensors (not diagonal); a test asserts each is
  symmetric positive definite.
* 8 collision meshes and 8 visual meshes.

> **Finding (soft-limit zero pose).** J2 and J4 have URDF lower bound `0`.
> With the default `soft_limit_margin = 0.05 rad` the *commandable* lower bound
> is therefore `0.05`, so a command to `q = 0` is **rejected** by the safety
> layer even though it is inside the URDF limits. The "preferred zero pose" note
> in `HARDWARE_BRINGUP.md` is corrected accordingly: keep J2/J4 ≥ 0.05 rad in
> any commanded pose. `examples/sim_viewer.py` checks this before moving.

---

## 2. Pinocchio kinematics (`python/r1_robot/kinematics.py`)

`R1Kinematics` wraps a fixed-base Pinocchio model of the official URDF
(`nq = nv = 7`).

| API | Meaning |
|-----|---------|
| `R1Kinematics()` | build the model from `official_urdf_path()` |
| `forward_kinematics(q, frame)` | 4×4 pose of `frame` (`Link7` by default) |
| `position(q, frame)` | 3-vector position |
| `jacobian(q, frame)` | 6×7 geometric Jacobian, `LOCAL_WORLD_ALIGNED` |
| `numerical_jacobian(q, frame)` | finite-difference Jacobian (test oracle) |
| `gravity_torques(q)` | joint torques to hold the arm (`computeGeneralizedGravity`) |
| `configuration_limits()` | lower/upper position limits |
| `urdf_facts()` | joints, axes, limits, masses, inertias, meshes, collisions |

### Numerical consistency

`tests/test_kinematics.py` compares the analytic Jacobian against a
finite-difference of FK over 5 random configurations; agreement is required to
`atol = 1e-5`. FK determinism, homogeneous-transform structure and orthonormal
rotation are also asserted.

---

## 3. MuJoCo model and `SimArm` (`python/r1_robot/sim.py`)

### Model generation (no drift, no edits to the official model)

`generate_mjcf()` reads the official URDF and emits MJCF:
bodies with URDF joint origins (as `pos`/`quat`), full inertia tensors rotated
into the body frame (`I_b = R · I_urdf · Rᵀ`, since MuJoCo's `fullinertia`
cannot be combined with `quat`), mesh geoms for visual + collision, and 7
`<general>` actuators named `Jk_ctrl`.

`write_sim_model()` writes a copy to `sim/models/r1_7a_sim.xml` with a
**relative** `meshdir` pointing at the SDK meshes (read-only, no duplication).
Regenerate with:

```bash
python -c "import r1_robot as rr; print(rr.write_sim_model())"
```

Resulting model: `nq = nv = nu = 7`, `nbody = 9` (world + base_link +
Link1..7), `ngeom = 17` (8 visual + 8 collision + floor).

### Cross-validation

`SimArm.verify_against_model()` (run on `connect()`) compares MuJoCo body FK for
`Link7` against Pinocchio over a non-trivial pose (position and rotation,
`atol = 1e-6`), so a model-generation regression fails immediately.
`tests/test_sim.py` repeats this over 5 random configurations, and checks joint
ranges and body masses against the URDF.

### Control law

Each joint actuator computes
`f = kp·(q_des − q) + kd·(dq_des − q̇) + tau_ff`
by setting `biasprm[1] = −kp`, `biasprm[2] = −kd` and
`ctrl = kp·q_des + kd·dq_des + tau_ff` — the same impedance law `MockArm`
integrates. A mode-0 (stop) command sets the gains and `ctrl` to zero, leaving
the joint free (motor output disabled), and the arm falls under gravity.

`SimArm` implements the phase-1 Python `ArmBackend` interface (`name`,
`is_real_hardware`, `connect`, `read_state`, `write_command`, `stop`), plus a
deterministic `advance(dt)` for headless tests, and reports
`is_real_hardware == False`.

---

## 4. Trajectories and outcome classification (`python/r1_robot/trajectory.py`)

`JointTrajectory` interpolates waypoints with **minimum-jerk** (quintic)
segments — zero velocity and acceleration at every segment boundary, which is
what makes the simulated arm move smoothly.

`TrajectoryExecutor.run(robot, traj, tau_ff=..., on_step=...)` drives the
trajectory through `R1Robot` (so every command passes the safety monitor) and
returns an `ExecutionResult` classified as:

| Outcome | Meaning |
|---------|---------|
| `REACHED` | every joint settled inside tolerance and held it |
| `NOT_REACHED` | settled, but outside tolerance (e.g. PD droop under load) |
| `TIMEOUT` | time budget expired before the trajectory finished |
| `MOTOR_FAULT` | a joint reported a non-zero motor error code |
| `REJECTED` | the safety layer refused a command (bad target/gains/state) |

`tau_ff` is an optional feed-forward hook; the simulation passes
`R1Kinematics.gravity_torques`, which removes the steady-state PD droop
(0 → ~0.03–0.045 rad) and yields millimetre-level position accuracy.

---

## 5. Viewer

`examples/sim_viewer.py` opens the MuJoCo viewer and runs a pre-defined safe
trajectory (waypoints validated against the soft limits before moving):

```bash
# interactive viewer (needs a display)
python examples/sim_viewer.py

# headless: same trajectory, prints result + end-effector pose
python examples/sim_viewer.py --headless

# behaviour
python examples/sim_viewer.py --no-hold   # exit right after the move
python examples/sim_viewer.py --loop      # repeat until the window is closed

# tuning
python examples/sim_viewer.py --headless --duration 1.0 --kp 80 --kd 5 --no-gravity
```

* **Interactive default (`--hold`)**: the window opens, the trajectory runs
  once, then the window **stays open** on the final pose so the model can be
  inspected; close it (or Ctrl-C) to quit. `--no-hold` exits immediately after
  the move, `--loop` repeats it.
* **Headless**: runs the same trajectory, prints the result, exits.
* The script only ever constructs a `SimArm`; it cannot drive real hardware.

> **Known GL teardown issue.** On some GL drivers (e.g. software rendering
> under Xvfb) MuJoCo's GLFW passive viewer segfaults in its *global
> destructor* at interpreter shutdown — after the script has already printed
> its result. `examples/sim_viewer.py` therefore flushes and calls
> `os._exit()` in its entry point to skip that teardown. If you hit the same
> crash in your own viewer code, that is the workaround (or try
> `MUJOCO_GL=egl` / `osmesa` where available).

Typical headless output:

```
backend        : sim (real_hardware=False)
trajectory     : 4 waypoints, 1.50s per segment, kp=80.0, kd=5.0
feed-forward   : Pinocchio gravity compensation
result         : reached (elapsed 4.60s, final error 0.0000 rad)
end-effector   : [0.0455, -0.0439, 0.2419] m
```

---

## 6. How to run the tests

```bash
# C++ (74 tests)
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

# Python (64 tests; MuJoCo/Pinocchio tests skip if the extras are missing)
cd python && python -m pytest -q
```

Simulation extras:

```bash
python -m pip install --user "mujoco>=3.0" "pin>=2.6"
# or, from python/:  pip install -e ".[sim]"
```

---

## 7. Scope / next steps

Done in this round: URDF + model inspection, Pinocchio FK/Jacobian with
numerical validation, MuJoCo model generation + cross-validation, `SimArm`,
minimum-jerk trajectories with outcome classification, headless viewer.

Not done (deliberately): no ROS2 / MoveIt2 / LeRobot, no VLA, no real-arm
motion, no contact-rich manipulation, no model-based controller beyond the
gravity feed-forward. Suggested phase-2 work:

1. **Task-space servo**: use `R1Kinematics.jacobian` for Cartesian impedance /
   resolved-rate control, with the same safety envelope.
2. **Dynamics**: validate `gravity_torques` against MuJoCo `qfrc_bias`, then add
   Coriolis + inertia-based computed-torque control.
3. **Contact**: enable collision geoms, add the gripper, and validate grasp
   models against the DM4310 gripper limits.
4. **Trajectory polish**: time-scaling to satisfy `max_velocity`/`max_accel`
   from the URDF, and Cartesian waypoints interpolated in task space.
5. **Hardware parity**: replay recorded SimArm poses through the C++ stack and
   compare against `MockArm` for regression coverage.
