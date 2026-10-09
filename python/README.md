# r1_robot (Python)

Python API for the Unitree R1-7A research stack.

## Scope — read this before using it

Python owns **planning, datasets, policy inference and analysis**. It does
**not** own the real-time control loop.

* The only backend implemented here is `MockArm` (offline, deterministic).
* Real hardware is reached through the C++ stack
  (`r1_robot_core` + `r1_robot_unitree_backend`), which holds the safety gates
  and the 6 Mbaud serial transport.
* The intended bridge for phase 3 is a pybind11 module exposing C++ `R1Arm`, or
  a thin ROS2 / ZMQ service in front of it. Either way the safety checks stay on
  the C++ side so Python cannot bypass them.
  `TODO(phase3)`: implement exactly one of those.
* `NullBackend` is the placeholder for "would be real hardware": it refuses
  everything.

## Install

```bash
cd python
pip install -e .          # or: pip install -e ".[dev]"
```

Requires Python ≥ 3.10 and `numpy`.

## Tests

Fully offline — no hardware, no serial port, no sleeping.

```bash
cd python
python -m pytest -q                              # if pytest is installed
python -m unittest discover -s tests -t .        # stdlib only
```

## Safety model (mirrors the C++ `R1Arm`)

1. `MockArm` is the default backend; a real backend is never selected implicitly.
2. `SafetyConfig.allow_real_hardware` defaults to `False`; while false the real
   backend refuses to connect.
3. `R1Robot.enable_motion(human_confirmed=True)` must be called explicitly.
   Nothing enables motion on connect, on error, or on state read.
4. Actuating commands are refused without a valid calibration, valid feedback for
   all seven joints, a fresh timestamp, no motor errors and a healthy voltage.
5. A **stop** command is always accepted — it can only remove energy.
6. `home()` raises `NotImplementedError` in phase 1.
7. `stop()` / `emergency_stop()` are **not** brakes: they remove motor output and
   cannot hold the arm against gravity.

The issue **codes** (`state.stale_feedback`, `cmd.step_too_large`, …) are
identical to the C++ implementation so logs can be grepped the same way.

## Quick start

```python
import numpy as np
import r1_robot as rr

robot = rr.R1Robot()                 # MockArm
robot.connect()

robot.backend.set_joint_positions([0.18] * rr.NUM_JOINTS)
robot.refresh()

print(robot.check_state())           # safety report

robot.enable_motion(human_confirmed=True)
robot.send_action(rr.ArmCommand.position(np.full(7, 0.2), kp=20.0, kd=3.0))
robot.emergency_stop()
```

### Configuration is shared with C++

`r1_robot.config` reads the same `key = value` file as the C++ side, so
`config/r1_robot.conf` is the single source of truth for the safety thresholds:

```python
import r1_robot as rr

values = rr.load_config("../config/r1_robot.conf")
safety = rr.safety_config_from(values)      # SafetyConfig
selection = rr.backend_selection_from(values)  # kind = "mock"
```

An unreadable or malformed value falls back to the conservative default — never
to a permissive one.

### Gripper

```python
import r1_robot as rr

gripper = rr.MockGripper()
gripper.connect()
gripper.write_command(
    rr.GripperCommand(mode=rr.GripperCommandMode.POSITION, position=0.7)
)

rr.Dm4310Gripper().connect()   # raises NotImplementedError (fails closed)
```

The DM4310 gripper is a separate CAN device; see `HARDWARE_BRINGUP.md` §8 for the
list of things that must be verified before it can be implemented.

## Modules

| Module | Contents |
|---|---|
| `types` | `ArmCommand`, `RobotState`, `ControlMode`, `NUM_JOINTS`, `JOINT_NAMES` |
| `limits` | `JointLimits`, `r1_default_limits()`, `check_limits()`, `clamp_to_limits()` |
| `safety` | `SafetyConfig`, `SafetyMonitor`, `SafetyReport`, `SafetyIssue` |
| `backends` | `ArmBackend`, `MockArm`, `MockArmConfig`, `NullBackend` |
| `robot` | `R1Robot`, `R1Status` |
| `gripper` | `Gripper`, `MockGripper`, `Dm4310Gripper` |
| `config` | `parse_config`, `load_config`, `safety_config_from`, `backend_selection_from` |

## Conventions

URDF joint frame (`J1..J7` order) and SI units everywhere:
`q [rad]`, `dq [rad/s]`, `tau [Nm]`, `kp [Nm/rad]`, `kd [Nm/(rad/s)]`.
The motor frame never appears in Python.
