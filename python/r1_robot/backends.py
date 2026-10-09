# ---------------------------------------------------------------------------
# Backend abstraction.
#
# PHASE 1 SCOPE
#   Python owns *planning, datasets and policy inference* - not the real-time
#   control loop.  The only backend implemented here is MockArm, so that
#   pipelines (LeRobot / ACT / Diffusion Policy / VLA) can be developed and
#   tested on a workstation with no arm attached.
#
#   Real hardware is reached through the C++ stack (r1_robot_core /
#   r1_robot_unitree_backend), which owns the safety gates and the 6 Mbaud
#   serial bus.  The intended phase-3 bridge is a pybind11 module exposing
#   R1Arm, or a thin ROS2/ZMQ service in front of it.
#   TODO(phase3): pick and implement exactly one of those, and keep the safety
#   checks on the C++ side - Python must not be able to bypass them.
# ---------------------------------------------------------------------------
from __future__ import annotations

import abc
from dataclasses import dataclass

import numpy as np

from .types import NUM_JOINTS, ArmCommand, ControlMode, RobotState


class ArmBackend(abc.ABC):
    """Thin I/O adapter.  Validating commands is the caller's job (R1Robot)."""

    @property
    @abc.abstractmethod
    def name(self) -> str: ...

    @property
    @abc.abstractmethod
    def is_real_hardware(self) -> bool: ...

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool: ...

    @abc.abstractmethod
    def has_valid_calibration(self) -> bool: ...

    @abc.abstractmethod
    def read_state(self) -> RobotState: ...

    @abc.abstractmethod
    def write_command(self, cmd: ArmCommand) -> None: ...

    @abc.abstractmethod
    def stop(self) -> None: ...


@dataclass
class MockArmConfig:
    calibration_ok: bool = True
    supply_voltage: float = 48.0
    gravity_torque: float = 0.0
    inertia: float = 0.5
    damping: float = 0.5
    max_dq: float = 2.0
    time_step_s: float = 0.02
    feedback_delay_s: float = 0.0
    error_code: int = 0


class MockArm(ArmBackend):
    """Deterministic offline backend.

    Time only advances when advance() is called, so tests never sleep and never
    depend on wall-clock timing.
    """

    def __init__(self, config: MockArmConfig | None = None) -> None:
        self.config = config or MockArmConfig()
        self._connected = False
        self._time_ns = 0
        self._q = np.zeros(NUM_JOINTS)
        self._dq = np.zeros(NUM_JOINTS)
        self._tau = np.zeros(NUM_JOINTS)
        self._cmd = ArmCommand.stop()
        self._valid = np.zeros(NUM_JOINTS, dtype=bool)
        self._error = np.zeros(NUM_JOINTS, dtype=np.uint32)
        self.command_count = 0

    # -- backend interface --------------------------------------------------
    @property
    def name(self) -> str:
        return "mock"

    @property
    def is_real_hardware(self) -> bool:
        return False

    def connect(self) -> None:
        self._connected = True
        self._time_ns = 0
        self.command_count = 0
        self._cmd = ArmCommand.stop()
        self._valid[:] = True
        self._error[:] = 0

    def disconnect(self) -> None:
        self._connected = False
        self._valid[:] = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def has_valid_calibration(self) -> bool:
        return bool(self.config.calibration_ok)

    def read_state(self) -> RobotState:
        import time

        now = time.monotonic_ns()
        delay_ns = int(max(0.0, self.config.feedback_delay_s) * 1e9)
        stamp = now - delay_ns if now > delay_ns else 0

        state = RobotState(connected=self._connected, stamp_ns=stamp)
        state.q = self._q.copy()
        state.dq = self._dq.copy()
        state.tau = self._tau.copy()
        state.voltage[:] = self.config.supply_voltage
        state.temperature_c[:] = 30.0
        state.mode = self._cmd.mode.copy()
        state.error_code = self._error.copy()
        state.valid = self._valid & self._connected
        return state

    def write_command(self, cmd: ArmCommand) -> None:
        if not self._connected:
            raise RuntimeError("MockArm: not connected")
        self._cmd = cmd
        self.command_count += 1

    def stop(self) -> None:
        if self._connected:
            self.write_command(ArmCommand.stop())

    # -- deterministic test helpers ----------------------------------------
    @property
    def simulated_time_ns(self) -> int:
        return self._time_ns

    @property
    def last_command(self) -> ArmCommand:
        return self._cmd

    def set_joint_positions(self, q) -> None:
        self._q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS).copy()

    def set_joint_velocities(self, dq) -> None:
        self._dq = np.asarray(dq, dtype=np.float64).reshape(NUM_JOINTS).copy()

    def set_feedback_valid(self, joint: int, valid: bool) -> None:
        self._valid[joint] = valid

    def set_feedback_valid_all(self, valid: bool) -> None:
        self._valid[:] = valid

    def inject_error(self, joint: int, code: int) -> None:
        self._error[joint] = code

    def clear_errors(self) -> None:
        self._error[:] = 0

    def advance(self, dt_s: float) -> None:
        if dt_s <= 0.0 or not self._connected:
            return
        step = self.config.time_step_s
        substeps = max(1, int(np.ceil(dt_s / step)))
        h = dt_s / substeps
        for _ in range(substeps):
            self._integrate(h)
        self._time_ns += int(dt_s * 1e9)

    # -- model --------------------------------------------------------------
    def _integrate(self, h: float) -> None:
        from .limits import r1_default_limits

        limits = r1_default_limits()
        for i in range(NUM_JOINTS):
            c_mode = self._cmd.mode[i]
            tau = -self.config.damping * self._dq[i] - self.config.gravity_torque
            if c_mode == ControlMode.FOC:
                tau += (
                    self._cmd.kp[i] * (self._cmd.q[i] - self._q[i])
                    + self._cmd.kd[i] * (self._cmd.dq[i] - self._dq[i])
                    + self._cmd.tau[i]
                )
            ddq = tau / self.config.inertia
            dq = float(np.clip(self._dq[i] + ddq * h, -self.config.max_dq, self.config.max_dq))
            self._dq[i] = dq
            q = self._q[i] + dq * h
            lim = limits[i]
            if q < lim.lower:
                q, self._dq[i] = lim.lower, 0.0
            elif q > lim.upper:
                q, self._dq[i] = lim.upper, 0.0
            self._q[i] = q
            self._tau[i] = tau


class NullBackend(ArmBackend):
    """Placeholder for the (not yet implemented) real-hardware bridge.

    Every method refuses, so nothing can be silently driven.
    """

    _MESSAGE = (
        "the real-hardware backend is not implemented in Python (phase 1). "
        "Use the C++ stack (r1_robot_unitree_backend) or the phase-3 bridge."
    )

    @property
    def name(self) -> str:
        return "null"

    @property
    def is_real_hardware(self) -> bool:
        return True

    def connect(self) -> None:
        raise NotImplementedError(self._MESSAGE)

    def disconnect(self) -> None:
        pass

    @property
    def is_connected(self) -> bool:
        return False

    def has_valid_calibration(self) -> bool:
        return False

    def read_state(self) -> RobotState:
        return RobotState(connected=False)

    def write_command(self, cmd: ArmCommand) -> None:
        raise NotImplementedError(self._MESSAGE)

    def stop(self) -> None:
        pass
