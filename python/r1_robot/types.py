# ---------------------------------------------------------------------------
# Core value types.
#
# CONVENTIONS (identical to the C++ layer - do not diverge):
#   * joint order = URDF order (J1..J7) and SI units throughout:
#       q [rad], dq [rad/s], tau [Nm], kp [Nm/rad], kd [Nm/(rad/s)]
#   * the motor frame never leaves the C++ hardware backend
#   * timestamps are monotonic nanoseconds
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

NUM_JOINTS = 7
JOINT_NAMES: tuple[str, ...] = ("J1", "J2", "J3", "J4", "J5", "J6", "J7")

# Guard rails for every array that crosses this API.
_FLOAT_DTYPE = np.float64
_INT_DTYPE = np.uint32


def zeros_joints() -> np.ndarray:
    return np.zeros(NUM_JOINTS, dtype=_FLOAT_DTYPE)


class ControlMode(IntEnum):
    """Mirrors the mode byte of the official SDK (0/1)."""

    STOP = 0
    FOC = 1


@dataclass
class ArmCommand:
    """One full-arm command frame (URDF frame, SI units)."""

    mode: np.ndarray = field(
        default_factory=lambda: np.full(NUM_JOINTS, ControlMode.STOP, dtype=np.int64)
    )
    q: np.ndarray = field(default_factory=zeros_joints)
    dq: np.ndarray = field(default_factory=zeros_joints)
    tau: np.ndarray = field(default_factory=zeros_joints)
    kp: np.ndarray = field(default_factory=zeros_joints)
    kd: np.ndarray = field(default_factory=zeros_joints)
    timed_out: bool = False

    # -- constructors -------------------------------------------------------
    @classmethod
    def stop(cls) -> "ArmCommand":
        return cls()

    @classmethod
    def zero_torque(cls) -> "ArmCommand":
        """FOC with zero gains and zero torque (the arm goes compliant).

        This still powers the motor bridges, so it is an *actuating* command
        and stays behind the motion gate.
        """
        cmd = cls()
        cmd.mode[:] = ControlMode.FOC
        return cmd

    @classmethod
    def position(cls, q, kp: float, kd: float) -> "ArmCommand":
        cmd = cls()
        cmd.mode[:] = ControlMode.FOC
        cmd.q = np.asarray(q, dtype=_FLOAT_DTYPE).reshape(NUM_JOINTS).copy()
        cmd.kp[:] = float(kp)
        cmd.kd[:] = float(kd)
        return cmd

    # -- predicates ---------------------------------------------------------
    def is_stop(self) -> bool:
        return bool(np.all(self.mode == ControlMode.STOP))

    def has_actuation(self) -> bool:
        return bool(np.any(self.mode != ControlMode.STOP))

    def all_finite(self) -> bool:
        return all(
            bool(np.all(np.isfinite(a)))
            for a in (self.q, self.dq, self.tau, self.kp, self.kd)
        )


@dataclass
class RobotState:
    """Snapshot of the whole arm."""

    stamp_ns: int = 0
    q: np.ndarray = field(default_factory=zeros_joints)
    dq: np.ndarray = field(default_factory=zeros_joints)
    tau: np.ndarray = field(default_factory=zeros_joints)
    voltage: np.ndarray = field(default_factory=zeros_joints)
    temperature_c: np.ndarray = field(default_factory=zeros_joints)
    mode: np.ndarray = field(
        default_factory=lambda: np.full(NUM_JOINTS, ControlMode.STOP, dtype=np.int64)
    )
    error_code: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=_INT_DTYPE)
    )
    valid: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=bool)
    )
    connected: bool = False

    def all_valid(self) -> bool:
        return bool(np.all(self.valid))

    def invalid_joints(self) -> list[int]:
        return [int(i) for i in np.nonzero(~self.valid)[0]]

    def any_error(self) -> bool:
        return bool(np.any(self.error_code != 0))

    def all_finite(self) -> bool:
        return all(
            bool(np.all(np.isfinite(a))) for a in (self.q, self.dq, self.tau)
        )

    def min_supply_voltage(self) -> float:
        """Smallest *reported* voltage, or 0.0 when no backend reports one.

        0.0 means "unknown" and is deliberately skipped by the safety checks.
        """
        reported = self.voltage[self.voltage > 0.0]
        return float(reported.min()) if reported.size else 0.0
