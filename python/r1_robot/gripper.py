# ---------------------------------------------------------------------------
# Gripper abstraction.
#
# The reBot-style dual rack-and-pinion gripper uses a DAMIAO DM4310 over CAN.
# It is a SEPARATE device from the R1-7A arm and must not be assumed to be
# reachable through the Unitree serial bus.
#
# Phase 1 ships the interface + MockGripper only.  Dm4310Gripper fails closed.
#
# 待验证 (see HARDWARE_BRINGUP.md): CAN interface & bitrate, DM4310 ids, MIT
# parameter ranges, rack gear ratio, travel limits, current/velocity limits,
# homing procedure, thermal model.
# Positions are therefore a NORMALIZED opening in [0, 1].
# ---------------------------------------------------------------------------
from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field
from enum import IntEnum


class GripperCommandMode(IntEnum):
    STOP = 0
    POSITION = 1
    CURRENT = 2


@dataclass(frozen=True)
class GripperLimits:
    position_min: float = 0.0
    position_max: float = 1.0
    max_current: float = 0.0
    max_velocity: float = 0.0
    default_current_limit: float = 0.0
    source: str = ""


@dataclass
class GripperCommand:
    mode: GripperCommandMode = GripperCommandMode.STOP
    position: float = 0.0
    current: float = 0.0
    velocity_limit: float = 0.0
    current_limit: float = 0.0


@dataclass
class GripperState:
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    temperature_c: float = 0.0
    valid: bool = False
    connected: bool = False
    error_code: int = 0
    stamp_ns: int = 0


class Gripper(abc.ABC):
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

    @property
    @abc.abstractmethod
    def limits(self) -> GripperLimits: ...

    @abc.abstractmethod
    def state(self) -> GripperState: ...

    @abc.abstractmethod
    def write_command(self, cmd: GripperCommand) -> None: ...

    @abc.abstractmethod
    def stop(self) -> None: ...


def _mock_limits(limits: GripperLimits | None) -> GripperLimits:
    base = limits or GripperLimits()
    if base.source:
        return base
    return GripperLimits(
        position_min=base.position_min,
        position_max=base.position_max,
        max_current=base.max_current or 1.0,
        max_velocity=base.max_velocity or 1.0,
        default_current_limit=base.default_current_limit or 0.5,
        source="mock defaults (NOT hardware verified)",
    )


class MockGripper(Gripper):
    """Offline gripper: validates its envelope, then applies the setpoint."""

    def __init__(self, limits: GripperLimits | None = None) -> None:
        self._limits = _mock_limits(limits)
        self._state = GripperState(position=self._limits.position_min)
        self._connected = False
        self._last = GripperCommand()
        self.command_count = 0

    @property
    def name(self) -> str:
        return "mock-gripper"

    @property
    def is_real_hardware(self) -> bool:
        return False

    def connect(self) -> None:
        self._connected = True
        self._state.connected = True
        self._state.valid = True
        self._state.stamp_ns = time.monotonic_ns()

    def disconnect(self) -> None:
        self._connected = False
        self._state.connected = False
        self._state.valid = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def limits(self) -> GripperLimits:
        return self._limits

    def state(self) -> GripperState:
        return self._state

    @property
    def last_command(self) -> GripperCommand:
        return self._last

    def write_command(self, cmd: GripperCommand) -> None:
        limits = self._limits
        if not self._connected:
            raise RuntimeError("refused: gripper is not connected")

        if cmd.mode == GripperCommandMode.POSITION:
            if not (limits.position_min <= cmd.position <= limits.position_max):
                raise ValueError(
                    f"refused: position {cmd.position} outside normalized range "
                    f"[{limits.position_min}, {limits.position_max}]"
                )
        elif cmd.mode == GripperCommandMode.CURRENT:
            if abs(cmd.current) > limits.max_current:
                raise ValueError(
                    f"refused: |current| {abs(cmd.current)} A exceeds "
                    f"max_current={limits.max_current}"
                )

        if cmd.velocity_limit > 0.0 and cmd.velocity_limit > limits.max_velocity:
            raise ValueError(
                f"refused: velocity_limit {cmd.velocity_limit} exceeds "
                f"max_velocity={limits.max_velocity}"
            )
        if cmd.current_limit > 0.0 and cmd.current_limit > limits.max_current:
            raise ValueError(
                f"refused: current_limit {cmd.current_limit} exceeds "
                f"max_current={limits.max_current}"
            )

        self._last = cmd
        self.command_count += 1
        self._state.stamp_ns = time.monotonic_ns()
        if cmd.mode == GripperCommandMode.POSITION:
            self._state.position = cmd.position
            self._state.velocity = 0.0
            self._state.current = 0.0
        elif cmd.mode == GripperCommandMode.STOP:
            self._state.velocity = 0.0
            self._state.current = 0.0

    def stop(self) -> None:
        if self._connected:
            self.write_command(GripperCommand())


class Dm4310Gripper(Gripper):
    """Fail-closed placeholder - nothing here may drive hardware."""

    _MESSAGE = (
        "not implemented in phase 1. The DM4310 gripper must be brought up "
        "first - see the gripper section of HARDWARE_BRINGUP.md (CAN "
        "interface/bitrate, motor id, MIT parameter ranges, rack travel, "
        "current/velocity limits, homing)."
    )

    @property
    def name(self) -> str:
        return "dm4310-gripper"

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

    @property
    def limits(self) -> GripperLimits:
        return GripperLimits(
            source="UNVERIFIED placeholder - not usable to drive hardware"
        )

    def state(self) -> GripperState:
        return GripperState(valid=False, connected=False)

    def write_command(self, cmd: GripperCommand) -> None:
        raise NotImplementedError(self._MESSAGE)

    def stop(self) -> None:
        pass
