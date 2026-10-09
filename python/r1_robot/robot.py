# ---------------------------------------------------------------------------
# R1Robot - the high level object upper layers program against.
#
# Mirrors the C++ R1Arm gates exactly:
#   1. safety.allow_real_hardware must be true for a real backend,
#   2. enable_motion(human_confirmed=True) must be called explicitly,
#   3. every actuating command is re-validated before it is forwarded.
# A stop command is always accepted.  Nothing here enables motion implicitly
# or homes the arm.
# ---------------------------------------------------------------------------
from __future__ import annotations

import time
from dataclasses import dataclass

from .backends import ArmBackend, MockArm
from .limits import JointLimits, r1_default_limits
from .safety import SafetyConfig, SafetyMonitor, SafetyReport
from .types import ArmCommand, RobotState


@dataclass
class R1Status:
    backend_name: str
    real_hardware: bool
    connected: bool
    calibration_ok: bool
    motion_enabled: bool


class R1Robot:
    """URDF-frame, SI-unit interface to a 7-DOF R1-7A arm."""

    def __init__(
        self,
        backend: ArmBackend | None = None,
        safety: SafetyConfig | None = None,
        limits: tuple[JointLimits, ...] | None = None,
    ) -> None:
        # Default: MockArm.  Real hardware is never selected implicitly.
        self._backend: ArmBackend = backend or MockArm()
        self._safety = safety or SafetyConfig()
        self._limits = limits or r1_default_limits()
        self._monitor = SafetyMonitor(self._safety)
        self._motion_enabled = False
        self._calibration_ok = False
        self._state = RobotState()

    # -- introspection ------------------------------------------------------
    @property
    def backend(self) -> ArmBackend:
        return self._backend

    @property
    def safety(self) -> SafetyConfig:
        return self._safety

    @property
    def limits(self) -> tuple[JointLimits, ...]:
        return self._limits

    @property
    def status(self) -> R1Status:
        return R1Status(
            backend_name=self._backend.name,
            real_hardware=self._backend.is_real_hardware,
            connected=self._backend.is_connected,
            calibration_ok=self._calibration_ok,
            motion_enabled=self._motion_enabled,
        )

    @property
    def motion_enabled(self) -> bool:
        return self._motion_enabled

    def __repr__(self) -> str:
        s = self.status
        return (
            f"R1Robot(backend={s.backend_name!r}, real_hardware={s.real_hardware}, "
            f"connected={s.connected}, calibration={'ok' if s.calibration_ok else 'MISSING'}, "
            f"motion_enabled={s.motion_enabled}, "
            f"allow_real_hardware={self._safety.allow_real_hardware})"
        )

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        """Connect the backend.  Never enables motion."""
        if self._backend.is_real_hardware and not self._safety.allow_real_hardware:
            raise PermissionError(
                f"refused: backend {self._backend.name!r} drives real hardware but "
                "SafetyConfig.allow_real_hardware is False (see HARDWARE_BRINGUP.md)"
            )
        self._backend.connect()
        self._calibration_ok = self._backend.has_valid_calibration()
        if (
            self._backend.is_real_hardware
            and self._safety.require_calibration
            and not self._calibration_ok
        ):
            self._backend.disconnect()
            raise PermissionError(
                "refused: real hardware without a valid joint calibration"
            )
        self._motion_enabled = False
        self.refresh()

    def disconnect(self) -> None:
        self._motion_enabled = False
        if self._backend.is_connected:
            self._backend.stop()
            self._backend.disconnect()
        self._calibration_ok = False
        self._state = RobotState()

    @property
    def connected(self) -> bool:
        return self._backend.is_connected

    # -- state --------------------------------------------------------------
    def refresh(self) -> RobotState:
        if self._backend.is_connected:
            self._state = self._backend.read_state()
        else:
            self._state = RobotState(connected=False)
        return self._state

    @property
    def state(self) -> RobotState:
        """Last cached state.  Call refresh() to poll the backend."""
        return self._state

    def check_state(self, now_ns: int | None = None) -> SafetyReport:
        return self._monitor.check_state(
            self._state, now_ns if now_ns is not None else time.monotonic_ns()
        )

    # -- motion gating ------------------------------------------------------
    def enable_motion(self, human_confirmed: bool = False) -> None:
        """Arm the command path.  Requires an explicit human confirmation."""
        if not human_confirmed:
            raise PermissionError(
                "refused: enabling motion requires enable_motion(human_confirmed=True)"
            )
        if not self._backend.is_connected:
            raise RuntimeError("refused: not connected")
        if self._backend.is_real_hardware and not self._safety.allow_real_hardware:
            raise PermissionError(
                "refused: real hardware motion requires allow_real_hardware=True"
            )

        self.refresh()
        self._calibration_ok = self._backend.has_valid_calibration()
        report = self._monitor.check_motion_enable(
            self._state, time.monotonic_ns(), self._calibration_ok
        )
        if not report.ok():
            raise PermissionError(f"refused: motion enable failed: {report}")

        self._motion_enabled = True

    def disable_motion(self) -> None:
        self._motion_enabled = False
        if self._backend.is_connected:
            self._backend.stop()

    def emergency_stop(self) -> None:
        """Disarm and stop immediately.  Always allowed."""
        self._motion_enabled = False
        if self._backend.is_connected:
            self._backend.stop()

    def stop(self) -> None:
        if self._backend.is_connected:
            self._backend.stop()

    # -- commands -----------------------------------------------------------
    def send_action(self, cmd: ArmCommand) -> None:
        """Validate and forward one command frame.

        Stop commands are always accepted; actuating commands require
        enable_motion() and pass the full safety envelope.
        """
        if not self._backend.is_connected:
            raise RuntimeError("refused: not connected")
        if not cmd.all_finite():
            raise ValueError("refused: command contains NaN/Inf")

        if not cmd.has_actuation():
            self._backend.write_command(cmd)
            self.refresh()
            return

        if not self._motion_enabled:
            raise PermissionError(
                "refused: motion is not enabled (call enable_motion(human_confirmed=True))"
            )
        if self._backend.is_real_hardware and not self._safety.allow_real_hardware:
            raise PermissionError(
                "refused: real hardware motion requires allow_real_hardware=True"
            )

        self.refresh()
        report = self._monitor.check_command(
            cmd, self._state, time.monotonic_ns(), self._calibration_ok, self._limits
        )
        if not report.ok():
            raise ValueError(f"refused: command rejected by safety checks: {report}")

        self._backend.write_command(cmd)
        self.refresh()

    def home(self, **kwargs) -> None:
        """Blocked in phase 1 (see HARDWARE_BRINGUP.md)."""
        raise NotImplementedError(
            "blocked: homing is intentionally not exposed in phase 1. "
            "Bring up the arm with the official SDK first."
        )
