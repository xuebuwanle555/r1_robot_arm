# ---------------------------------------------------------------------------
# Safety framework - fail closed.
#
# Python mirror of include/r1_robot/safety.hpp.  Issue *codes* are identical on
# purpose, so a log line from C++ and a log line from Python can be grepped the
# same way.
#
# This module only ever *rejects* commands.  It never actuates anything.
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from .limits import JointLimits, r1_default_limits
from .types import JOINT_NAMES, NUM_JOINTS, ArmCommand, ControlMode, RobotState


def lim_name(index: int) -> str:
    """Human readable joint name (J1..J7) for messages."""
    return JOINT_NAMES[index]


class SafetyLevel(IntEnum):
    OK = 0
    WARNING = 1
    FAULT = 2


_LEVEL_NAMES = {
    SafetyLevel.OK: "ok",
    SafetyLevel.WARNING: "warning",
    SafetyLevel.FAULT: "fault",
}


@dataclass(frozen=True)
class SafetyIssue:
    level: SafetyLevel
    code: str
    message: str


@dataclass
class SafetyReport:
    issues: list[SafetyIssue] = field(default_factory=list)

    def ok(self) -> bool:
        return not any(i.level is SafetyLevel.FAULT for i in self.issues)

    def has_warnings(self) -> bool:
        return any(i.level is SafetyLevel.WARNING for i in self.issues)

    def fault_codes(self) -> list[str]:
        return [i.code for i in self.issues if i.level is SafetyLevel.FAULT]

    def __str__(self) -> str:
        if not self.issues:
            return "ok"
        return "; ".join(
            f"{_LEVEL_NAMES[i.level]} [{i.code}] {i.message}" for i in self.issues
        )


@dataclass
class SafetyConfig:
    """Conservative placeholders until hardware bring-up has been completed."""

    allow_real_hardware: bool = False  # master switch: default OFF
    require_all_joints_valid: bool = True
    require_calibration: bool = True
    max_feedback_age_s: float = 0.05
    soft_limit_margin: float = 0.05
    max_joint_step_rad: float = 0.05
    max_kp: float = 100.0
    max_kd: float = 5.0
    max_tau: float = 30.0
    max_abs_velocity: float = 1.0
    min_supply_voltage: float = 40.0


class SafetyMonitor:
    """Stateless checker holding only the configuration."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()

    # -- helpers ------------------------------------------------------------
    @staticmethod
    def _fault(code: str, message: str) -> SafetyIssue:
        return SafetyIssue(SafetyLevel.FAULT, code, message)

    @staticmethod
    def _warn(code: str, message: str) -> SafetyIssue:
        return SafetyIssue(SafetyLevel.WARNING, code, message)

    # -- state --------------------------------------------------------------
    def check_state(self, state: RobotState, now_ns: int) -> SafetyReport:
        cfg = self.config
        report = SafetyReport()

        if not state.connected:
            return SafetyReport(
                [self._fault("state.not_connected", "backend is not connected")]
            )

        if state.stamp_ns == 0:
            report.issues.append(
                self._fault("state.no_timestamp", "state carries no timestamp")
            )
        elif state.stamp_ns > now_ns:
            report.issues.append(
                self._fault(
                    "state.timestamp_in_future",
                    "state timestamp is in the future (clock or backend bug)",
                )
            )
        else:
            age_s = (now_ns - state.stamp_ns) * 1e-9
            if age_s > cfg.max_feedback_age_s:
                report.issues.append(
                    self._fault(
                        "state.stale_feedback",
                        f"feedback age {age_s:.3f} s exceeds "
                        f"max_feedback_age_s={cfg.max_feedback_age_s}",
                    )
                )

        if not state.all_finite():
            report.issues.append(
                self._fault("state.non_finite", "state contains NaN/Inf (q, dq or tau)")
            )

        invalid = state.invalid_joints()
        if invalid:
            level = (
                SafetyLevel.FAULT
                if cfg.require_all_joints_valid
                else SafetyLevel.WARNING
            )
            report.issues.append(
                SafetyIssue(
                    level,
                    "state.joint_invalid",
                    f"{len(invalid)}/{NUM_JOINTS} joints have no valid feedback "
                    f"(joints: {', '.join(str(i) for i in invalid)})",
                )
            )

        for i in np.nonzero(state.error_code != 0)[0]:
            report.issues.append(
                self._fault(
                    "state.motor_error",
                    f"joint {int(i)} reports motor error "
                    f"0x{int(state.error_code[i]):x} (0x8 = transient undervoltage)",
                )
            )

        if cfg.min_supply_voltage > 0.0:
            for i in np.nonzero(state.voltage > 0.0)[0]:
                volts = float(state.voltage[i])
                if volts < cfg.min_supply_voltage:
                    report.issues.append(
                        self._fault(
                            "state.undervoltage",
                            f"joint {int(i)} supply voltage {volts:.3f} V below "
                            f"min_supply_voltage={cfg.min_supply_voltage} V",
                        )
                    )
        return report

    # -- commands -----------------------------------------------------------
    def check_command(
        self,
        cmd: ArmCommand,
        state: RobotState,
        now_ns: int,
        calibration_ok: bool,
        limits: tuple[JointLimits, ...] | None = None,
    ) -> SafetyReport:
        cfg = self.config
        limits = limits or r1_default_limits()

        if not cmd.all_finite():
            return SafetyReport(
                [self._fault("cmd.non_finite", "command contains NaN/Inf")]
            )

        # A stop command can only remove energy: always allowed.
        if not cmd.has_actuation():
            return SafetyReport()

        report = SafetyReport()
        if cfg.require_calibration and not calibration_ok:
            report.issues.append(
                self._fault(
                    "cmd.no_calibration",
                    "actuating command refused: no valid joint calibration",
                )
            )

        # The command is only as trustworthy as the state it applies to.
        report.issues.extend(self.check_state(state, now_ns).issues)

        has_output = False
        for i in range(NUM_JOINTS):
            if cmd.mode[i] == ControlMode.STOP:
                continue
            lim = limits[i]

            def fault(code: str, detail: str, joint: int = i) -> None:
                report.issues.append(
                    self._fault(code, f"joint {joint} ({lim_name(joint)}) {detail}")
                )

            if cmd.kp[i] < 0.0 or cmd.kp[i] > cfg.max_kp:
                fault(
                    "cmd.gain_out_of_range",
                    f"kp={cmd.kp[i]} outside [0, {cfg.max_kp}] Nm/rad",
                )
            if cmd.kd[i] < 0.0 or cmd.kd[i] > cfg.max_kd:
                fault(
                    "cmd.gain_out_of_range",
                    f"kd={cmd.kd[i]} outside [0, {cfg.max_kd}] Nm/(rad/s)",
                )
            if abs(cmd.tau[i]) > cfg.max_tau:
                fault(
                    "cmd.torque_out_of_range",
                    f"tau={cmd.tau[i]} Nm exceeds max_tau={cfg.max_tau}",
                )
            if abs(cmd.dq[i]) > cfg.max_abs_velocity:
                fault(
                    "cmd.velocity_out_of_range",
                    f"dq={cmd.dq[i]} rad/s exceeds "
                    f"max_abs_velocity={cfg.max_abs_velocity}",
                )

            # Position-derived checks only apply when the joint actually has
            # position authority (kp > 0); the `q` of a compliant FOC command is
            # meaningless.
            if cmd.kp[i] > 0.0:
                centre = 0.5 * (lim.lower + lim.upper)
                lo = min(lim.lower + cfg.soft_limit_margin, centre)
                hi = max(lim.upper - cfg.soft_limit_margin, centre)
                if cmd.q[i] < lo or cmd.q[i] > hi:
                    fault(
                        "cmd.out_of_limits",
                        f"q={cmd.q[i]} rad outside soft limits [{lo}, {hi}] rad",
                    )
                if state.valid[i]:
                    step = abs(cmd.q[i] - state.q[i])
                    if step > cfg.max_joint_step_rad:
                        fault(
                            "cmd.step_too_large",
                            f"q jump {step} rad from measured {state.q[i]} "
                            f"exceeds max_joint_step_rad={cfg.max_joint_step_rad}",
                        )

            if cmd.kp[i] != 0.0 or cmd.kd[i] != 0.0 or cmd.tau[i] != 0.0:
                has_output = True

        if not has_output:
            report.issues.append(
                self._warn(
                    "cmd.compliant_foc",
                    "every actuated joint has zero gains and zero torque: the arm "
                    "will be compliant and will NOT hold against gravity",
                )
            )
        return report

    # -- enable gate --------------------------------------------------------
    def check_motion_enable(
        self, state: RobotState, now_ns: int, calibration_ok: bool
    ) -> SafetyReport:
        report = self.check_state(state, now_ns)

        if self.config.require_calibration and not calibration_ok:
            report.issues.append(
                self._fault(
                    "enable.no_calibration",
                    "motion enable refused: no valid joint calibration",
                )
            )

        for i in range(NUM_JOINTS):
            if state.mode[i] != ControlMode.STOP:
                report.issues.append(
                    self._warn(
                        "enable.already_actuated",
                        f"joint {i} already reports FOC before enabling; the "
                        "enable path expects a stopped arm",
                    )
                )
                break
        return report
