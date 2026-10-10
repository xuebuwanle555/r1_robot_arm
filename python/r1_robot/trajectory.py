# ---------------------------------------------------------------------------
# r1_robot/trajectory.py
#
# Joint-space trajectories and an executor that classifies *why* a motion did
# not finish.  A controller that only knows "done / not done" cannot tell a
# benign slow approach from a stalled joint, a timeout, or a motor fault; this
# module makes those cases explicit so an upper layer can react (and so a
# simulation test can assert on them).
#
# OUTCOMES (mutually exclusive, checked in this order)
#   * REJECTED     - the safety layer refused a command (bad target/gains/state)
#   * MOTOR_FAULT  - the backend reported a non-zero motor error code
#   * TIMEOUT      - the motion did not settle inside the time budget
#   * NOT_REACHED  - time ran out but the arm settled far from the target
#   * REACHED      - the arm held *inside both* the joint-position tolerance and
#                    the joint-velocity tolerance, continuously, for settle_hold_s
#                    after the trajectory time.  Merely passing through the goal
#                    while still moving, or touching the budget while inside
#                    tolerance, is NOT enough: the timeout branch can never grant
#                    REACHED.
#
# The executor does NOT arm motion: the caller must call
# `R1Robot.enable_motion(human_confirmed=True)` first.  A trajectory never
# bypasses the safety monitor.
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .types import NUM_JOINTS, ArmCommand


class Outcome(str, Enum):
    REACHED = "reached"
    NOT_REACHED = "not_reached"
    TIMEOUT = "timeout"
    MOTOR_FAULT = "motor_fault"
    REJECTED = "rejected"


@dataclass
class ExecutionResult:
    outcome: Outcome
    elapsed_s: float
    max_error: float
    worst_joint: int
    final_error: np.ndarray
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.outcome is Outcome.REACHED


def _minimum_jerk(tau: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Position/velocity s(τ), s'(τ) for τ in [0, 1] (zero vel & accel at ends)."""
    tau = np.clip(tau, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    return s, ds


#: Exact peak of |s'| for the minimum-jerk profile above (at τ = 1/2).
MIN_JERK_PEAK_VELOCITY = 1.875
#: Exact peak of |s''| for the minimum-jerk profile (at τ = (1 ± 1/√3)/2).
MIN_JERK_PEAK_ACCELERATION = 10.0 / np.sqrt(3.0)

#: Conservative default acceleration cap used when the caller does not supply
#: one [rad/s^2].  TODO(bringup): derive the real value from the actuator and
#: power budget; the URDF has no acceleration limit.
DEFAULT_MAX_ACCELERATION = 21.0


@dataclass
class TrajectoryLimits:
    """Per-joint motion limits for a trajectory.

    ``max_velocity`` comes from the URDF (`velocity` attribute).
    ``max_acceleration`` has no URDF equivalent and is a conservative default
    until it is measured on hardware (HARDWARE_UNVERIFIED).
    """

    max_velocity: np.ndarray
    max_acceleration: np.ndarray

    @classmethod
    def from_urdf_defaults(cls, max_acceleration=None) -> "TrajectoryLimits":
        from .limits import R1_MAX_VELOCITY

        accel = (
            np.full(NUM_JOINTS, DEFAULT_MAX_ACCELERATION)
            if max_acceleration is None
            else np.asarray(max_acceleration, dtype=np.float64).reshape(NUM_JOINTS)
        )
        return cls(
            max_velocity=np.asarray(R1_MAX_VELOCITY, dtype=np.float64).copy(),
            max_acceleration=accel,
        )


def required_segment_duration(
    delta_q: np.ndarray,
    max_velocity: np.ndarray,
    max_acceleration: np.ndarray,
    safety_factor: float = 1.0,
) -> float:
    """Shortest minimum-jerk segment duration meeting every joint's limits.

    For a minimum-jerk segment of duration T covering ``delta_q``:
        peak |dq| = 1.875 * |delta_q| / T
        peak |ddq| = 10/√3 * |delta_q| / T²
    so T must satisfy both.  The slowest joint decides.
    """
    dq = np.abs(np.asarray(delta_q, dtype=np.float64))
    vmax = np.asarray(max_velocity, dtype=np.float64)
    amax = np.asarray(max_acceleration, dtype=np.float64)
    if np.any(vmax <= 0.0) or np.any(amax <= 0.0):
        raise ValueError("velocity/acceleration limits must be positive")

    t_v = float(np.max(MIN_JERK_PEAK_VELOCITY * dq / vmax))
    t_a = float(np.max(np.sqrt(MIN_JERK_PEAK_ACCELERATION * dq / amax)))
    base = max(t_v, t_a)
    return base if base <= 0.0 else base * max(1.0, safety_factor)


@dataclass
class JointTrajectory:
    """Piecewise minimum-jerk trajectory through joint-space waypoints.

    Each segment is a quintic in the normalised time so position, velocity and
    acceleration are continuous, and velocity/acceleration are zero at the
    start and end of every segment.  This is what makes the simulated arm move
    smoothly instead of stepping between setpoints.
    """

    times: list[float] = field(default_factory=list)
    positions: list[np.ndarray] = field(default_factory=list)

    def add(self, t: float, q) -> "JointTrajectory":
        q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
        if self.times and t <= self.times[-1]:
            raise ValueError("waypoint times must be strictly increasing")
        self.times.append(float(t))
        self.positions.append(q.copy())
        return self

    @classmethod
    def from_waypoints(cls, waypoints) -> "JointTrajectory":
        traj = cls()
        for t, q in waypoints:
            traj.add(t, q)
        return traj

    @property
    def duration(self) -> float:
        return self.times[-1] if self.times else 0.0

    @property
    def start(self) -> np.ndarray:
        return self.positions[0].copy()

    @property
    def goal(self) -> np.ndarray:
        return self.positions[-1].copy()

    def sample(self, t: float) -> np.ndarray:
        return self._eval(t, want_velocity=False)

    def sample_velocity(self, t: float) -> np.ndarray:
        return self._eval(t, want_velocity=True)

    def _eval(self, t: float, want_velocity: bool) -> np.ndarray:
        if not self.times:
            raise ValueError("empty trajectory")
        if len(self.times) == 1:
            return (
                np.zeros(NUM_JOINTS) if want_velocity else self.positions[0].copy()
            )
        if t <= self.times[0]:
            idx, tau = 0, 0.0
        elif t >= self.times[-1]:
            idx, tau = len(self.times) - 2, 1.0
        else:
            idx = int(np.searchsorted(self.times, t, side="right") - 1)
            tau = (t - self.times[idx]) / (self.times[idx + 1] - self.times[idx])
        q0 = self.positions[idx]
        q1 = self.positions[idx + 1]
        s, ds = _minimum_jerk(np.array([tau]))
        if not want_velocity:
            return q0 + (q1 - q0) * s[0]
        dt = self.times[idx + 1] - self.times[idx]
        return (q1 - q0) * (ds[0] / dt)

    # -- velocity / acceleration limits -------------------------------------
    def segment_durations(self) -> list[float]:
        return [
            self.times[i + 1] - self.times[i] for i in range(len(self.times) - 1)
        ]

    def time_scale_for(
        self, limits: TrajectoryLimits, safety_factor: float = 1.0
    ) -> float:
        """Smallest uniform time scale >= 1 that keeps the profile inside limits.

        Scaling every waypoint time by this factor is the *minimum* slowdown;
        the profile shape (and therefore smoothness) is unchanged.
        """
        if len(self.times) < 2:
            return 1.0
        scale = 1.0
        for i in range(len(self.times) - 1):
            duration = self.times[i + 1] - self.times[i]
            if duration <= 0.0:
                continue
            required = required_segment_duration(
                self.positions[i + 1] - self.positions[i],
                limits.max_velocity,
                limits.max_acceleration,
                safety_factor,
            )
            if required > duration:
                scale = max(scale, required / duration)
        return scale

    def scaled_to(
        self, limits: TrajectoryLimits, safety_factor: float = 1.0
    ) -> "JointTrajectory":
        """A copy of this trajectory stretched in time to respect `limits`."""
        scale = self.time_scale_for(limits, safety_factor)
        out = JointTrajectory()
        for t, q in zip(self.times, self.positions):
            out.add(t * scale, q)
        return out

    def limit_violations(
        self, limits: TrajectoryLimits, samples_per_segment: int = 400
    ) -> list[str]:
        """Dense numerical check of the profile against the limits.

        Independent of the analytic scaling: it samples the actual profile, so a
        bug in `required_segment_duration` cannot hide behind it.
        """
        vmax = np.asarray(limits.max_velocity, dtype=np.float64)
        amax = np.asarray(limits.max_acceleration, dtype=np.float64)
        problems: set[str] = set()
        for i in range(len(self.times) - 1):
            t0, t1 = self.times[i], self.times[i + 1]
            if t1 <= t0:
                continue
            h = (t1 - t0) / samples_per_segment
            for k in range(samples_per_segment + 1):
                t = t0 + k * h
                v = self.sample_velocity(t)
                t_lo = max(t - h, t0)
                t_hi = min(t + h, t1)
                if t_hi <= t_lo:
                    continue
                a = (self.sample_velocity(t_hi) - self.sample_velocity(t_lo)) / (
                    t_hi - t_lo
                )
                for j in range(NUM_JOINTS):
                    if abs(v[j]) > vmax[j] * (1.0 + 1e-6):
                        problems.add(
                            f"segment {i} joint {j}: |dq|={abs(v[j]):.4f} > "
                            f"limit {vmax[j]:.4f} rad/s"
                        )
                    if abs(a[j]) > amax[j] * (1.0 + 1e-6):
                        problems.add(
                            f"segment {i} joint {j}: |ddq|={abs(a[j]):.4f} > "
                            f"limit {amax[j]:.4f} rad/s^2"
                        )
        return sorted(problems)


@dataclass
class ExecutorConfig:
    kp: float = 60.0
    kd: float = 4.0
    #: Simulation/physics step per control cycle [s].
    dt: float = 0.002
    #: Goal tolerance per joint [rad].
    settle_tol: float = 0.02
    #: Velocity tolerance per joint [rad/s]: the arm must be essentially still.
    #: A joint crossing the goal while still moving has NOT settled.
    settle_vel_tol: float = 0.05
    #: How long the arm must stay inside both tolerances to count as settled [s].
    settle_hold_s: float = 0.1
    #: Extra time allowed *after* the trajectory end before declaring a timeout.
    extra_time_s: float = 1.0
    #: Hard cap on the whole motion [s]; None -> duration + extra_time_s.
    timeout_s: float | None = None


class TrajectoryExecutor:
    """Drives a trajectory through a robot and classifies the outcome."""

    def __init__(self, config: ExecutorConfig | None = None) -> None:
        self.config = config or ExecutorConfig()

    def run(
        self,
        robot,
        trajectory: JointTrajectory,
        tau_ff=None,
        on_step=None,
    ) -> ExecutionResult:
        """Execute ``trajectory`` on ``robot`` (motion must already be enabled).

        ``tau_ff`` is an optional callable ``q_des -> tau[7]`` used as a
        feed-forward torque.  A Pinocchio gravity compensator
        (``R1Kinematics.gravity_torques``) removes the steady-state droop;
        simulations use it, real hardware would need a validated model first.

        ``on_step`` is an optional callable ``(t, state)`` invoked once per
        control cycle, e.g. to sync a MuJoCo viewer or log a trace.  It must not
        send commands.
        """
        cfg = self.config
        backend = robot.backend
        goal = trajectory.goal
        timeout = cfg.timeout_s if cfg.timeout_s is not None else (
            trajectory.duration + cfg.extra_time_s
        )

        t = 0.0
        settled_for = 0.0
        max_error = 0.0

        def fault_result(state, when: float) -> ExecutionResult | None:
            if not state.any_error():
                return None
            err = np.abs(np.asarray(state.q) - goal)
            bad = int(np.argmax(state.error_code))
            return ExecutionResult(
                Outcome.MOTOR_FAULT, when, float(np.max(err)), bad, err,
                f"joint {bad} reported motor error "
                f"0x{int(state.error_code[bad]):x}",
            )

        while True:
            # A pre-existing motor fault is more specific than "command
            # rejected", so it is diagnosed before the safety layer sees it.
            result = fault_result(robot.refresh(), t)
            if result is not None:
                return result

            q_des = trajectory.sample(t)
            dq_des = trajectory.sample_velocity(t)
            cmd = ArmCommand.position(q_des, cfg.kp, cfg.kd)
            cmd.dq[:] = dq_des
            if tau_ff is not None:
                tau = np.asarray(tau_ff(q_des), dtype=np.float64)
                if tau.shape != (NUM_JOINTS,):
                    raise ValueError("tau_ff must return 7 values")
                cmd.tau[:] = tau

            try:
                robot.send_action(cmd)
            except (PermissionError, ValueError, RuntimeError) as exc:
                state = robot.state
                err = np.abs(np.asarray(state.q) - goal)
                return ExecutionResult(
                    Outcome.REJECTED, t, float(np.max(err)) if err.size else float("nan"),
                    int(np.argmax(err)) if err.size else -1, err,
                    f"command rejected by the safety layer: {exc}",
                )

            state = robot.refresh()
            result = fault_result(state, t)
            if result is not None:
                return result

            if on_step is not None:
                on_step(t, state)

            error = np.abs(np.asarray(state.q) - goal)
            speed = np.abs(np.asarray(state.dq))
            max_error = max(max_error, float(np.max(error)))

            # Settled only if the arm is inside BOTH the position and velocity
            # tolerance and has been there continuously for `settle_hold_s`.  A
            # joint merely passing through the goal while still moving is not
            # settled (velocity fails) and does not accumulate dwell time.
            if (
                t >= trajectory.duration
                and float(np.max(error)) <= cfg.settle_tol
                and float(np.max(speed)) <= cfg.settle_vel_tol
            ):
                settled_for += cfg.dt
                if settled_for >= cfg.settle_hold_s:
                    return ExecutionResult(
                        Outcome.REACHED, t, float(np.max(error)),
                        int(np.argmax(error)), error,
                        f"settled within tolerance (worst |dq|="
                        f"{float(np.max(speed)):.4f} rad/s)",
                    )
            else:
                settled_for = 0.0

            if t + cfg.dt > timeout:
                # The time budget expired without a sustained, stationary settle.
                # REACHED is intentionally NOT available on this branch: a timeout
                # must never substitute for the stable-time requirement.
                err = np.abs(np.asarray(state.q) - goal)
                worst = int(np.argmax(err))
                outcome = Outcome.NOT_REACHED if t >= trajectory.duration else Outcome.TIMEOUT
                return ExecutionResult(
                    outcome, t, max_error, worst, err,
                    f"goal not reached within {timeout:.3f} s "
                    f"(worst joint {worst}, error {err[worst]:.4f} rad)",
                )

            self._advance(backend, cfg.dt)
            t += cfg.dt

    @staticmethod
    def _advance(backend, dt: float) -> None:
        advance = getattr(backend, "advance", None)
        if callable(advance):
            advance(dt)


def waypoints_from_positions(
    positions, dt_per_segment: float = 1.0
) -> JointTrajectory:
    """Convenience: build a trajectory from a sequence of joint positions."""
    traj = JointTrajectory()
    for i, q in enumerate(positions):
        traj.add(i * dt_per_segment, q)
    return traj
