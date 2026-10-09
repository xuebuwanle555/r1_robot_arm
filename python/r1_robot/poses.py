# ---------------------------------------------------------------------------
# r1_robot/poses.py
#
# Three DIFFERENT "zero-ish" poses, deliberately defined in one place because
# conflating them is a classic source of unsafe commands:
#
#   KINEMATIC_ZERO    q = 0 in the URDF frame.  It is the reference pose for
#                     kinematics (FK/Jacobian) and the calibration target.
#                     It is NOT commandable: J2 and J4 have URDF lower bound 0
#                     and the safety layer keeps a soft margin, so the lowest
#                     commandable value for them is `soft_limit_margin`.
#
#   SIM_INITIAL_POSE  where the MuJoCo/MockArm simulation starts.  It must be
#                     commandable, comfortably inside the limits, and away from
#                     singularities so the first commands are unremarkable.
#
#   SAFE_HOME         the pose a controller may return to.  Same requirement as
#                     SIM_INITIAL_POSE, plus "gravity-friendly": close to the
#                     middle of each range so the static holding torque stays
#                     small and a failure has little energy to release.
#
# WARNING (HARDWARE_UNVERIFIED): SAFE_HOME is validated only against the URDF
# limits, the soft margins and the simulated gravity load.  It is NOT verified
# against the real arm (cable routing, self-collision, fixtures, brake
# behaviour).  Treat it as the simulation starting point and a candidate home
# pose, not as a proven-safe real-arm posture.
# ---------------------------------------------------------------------------
from __future__ import annotations

import numpy as np

from .limits import R1_MAX_TORQUE, check_limits
from .types import JOINT_NAMES, NUM_JOINTS

#: URDF-frame kinematic reference (q = 0).  FK/Jacobian reference only - NOT
#: commandable, because of the soft margin on J2/J4.
KINEMATIC_ZERO = np.zeros(NUM_JOINTS)

#: Where simulations start.  Commandable with margin to spare.
SIM_INITIAL_POSE = np.array([0.0, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0])

#: Candidate "return to" pose for a controller.  See the module warning.
SAFE_HOME = np.array([0.0, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0])

#: Poses used by examples/sim_viewer.py and by the simulation tests.  Kept here
#: so a single test can assert that every waypoint respects the soft limits.
DEMO_TRAJECTORY_POSES = [
    SIM_INITIAL_POSE,
    np.array([0.30, 0.60, 0.20, 0.50, -0.30, 0.20, 0.30]),
    np.array([-0.30, 0.40, 0.50, 0.20, 0.30, -0.20, -0.30]),
    SAFE_HOME,
]

#: How far from a limit a commanded pose should stay [rad].  Matches
#: SafetyConfig.soft_limit_margin's default so the two cannot drift apart
#: without the tests noticing.
RECOMMENDED_POSE_MARGIN = 0.05


def pose_problems(q, margin: float = RECOMMENDED_POSE_MARGIN) -> list[str]:
    """Return the reasons `q` is not a good commanded pose (empty = fine).

    Applies the same soft-margin rule as the safety layer, so a pose accepted
    here is accepted by SafetyMonitor as well.
    """
    q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
    return check_limits(q, margin=margin)


def is_commandable(q, margin: float = RECOMMENDED_POSE_MARGIN) -> bool:
    return not pose_problems(q, margin)


def pose_distance(q) -> np.ndarray:
    """Per-joint distance of `q` from the centre of its range [rad]."""
    from .limits import r1_default_limits

    limits = r1_default_limits()
    q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
    centres = np.array([0.5 * (l.lower + l.upper) for l in limits])
    return np.abs(q - centres)


def validate_trajectory_poses(poses, margin: float = RECOMMENDED_POSE_MARGIN):
    """Return a problem list for a whole waypoint sequence (empty = fine)."""
    problems: list[str] = []
    for i, q in enumerate(poses):
        for problem in pose_problems(q, margin):
            problems.append(f"waypoint {i}: {problem}")
    return problems


def min_soft_margin(q) -> float:
    """Smallest distance from any joint limit, minus the recommended margin.

    Negative means the pose is inside the soft-limits forbidden band.
    """
    from .limits import r1_default_limits

    limits = r1_default_limits()
    q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
    worst = float("inf")
    for i, limit in enumerate(limits):
        worst = min(worst, float(q[i] - limit.lower), float(limit.upper - q[i]))
    return worst - RECOMMENDED_POSE_MARGIN


def describe_pose(q, label: str = "pose") -> str:
    """Human-readable one-liner used by examples/tests/logs."""
    q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
    parts = " ".join(f"{n}={v:+.3f}" for n, v in zip(JOINT_NAMES, q))
    problems = pose_problems(q)
    status = "commandable" if not problems else f"NOT commandable: {problems}"
    return f"{label}: {parts} [{status}]"


def gravity_hold_torques(q, kinematics=None) -> np.ndarray:
    """Static joint torques needed to hold `q` (Pinocchio; simulation helper)."""
    if kinematics is None:
        from .kinematics import R1Kinematics

        kinematics = R1Kinematics()
    return kinematics.gravity_torques(q)


def static_hold_within_torque_limits(q, tol_ratio: float = 0.5) -> bool:
    """True when holding `q` against gravity needs < tol_ratio of each limit."""
    tau = np.abs(gravity_hold_torques(q))
    return bool(np.all(tau <= tol_ratio * np.asarray(R1_MAX_TORQUE)))
