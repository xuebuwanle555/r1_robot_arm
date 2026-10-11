# ---------------------------------------------------------------------------
# r1_robot/gripper_ik.py
#
# Inverse kinematics for the GRIPPER tool point, not for Link7.
#
# The repo already has FK and the analytic Jacobian (`R1Kinematics`, Pinocchio
# on the official URDF) but no IK.  This module adds the missing piece, on top
# of the frozen model: Pinocchio solves the seven arm joints, and the welded
# Link7 -> `tcp_site` transform is taken from the *generated MJCF*, so the tool
# point the planner aims is the same point the physics engine reports.
#
# WHY THE OFFSET IS READ FROM XML: the adapter chain contains no movable joint,
# so Link7 -> TCP is a constant homogeneous transform.  Reading it from the
# model (instead of typing a number) keeps one source of truth; reading it from
# the XML text instead of a compiled MuJoCo model keeps this module usable
# without MuJoCo installed.  `test_gripper_ik.py` cross-checks the XML chain
# against the compiled model at two different arm configurations.
#
# The solver is damped least squares with a null-space pull toward a preferred
# posture.  It is a *geometric* solver: it knows nothing about the table, the
# block or the arm's own collision spheres.  Every returned pose therefore comes
# back with `poses.pose_problems()` and static-torque findings attached, and an
# `ok` that means "reached the target AND has no findings", never "looks fine".
# ---------------------------------------------------------------------------
from __future__ import annotations

import math

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import numpy as np

from . import poses
from .gripper_mjcf import GRIPPER_MODEL_PATH
from .kinematics import END_EFFECTOR_FRAME, R1Kinematics
from .types import NUM_JOINTS

TCP_SITE = "tcp_site"
#: Where the tool's own features sit inside `tcp_site`, MEASURED on the compiled
#: model (asserted by `test_tcp_frame_axes` in test_gripper_ik.py):
#:   the fingertip plane is at (0, 0, -0.0826), i.e. the gripper points along its
#:      own -z, so a top-down grasp wants the tcp -z axis near world (0, 0, -1);
#:   the fingers separate along -y (fingertip_left -> fingertip_right).
#: Assuming +x was the approach instead aims the tool 90 deg off: the fingertips
#: stop short of the object and the gripper body drives into the work surface.
TCP_APPROACH_AXIS = np.array([0.0, 0.0, -1.0])
#: World direction a top-down grasp approaches along.
TCP_APPROACH_DOWN = np.array([0.0, 0.0, -1.0])
TCP_OPENING_AXIS = np.array([0.0, -1.0, 0.0])


# -- small rotation helpers (stdlib only, so this module has no hard deps) ---
def quat_wxyz_to_mat(quat) -> np.ndarray:
    w, x, y, z = _vec(quat, (4,)) if quat is not None else (1.0, 0.0, 0.0, 0.0)
    n = np.hypot(np.hypot(w, x), np.hypot(y, z))
    if n == 0.0:
        raise ValueError("zero quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat_wxyz(mat) -> np.ndarray:
    """Shepperd's method; no branch singularity for valid rotation matrices."""
    m = np.asarray(mat, dtype=float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
        q = np.zeros(4)
        q[1 + i] = 0.25 * s
        q[0] = (m[k, j] - m[j, k]) / s
        q[1 + j] = (m[j, i] + m[i, j]) / s
        q[1 + k] = (m[k, i] + m[i, k]) / s
    if q[0] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def homogeneous(pos, mat) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = np.asarray(_vec(mat, (3, 3)), dtype=float).reshape(3, 3)
    t[:3, 3] = _vec(pos, (3,))
    return t


def _vec(value, shape) -> np.ndarray:
    """Numbers, or whitespace-separated XML attribute strings, or None."""
    if value is None:
        return np.zeros(shape[-1])
    if isinstance(value, str):
        return np.array([float(v) for v in value.split()], dtype=float)
    return np.asarray(value, dtype=float)


def pose_error(current, target) -> tuple[np.ndarray, np.ndarray]:
    """[dp, dtheta] in world axes: how far `current` is from `target`."""
    dp = target[:3, 3] - current[:3, 3]
    r_err = target[:3, :3] @ current[:3, :3].T
    q = mat_to_quat_wxyz(r_err)
    w = float(np.clip(q[0], -1.0, 1.0))
    v = q[1:]
    sn = np.linalg.norm(v)
    ang = 2.0 * np.arctan2(sn, w)
    axis = v / sn if sn > 1e-12 else np.array([r_err[2, 1], r_err[0, 2],
                                               r_err[1, 0]]) * 0.5
    return dp, ang * axis


def make_pose(pos, approach, opening=None) -> np.ndarray:
    """A 4x4 TCP pose whose tool axes land on the requested world directions.

    `approach` is the world direction of TCP_APPROACH_AXIS (the way the gripper
    points); `opening` is the world direction of fingertip_left -> tip_right and
    defaults to world +y.  Only the roll about the approach axis is left free, so
    an out-of-plane `opening` is projected away.
    """
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    o = np.array([0.0, 1.0, 0.0]) if opening is None else np.asarray(
        opening, dtype=float)
    o = o - np.dot(o, a) * a
    if np.linalg.norm(o) < 1e-9:
        raise ValueError("opening is parallel to approach; no frame defined")
    o = o / np.linalg.norm(o)
    # Solve R @ TCP_APPROACH_AXIS = a and R @ TCP_OPENING_AXIS = o for the frame
    # columns.  Both tool axes are single basis axes of this model's convention,
    # which is what makes this direct rather than a general least-squares fit.
    if not np.allclose(TCP_APPROACH_AXIS, [0, 0, TCP_APPROACH_AXIS[2]]):
        raise NotImplementedError("tool approach axis must be +-z")
    if not np.allclose(TCP_OPENING_AXIS, [0, TCP_OPENING_AXIS[1], 0]):
        raise NotImplementedError("tool opening axis must be +-y")
    z = -a if TCP_APPROACH_AXIS[2] < 0 else a
    y = -o if TCP_OPENING_AXIS[1] < 0 else o
    x = np.cross(y, z)
    if np.linalg.norm(x) < 1e-9:
        raise ValueError("degenerate tool frame")
    return homogeneous(pos, np.column_stack([x / np.linalg.norm(x), y, z]))


# -- the welded tool offset --------------------------------------------------
def tcp_offset_from_xml(xml_text: str, *, root_link: str = END_EFFECTOR_FRAME,
                        site: str = TCP_SITE) -> np.ndarray:
    """Constant Link7 -> tcp_site transform, by chaining the welded body frames.

    The ancestor chain is walked from the site upward, so document order does
    not matter.  Every body on the chain is checked for a movable `<joint>`: one
    would make the offset depend on that joint's value, so this raises instead of
    quietly returning a transform that is only valid at one configuration.
    """
    root = ET.fromstring(xml_text)
    parent = {child: node for node in root.iter() for child in node}

    site_el = next((e for e in root.iter("site") if e.get("name") == site), None)
    if site_el is None:
        raise RuntimeError(f"no site named {site!r} in the model")

    chain: list = []
    node = site_el
    while node is not None:
        chain.append(node)
        if node.tag == "body" and node.get("name") == root_link:
            break
        node = parent.get(node)
    if node is None or node.tag != "body":
        raise RuntimeError(f"site {site!r} is not attached under {root_link!r}")
    chain.reverse()                      # [Link7 body, ..., site]

    acc = np.eye(4)
    for elem in chain[1:]:
        if elem.tag == "body":
            for joint in elem.findall("joint"):
                if joint.get("type", "hinge") != "fixed":
                    raise RuntimeError(
                        "movable joint "
                        f"{joint.get('name')!r} on body {elem.get('name')!r} is "
                        f"on the {root_link}->{site} path: the transform is not "
                        "constant and this offset would be wrong")
        if elem.tag not in ("body", "site"):
            continue
        acc = acc @ homogeneous(elem.get("pos"),
                                quat_wxyz_to_mat(elem.get("quat")))
    return acc


def tcp_offset(model_path: Path | str | None = None) -> np.ndarray:
    return tcp_offset_from_xml(Path(model_path or GRIPPER_MODEL_PATH).read_text())


# -- forward kinematics ------------------------------------------------------
class GripperKinematics:
    """FK and the 6x7 Jacobian at `tcp_site`, from the official arm model."""

    def __init__(self, kinematics: R1Kinematics | None = None,
                 model_path: Path | str | None = None) -> None:
        self.kin = kinematics or R1Kinematics()
        self._offset = tcp_offset(model_path)
        self._offset_inv = np.linalg.inv(self._offset)

    @property
    def offset(self) -> np.ndarray:
        return self._offset.copy()

    def link7_pose(self, q) -> np.ndarray:
        return self.kin.forward_kinematics(q, END_EFFECTOR_FRAME)

    def fk(self, q) -> np.ndarray:
        """4x4 world <- tcp_site transform at arm configuration `q`."""
        return self.link7_pose(q) @ self._offset

    def tcp_position(self, q) -> np.ndarray:
        return self.fk(q)[:3, 3]

    def jacobian(self, q) -> np.ndarray:
        """6x7 Jacobian at the TCP: rows 0-2 linear (world), 3-5 angular."""
        q = self.kin._as_configuration(q)
        j7 = self.kin.jacobian(q, END_EFFECTOR_FRAME)
        r = self.tcp_position(q) - self.link7_pose(q)[:3, 3]
        hat = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]],
                        [-r[1], r[0], 0.0]])
        jac = j7.copy()
        jac[:3] = j7[:3] - hat @ j7[3:]
        return jac


# -- the solver --------------------------------------------------------------
@dataclass
class IkResult:
    q: np.ndarray
    ok: bool = False
    converged: bool = False
    position_error_m: float = float("inf")
    orientation_error_rad: float = float("inf")
    iterations: int = 0
    condition: float = float("inf")
    #: Rotation error AROUND the approach axis, and the tilt error left over.
    #: Reported separately because near the edge of this arm's workspace the roll
    #: is the component that has to be given up; the tilt is not.
    roll_error_rad: float = float("nan")
    tilt_error_rad: float = float("nan")
    #: Weighted task-error norm of the returned pose, for comparing attempts.
    task_cost: float = float("inf")
    #: Kinematic findings on the returned pose: soft limits, torque feasibility.
    #: Non-empty means `ok` is False even when the target was reached.
    problems: list[str] = field(default_factory=list)


class IKSolver:
    """Damped least squares with Levenberg-Marquardt rollback.

    Plain DLS converges quadratically near a solution but gets trapped when a
    7-joint arm with hard limits starts far away - measured here: from a nearby
    seed it lands in 3 iterations, from a distant one it stalls at 18 cm with no
    way out.  Two changes fix that, both standard:

    * a step is *accepted only if the task error decreases*; otherwise damping
      is raised and the step retried (LM),
    * the orientation constraint is ramped in from 0.1 to 1.0 over the first
      40% of the budget, so position is satisfied first on a well-conditioned
      3-constraint problem instead of fighting the wrist from far away,
    * `solve_from_seeds` runs several starts and returns the best result,
      because a single start still depends on the basin it begins in.

    Limits are enforced inside the loop (the iterate is clipped every step), so
    a solution is always inside the URDF range.  Soft margins and static torque
    feasibility are *reported* rather than silently traded away, because a pose
    that needs 90% of every torque limit is not a pose you can drive: `ok` is
    only True when the target was reached AND there are no findings.

    This is a geometric solver: it does not know about the table, the block or
    the arm's collision geoms.  Self-collision and clearance are checked by the
    simulation, never assumed here.
    """

    def __init__(self, gripper_kin: GripperKinematics | None = None, *,
                 damping: float = 0.05, max_step_rad: float = 0.12,
                 iterations: int = 1200, null_gain: float = 0.0,
                 preferred_posture: np.ndarray | None = None) -> None:
        self.gk = gripper_kin or GripperKinematics()
        self.damping = float(damping)
        self.max_step = float(max_step_rad)
        self.iterations = int(iterations)
        self.null_gain = float(null_gain)
        #: Posture the redundant DOF are pulled toward, only used when
        #: `null_gain > 0`.  Measured on this arm: with `preferred == q_seed` and
        #: gain 0.25 the null-space term drags the iterate back to where it
        #: started and the final position error goes from 1 mm to 190 mm; even
        #: pulling toward SAFE_HOME at 0.1 costs iterations.  So the posture
        #: preference is opt-in - `null_gain` defaults to 0 - and any caller who
        #: enables it must re-check convergence.
        self.preferred_posture = (poses.SAFE_HOME if preferred_posture is None
                                  else np.asarray(preferred_posture, float))
        self.lower, self.upper = self.gk.kin.configuration_limits()

    # -- internals ----------------------------------------------------------
    def _raw(self, q, target):
        """(dp, dth) unweighted, for callers that need the vectors."""
        return pose_error(self.gk.fk(q), target)

    @staticmethod
    def _split_rotation(dth, axis):
        """Split a rotation error into tilt (must hold) and roll (may be free)."""
        a = np.asarray(axis, dtype=float)
        a = a / np.linalg.norm(a)
        roll = float(np.dot(dth, a))
        return dth - roll * a, roll

    # -- internals ----------------------------------------------------------
    def _measure(self, q, target, w_rot, W) -> dict:
        """One FK call, everything the loop needs.

        The weighted cost and the weighted Jacobian rows MUST use the same
        weighting: comparing a weighted current cost against an unweighted trial
        cost makes the LM acceptance test meaningless, which is how a first
        version of this loop ended up worse than no weighting at all.
        """
        dp, dth = pose_error(self.gk.fk(q), target)
        e = np.concatenate([dp, w_rot * (W @ dth)])
        return {"e": e, "cost": float(np.linalg.norm(e)),
                "perr": float(np.linalg.norm(dp)),
                "rerr": float(np.linalg.norm(dth)), "dth": dth}

    def _weight(self, it: int) -> float:
        """Orientation priority, ramped in over the first 40% of the budget.

        Position alone is a well-conditioned 3-constraint problem on 7 DOF;
        fighting the wrist from far away is what makes plain DLS stall at 18 cm.
        """
        ramp = max(1, int(0.4 * self.iterations))
        return min(1.0, 0.1 + 0.9 * it / ramp)

    def _report(self, q, target, converged, iterations, condition,
                problems: list[str], a_world=None) -> IkResult:
        dp, dth = pose_error(self.gk.fk(q), target)
        res = IkResult(q=q.copy(), converged=converged,
                       position_error_m=float(np.linalg.norm(dp)),
                       orientation_error_rad=float(np.linalg.norm(dth)),
                       iterations=iterations, condition=condition,
                       task_cost=float(np.linalg.norm(np.concatenate([dp, dth]))))
        if a_world is not None:
            tilt, roll = self._split_rotation(dth, a_world)
            res.roll_error_rad = abs(roll)
            res.tilt_error_rad = float(np.linalg.norm(tilt))
        res.problems.extend(problems)
        res.problems.extend(poses.pose_problems(q))
        hold = self.gk.kin.gravity_torques(q)
        limit = np.asarray(self.gk.kin.facts.effort, dtype=float)
        over = [int(i) for i in np.nonzero(np.abs(hold) > 0.9 * limit)[0]]
        if over:
            res.problems.append(
                f"ik: holding q against gravity needs >90% of the torque limit "
                f"on joints {over} (HARDWARE_UNVERIFIED for a real payload)")
        res.ok = bool(converged and not res.problems)
        return res

    # -- public API ---------------------------------------------------------
    def solve(self, target: np.ndarray, q_seed, *,
              position_tol: float = 2e-4, orientation_tol: float = 5e-3,
              roll_weight: float = 1.0, preferred: np.ndarray | None = None
              ) -> IkResult:
        """Damped least squares with LM rollback and an optional free wrist roll.

        `roll_weight` < 1 de-emphasises the component of the orientation error
        about the target's approach axis.  That is not a convenience: near the
        edge of this arm's workspace a fully constrained 6-D pose can only be
        met with a joint sitting exactly ON its hard limit, which the soft-margin
        check then refuses.  Letting the roll float finds interior solutions, at
        the price of a grasp azimuth the caller has to live with - so it is
        reported (`roll_error_rad`), never hidden.
        """
        target = np.asarray(target, dtype=float)
        q = np.clip(np.asarray(q_seed, dtype=float).reshape(NUM_JOINTS),
                    self.lower, self.upper)
        pref = np.asarray(self.preferred_posture if preferred is None
                          else preferred, dtype=float).reshape(NUM_JOINTS)
        a_world = target[:3, :3] @ TCP_APPROACH_AXIS
        a_world = a_world / np.linalg.norm(a_world)
        W = np.eye(3) - (1.0 - float(roll_weight)) * np.outer(a_world, a_world)
        lam = self.damping
        problems: list[str] = []
        condition = float("inf")
        stall = 0
        it = 0

        for it in range(1, self.iterations + 1):
            w = self._weight(it)
            m = self._measure(q, target, w, W)
            rot_err = (self._split_rotation(m["dth"], a_world)[0]
                       if roll_weight != 1.0 else m["dth"])
            if (m["perr"] < position_tol
                    and np.linalg.norm(rot_err) < orientation_tol):
                return self._report(q, target, True, it, condition, problems,
                                    a_world)
            jac = self.gk.jacobian(q)
            jac[3:] = w * (W @ jac[3:])
            condition = float(np.linalg.cond(jac))
            a = jac @ jac.T + (lam ** 2) * np.eye(6)
            try:
                a_inv = np.linalg.inv(a)
            except np.linalg.LinAlgError:
                problems.append("ik: singular normal matrix")
                break
            step = jac.T @ (a_inv @ m["e"])
            nrm = np.linalg.norm(step)
            if nrm > self.max_step:
                step *= self.max_step / nrm
            if self.null_gain > 0.0:
                j_pinv = jac.T @ a_inv
                step = ((np.eye(NUM_JOINTS) - j_pinv @ jac)
                        @ (self.null_gain * (pref - q))) + step
            trial = np.clip(q + step, self.lower, self.upper)
            trial_cost = self._measure(trial, target, w, W)["cost"]
            if trial_cost < m["cost"]:
                gain, q = m["cost"] - trial_cost, trial
                lam = max(lam * 0.5, 1e-8)
            else:
                gain, lam = 0.0, min(lam * 2.0, 1e3)
            stall = stall + 1 if gain < 1e-12 else 0
            if stall >= 150:
                problems.append(
                    "ik: stalled without improving the task error; this pose is "
                    "probably outside the reachable set (or this basin needs a "
                    "different seed - see solve_from_seeds)")
                break
        return self._report(q, target, False, it, condition, problems, a_world)

    def solve_from_seeds(self, target: np.ndarray,
                         seeds: Iterable[np.ndarray] | None = None, *,
                         position_tol: float = 2e-4,
                         orientation_tol: float = 5e-3,
                         roll_weight: float = 1.0,
                         preferred: np.ndarray | None = None) -> IkResult:
        """Best solution over a set of starts (a deterministic bank by default).

        Ranking is deliberate: a converged, finding-free solution wins; among
        failures the smallest weighted task cost is reported, so the caller sees
        how close the arm can get rather than an arbitrary basin.
        """
        best: IkResult | None = None
        for seed in (default_seeds() if seeds is None else seeds):
            res = self.solve(target, seed, position_tol=position_tol,
                             orientation_tol=orientation_tol,
                             roll_weight=roll_weight, preferred=preferred)
            if best is None or result_key(res) > result_key(best):
                best = res
            if best.ok:
                # Nothing can rank higher: reached, no soft-limit or static
                # torque findings.  Returning here is what makes a warm-started
                # plan (previous solution first in `seeds`) cost one iteration
                # set instead of the whole bank.
                return best
        if best is None:
            raise ValueError("no seeds given")
        return best


def result_key(res: IkResult) -> tuple:
    """Ranking: clean solution, then reached, then posture margin, then cost.

    Posture margin comes before cost so a solution that sits 1 rad from every
    limit never outranks a comfortable one that is 1 mm further from the target.
    """
    return (res.ok, res.converged, round(poses.min_soft_margin(res.q), 4),
            -res.task_cost)


#: Deterministic starts for multi-seed IK: the posture the controller holds
#: between tasks, the two mirrored shoulder directions the tool needs to aim at
#: a workspace point anywhere on the table, and a fixed pseudo-random spread
#: inside the soft limits.  Fixed RNG, so a run reproduces.
IK_SEED_VARIANTS = [
    np.array([1.57, 0.6, 0.9, 0.6, 0.6, 0.0, 0.0]),
    np.array([-1.57, 0.6, 0.9, 0.6, 0.6, 0.0, 0.0]),
    np.array([1.57, 1.2, 1.2, 0.6, 0.9, 0.0, 0.0]),
    np.array([-1.57, 1.2, 1.2, 0.6, 0.9, 0.0, 0.0]),
]


def default_seeds(count: int = 12) -> list[np.ndarray]:
    from .limits import r1_default_limits

    bounds = np.array([[l.lower + poses.RECOMMENDED_POSE_MARGIN,
                        l.upper - poses.RECOMMENDED_POSE_MARGIN]
                       for l in r1_default_limits()])
    seeds = [poses.SAFE_HOME.astype(float).copy()]
    seeds += [v.astype(float).copy() for v in IK_SEED_VARIANTS]
    rng = np.random.default_rng(0)
    while len(seeds) < max(1, int(count)):
        seeds.append(rng.uniform(bounds[:, 0], bounds[:, 1], NUM_JOINTS))
    return seeds[:max(1, int(count))]


#: Coarse wrist-roll candidates for the search below, in degrees of opening
#: azimuth about the world z axis.  Deliberately coarse: each extra roll costs a
#: full multi-seed solve, and the feasible band for this wrist is wide (measured
#: ~80 deg at the table positions used by the pick-and-place task).
ROLL_CANDIDATES_DEG: tuple[float, ...] = (0.0, 45.0, 90.0, 135.0, 180.0,
                                          225.0, 270.0, 315.0)


def solve_with_roll_search(solver: IKSolver, position, approach=None, *,
                          rolls_deg=ROLL_CANDIDATES_DEG,
                          seeds: list[np.ndarray] | None = None,
                          **kwargs) -> IkResult:
    """Solve `position` + `approach` with the wrist roll left free.

    The R1 wrist cannot roll arbitrarily: J6 spans +-0.524 rad and J7 +-1.571
    rad, so at a table point only a limited band of grasp azimuths is reachable.
    For a symmetric object that band is usually enough - which is exactly why it
    has to be *searched* rather than assumed.  The first `ok` solution wins;
    otherwise the least-bad attempt is returned with its problems attached.
    """
    approach = TCP_APPROACH_DOWN if approach is None else np.asarray(
        approach, dtype=float)
    seeds = default_seeds() if seeds is None else seeds
    best: IkResult | None = None
    for deg in rolls_deg:
        a = math.radians(float(deg))
        try:
            tgt = make_pose(position, approach, (math.cos(a), math.sin(a), 0.0))
        except ValueError:
            continue     # that roll is parallel to the approach: not a pose
        res = solver.solve_from_seeds(tgt, seeds, **kwargs)
        if best is None or result_key(res) > result_key(best):
            best = res
        if best.ok:
            return best
    assert best is not None
    return best


class WorkspaceSampler:
    """FK samples of the in-limit workspace, for seeding IK and proving reach.

    DLS IK is a local method: on this arm a guessed seed reaches a down-and-out
    pose about as often as it pins J2 against its zero lower bound.  Sampling the
    forward map instead answers two questions with data: does a pose exist in the
    reachable set at all, and which configuration is the seed that finds it.
    Stored per sample: the TCP, the world approach axis (tool -z) and the world
    jaw axis (tool -y, fingertip_left -> tip_right), plus the fingertip-plane
    centre, which is what a grasp actually has to be on the object.
    """

    def __init__(self, gripper_kin: GripperKinematics | None = None, *,
                 samples: int = 60000, seed: int = 0,
                 margin: float = 0.06, tip_offset_m: float = 0.0826) -> None:
        from .limits import r1_default_limits

        self.gk = gripper_kin or GripperKinematics()
        self.tip_offset_m = float(tip_offset_m)
        bounds = np.array([[l.lower + margin, l.upper - margin]
                           for l in r1_default_limits()])
        rng = np.random.default_rng(seed)
        Q = rng.uniform(bounds[:, 0], bounds[:, 1], size=(int(samples), NUM_JOINTS))
        self.q = Q
        self.tcp = np.empty((len(Q), 3))
        self.approach = np.empty((len(Q), 3))
        self.jaw = np.empty((len(Q), 3))
        for i, qq in enumerate(Q):
            T = self.gk.fk(qq)
            self.tcp[i] = T[:3, 3]
            self.approach[i] = -T[:3, 2]     # tool -z, per TCP_APPROACH_AXIS
            self.jaw[i] = -T[:3, 1]          # tool -y, per TCP_OPENING_AXIS
        self.tip = self.tcp + self.tip_offset_m * self.approach

    def seeds_for(self, tip_position, *, jaw_axis=None, approach_axis=None,
                  count: int = 4, tip_tol_m: float = 0.05) -> list[np.ndarray]:
        """Configurations whose fingertip plane is near `tip_position`.

        `jaw_axis` / `approach_axis`, when given, are preferred rather than
        required: the returned order puts the best-matching samples first, so the
        caller's IK can still refine a near miss instead of failing on an exact
        filter.  Nothing is returned when the region is simply not reachable.
        """
        p = np.asarray(tip_position, dtype=float)
        d = np.linalg.norm(self.tip - p, axis=1)
        keep = np.nonzero(d <= tip_tol_m)[0]
        if not len(keep):
            return []
        score = d[keep].copy()
        if jaw_axis is not None:
            ja = np.asarray(jaw_axis, dtype=float)
            score += 0.05 * (1.0 - np.abs(self.jaw[keep] @ ja / np.maximum(
                np.linalg.norm(self.jaw[keep], axis=1), 1e-9)))
        if approach_axis is not None:
            aa = np.asarray(approach_axis, dtype=float)
            score += 0.05 * (1.0 - np.abs(self.approach[keep] @ aa / np.maximum(
                np.linalg.norm(self.approach[keep], axis=1), 1e-9)))
        order = keep[np.argsort(score)][:int(count)]
        return [self.q[i].copy() for i in order]

    def reachable(self, tip_position, *, tip_tol_m: float = 0.02) -> int:
        """How many samples put the fingertip plane within `tip_tol_m`."""
        return int(np.sum(np.linalg.norm(
            self.tip - np.asarray(tip_position, dtype=float), axis=1)
            <= tip_tol_m))

    def grasp_candidates(self, tip_position, *, count: int = 6,
                         min_down_cos: float = 0.85,
                         max_jaw_vertical: float = 0.26,
                         min_face_alignment: float = 0.98,
                         tip_tol_m: float = 0.06):
        """Sampled poses near `tip_position`, as (q, approach, jaw, distance).

        The tool orientation comes from the reachable set instead of being asked
        for.  That distinction is measured, not stylistic: demanding a vertical
        approach at a point whose best in-limit sample tilts 35 deg away makes DLS
        trade position for orientation, and it was observed leaving the TCP
        176 mm from a target the forward map says is 9 mm away.  Seeded with the
        pose the workspace actually contains, the same solver converges to a few
        millimetres.

        The three filters are what makes a sampled pose a *parallel-jaw* grasp of
        an axis-aligned object standing on a horizontal surface, rather than some
        orientation that happens to pass by it:

        * `min_down_cos` keeps the tool pointing at the table;
        * `max_jaw_vertical` (= sin 15 deg) keeps the pad-separating axis in the
          horizontal plane.  Without it the sampler returned a candidate whose jaws
          were canted 20 deg, which met the top corner of a 120 mm bar and drove it
          40 mm into the work surface instead of straddling it;
        * `min_face_alignment` keeps the jaw axis within about 10 deg of one of the
          object's face normals (the objects here are axis-aligned, so that is
          max(|jaw.x|, |jaw.y|)).  A 40 mm square gripped 29 deg off its faces
          presents 40(cos + sin) = 54 mm between opposing corners, more than the
          52 mm the jaws open to, and the pads then slide past the object with zero
          contact force - measured, not hypothesised.

        Ordering is by tip distance.
        """
        p = np.asarray(tip_position, dtype=float)
        d = np.linalg.norm(self.tip - p, axis=1)
        # The approach axis points from the tool mount TOWARD the pads (checked
        # against the compiled model in `tests/test_gripper_ik.py`), so a tool
        # reaching down over an object has `approach[2] < 0`.  Getting this sign
        # backwards silently selects the poses whose pads point up at the sky -
        # which still reach the target position, still report no penetration once
        # the object is excluded from the check, and then cannot grip anything: it
        # was the reason a whole trial sweep closed at 0.0-0.2 N.
        down = -self.approach[:, 2]
        vertical_jaw = np.abs(self.jaw[:, 2])
        face_align = np.maximum(np.abs(self.jaw[:, 0]), np.abs(self.jaw[:, 1]))
        keep = np.nonzero((d <= tip_tol_m) & (down >= min_down_cos)
                          & (vertical_jaw <= max_jaw_vertical)
                          & (face_align >= min_face_alignment))[0]
        if not len(keep):
            return []
        order = keep[np.argsort(d[keep])][:int(count)]
        return [(self.q[i].copy(), self.approach[i].copy(), self.jaw[i].copy(),
                 float(d[i])) for i in order]
