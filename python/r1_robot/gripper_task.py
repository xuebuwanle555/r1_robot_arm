# ---------------------------------------------------------------------------
# r1_robot/gripper_task.py
#
# The rig: a MuJoCo gripper scene driven through the FROZEN safety stack.
#
# Every moving part of an experiment goes through the same doors a real arm
# would use - `R1Robot.send_action()` (which runs `SafetyMonitor.check_command`
# and needs `enable_motion(human_confirmed=True)`) and `Gripper.write_command()`
# (which validates the normalised envelope).  `SceneRig.place_arm_for_setup()`
# is the one teleport left in the file, it is named as setup, and nothing in the
# pick-and-place sequence calls it: motion in a trial is produced by commands.
#
# Gains and rates are chosen INSIDE SafetyConfig, not beside it: the default
# envelope allows kp <= 100, kd <= 5, |tau| <= 30 N m and 0.05 rad of setpoint
# travel per command, so the rig uses 80 / 4 / 24 and 0.03 rad.  The consequence
# is honest and measurable - a steady-state droop under gravity, logged rather
# than hidden by raising the gains behind the monitor's back.
# ---------------------------------------------------------------------------
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

from .gripper import GripperCommand, GripperCommandMode
from . import gripper_ik
from .gripper_ik import (GripperKinematics, IKSolver, IkResult, WorkspaceSampler,
                         default_seeds, make_pose)
from .gripper_mjcf import FINGER_LEFT, FINGER_TRAVEL_M, ObjectSpec
from .gripper_sim import GripperScene, SceneArmBackend, SceneGripper
from .kinematics import R1Kinematics
from .limits import R1_MAX_TORQUE
from .robot import R1Robot
from .safety import SafetyConfig
from .poses import SAFE_HOME
from .types import ArmCommand, ControlMode, NUM_JOINTS

#: Gripper openings, metres of driven-finger travel (0.05 = joint limit).
GRIPPER_OPEN_M = 0.026        #: fingertip gap ~52 mm: clears a 40 mm object
GRIPPER_GRASP_M = 0.019       #: gap ~38 mm against a 40 mm object: 2 mm of squeeze
GRIPPER_RELEASE_M = 0.026

#: Arm gains.  Inside SafetyConfig (max_kp 100, max_kd 5, max_tau 30).
DEFAULT_KP = 80.0
DEFAULT_KD = 4.0
TAU_CAP_Nm = 24.0
MAX_STEP_PER_COMMAND_RAD = 0.03   #: SafetyConfig.max_joint_step_rad is 0.05
#: A move counts as arrived only when the measured joints are this close.  The
#: PD inside the frozen envelope droops; admitting that is the point.
ARRIVAL_TOL_RAD = 0.08


@dataclass
class MoveResult:
    reached: bool = False
    cycles: int = 0
    refused: int = 0
    saturated: int = 0
    min_distance_rad: float = float("inf")
    final_error_rad: float = float("inf")
    reason: str = ""


@dataclass
class StepResult:
    """One step of the pick-and-place sequence, with what the physics said."""

    name: str
    ok: bool
    detail: str = ""
    cycles: int = 0
    refused: int = 0
    saturated: int = 0


@dataclass
class TrialResult:
    """The whole task, and the three things it is judged on."""

    steps: list[StepResult] = field(default_factory=list)
    grasped: bool = False
    lifted: bool = False
    placed: bool = False
    #: Measured outcomes, not assertions: how far the object actually rose, the
    #: grip force it reached, where it ended up.
    metrics: dict = field(default_factory=dict)
    rejections: list[str] = field(default_factory=list)

    @property
    def success(self) -> bool:
        return bool(self.grasped and self.lifted and self.placed
                    and all(s.ok for s in self.steps))

    def failure_reason(self) -> str:
        for s in self.steps:
            if not s.ok:
                return f"{s.name}: {s.detail}"
        if not self.grasped:
            return "grip: no stable contact"
        if not self.lifted:
            return "lift: object did not rise with the gripper"
        if not self.placed:
            return "place: object not delivered inside the target"
        return ""
    reached: bool = False
    cycles: int = 0
    refused: int = 0
    saturated: int = 0
    min_distance_rad: float = float("inf")
    final_error_rad: float = float("inf")
    reason: str = ""


@dataclass
class SceneRig:
    """One experiment instance: physics + the frozen control objects."""

    scene: GripperScene
    backend: SceneArmBackend
    gripper: SceneGripper
    robot: R1Robot
    kin: GripperKinematics
    solver: IKSolver
    kp: float = DEFAULT_KP
    kd: float = DEFAULT_KD
    dt: float = 0.02
    last_refusal: str = ""
    _sampler: WorkspaceSampler | None = None
    #: When set, every control cycle appends a telemetry row for this object
    #: (Task D).  Unset outside a trial so a bring-up does not fill the log.
    log_object: str | None = None
    log: list = field(default_factory=list)

    # -- construction -------------------------------------------------------
    @classmethod
    def create(cls, objects: Iterable[ObjectSpec] = (), *, seed: int = 0,
               model_path: Path | str | None = None,
               timestep: float = 0.002,
               sampler: WorkspaceSampler | None = None) -> "SceneRig":
        scene = GripperScene(objects=list(objects), seed=seed,
                             model_path=model_path, timestep=timestep)
        backend = SceneArmBackend(scene)
        gripper = SceneGripper(scene)
        # NOTE: the default SafetyConfig is used unchanged.  Nothing here widens
        # an envelope; if a motion is refused, that is a result to report.
        robot = R1Robot(backend=backend, safety=SafetyConfig())
        kin = GripperKinematics(model_path=model_path)
        rig = cls(scene=scene, backend=backend, gripper=gripper, robot=robot,
                  kin=kin, solver=IKSolver(kin))
        # A sweep builds one workspace sample set and hands it to every trial:
        # forward-kinematics over 100k configurations is the slow part of planning
        # a grasp, and the set does not depend on the scene's objects.
        rig._sampler = sampler
        return rig

    # -- bring-up -----------------------------------------------------------
    def bringup(self, q_start, *, settle_s: float = 1.0) -> None:
        """Connect, place the arm at the initial condition, arm, hold, then
        verify the pose was actually held.

        Order matters and is not a style choice: `enable_motion()` is the frozen
        gate, so commands before it are refused and the settle has to happen
        after arming.  The verification is what makes this an honest start:
        teleported joint values are not evidence that the pose is held, and a rig
        whose initial condition is a penetration reports that instead of
        continuing from a fantasy.  The test that catches this: a grasp pose that
        buries the gripper body in the work surface drives the contact force to
        kilo-newtons and throws the block 1.5 m across the scene, and with the
        check in place it fails loudly rather than silently.
        """
        self.backend.connect()
        self.gripper.connect()
        self.place_arm_for_setup(q_start)
        self.robot.enable_motion(human_confirmed=True)
        self.hold(settle_s, q_start)
        self.verify_initial_condition(q_start)

    def verify_initial_condition(self, q_start, *, pose_tol_rad: float = 0.12,
                                 max_contact_force_n: float = 25.0) -> None:
        err = float(np.abs(np.asarray(q_start, dtype=float)
                           - self.scene.arm_joint_positions).max())
        worst = max((r.normal_force_n for r in self.scene.contacts()),
                    default=0.0)
        if err > pose_tol_rad:
            raise RuntimeError(
                f"initial condition not held: {err:.3f} rad from the commanded "
                f"start pose ({self.last_refusal or 'no refusal seen'})")
        if worst > max_contact_force_n:
            raise RuntimeError(
                f"initial condition penetrates the scene: {worst:.1f} N of "
                f"contact force, expected a settled pose under "
                f"{max_contact_force_n} N")

    def place_arm_for_setup(self, q) -> None:
        """Teleport the arm.  SETUP ONLY - never call this inside a trial."""
        self.scene.set_arm_positions(q)

    # -- primitives ---------------------------------------------------------
    def command(self, q_setpoint, tau_ff=None) -> bool:
        """Send one FOC position frame through the safety monitor."""
        cmd = ArmCommand()
        cmd.mode[:] = ControlMode.FOC
        cmd.q = np.asarray(q_setpoint, dtype=float)
        cmd.dq = np.zeros(NUM_JOINTS)
        cmd.kp[:] = self.kp
        cmd.kd[:] = self.kd
        if tau_ff is not None:
            # The monitor caps |tau| at SafetyConfig.max_tau (30 N m) and the
            # frozen per-joint ratings are lower from J3 on, so honour both.
            cap = np.minimum(TAU_CAP_Nm, R1_MAX_TORQUE)
            cmd.tau = np.clip(np.asarray(tau_ff, dtype=float), -cap, cap)
        try:
            self.robot.send_action(cmd)
            return True
        except (ValueError, PermissionError, RuntimeError) as exc:
            # Kept, not swallowed: "why did the plan stop here" is the question
            # an experiment log has to answer.
            self.last_refusal = str(exc)
            return False

    def hold(self, seconds: float, q_ref=None) -> int:
        """Hold a FIXED setpoint for `seconds`, with gravity feed-forward.

        The setpoint must be fixed.  Re-reading the measured joints every tick
        and commanding them back makes the proportional term exactly zero, so
        the "hold" degenerates into free fall - which is how this rig initially
        dropped the arm onto its limits before the bug was fixed.
        """
        ref = (self.scene.arm_joint_positions.copy() if q_ref is None
               else np.asarray(q_ref, dtype=float))
        cycles = 0
        for _ in range(max(1, int(round(seconds / self.dt)))):
            self.command(ref, self.gravity_feedforward(ref))
            self.scene.step(self.dt)
            if self.log_object is not None:
                self.snapshot(self.log_object)
            cycles += 1
        return cycles

    def move_to(self, q_target, *, timeout: float = 20.0) -> MoveResult:
        """Ramp a setpoint toward `q_target` and let the PD follow it.

        Two separate limits, and the reason they are separate:

        * the commanded setpoint moves at most `MAX_STEP_PER_COMMAND_RAD` per
          tick - that is the trajectory generator's own rate limit,
        * it never leads the *measured* joints by more than the monitor's
          `max_joint_step_rad`, because that comparison is what actually refuses
          a command.  Capping the lead keeps a droop from tripping the envelope
          instead of the plan.

        A refusal is reported in `reason`, never patched over by loosening a
        limit or writing straight to the backend.
        """
        q_target = np.asarray(q_target, dtype=float)
        lead = self.robot.safety.max_joint_step_rad - 0.01
        res = MoveResult()
        cmd_q = self.scene.arm_joint_positions.copy()
        sat0 = self.scene.saturation_events

        for i in range(max(1, int(round(timeout / self.dt)))):
            measured = self.scene.arm_joint_positions
            err = q_target - cmd_q
            if np.all(np.abs(err) < 1e-3):
                res.reached = True
                break
            step = np.clip(err, -MAX_STEP_PER_COMMAND_RAD,
                           MAX_STEP_PER_COMMAND_RAD)
            cmd_q = np.clip(cmd_q + step, measured - lead, measured + lead)
            if not self.command(cmd_q, self.gravity_feedforward(cmd_q)):
                res.refused += 1
                res.reason = f"command refused: {self.last_refusal}"
                break
            self.scene.step(self.dt)
            if self.log_object is not None:
                self.snapshot(self.log_object)
            res.cycles = i + 1
            res.min_distance_rad = min(res.min_distance_rad,
                                       float(np.abs(q_target - measured).max()))

        res.final_error_rad = float(np.abs(q_target
                                           - self.scene.arm_joint_positions).max())
        res.saturated = self.scene.saturation_events - sat0
        if res.reached and res.final_error_rad > ARRIVAL_TOL_RAD:
            res.reached = False
            res.reason = (f"setpoint arrived but the arm settled "
                          f"{res.final_error_rad:.3f} rad away "
                          f"(tolerance {ARRIVAL_TOL_RAD})")
        elif res.reached:
            res.reason = "ok"
        elif not res.reason:
            res.reason = (f"timed out after {res.cycles} cycles, still "
                          f"{res.final_error_rad:.3f} rad from target")
        return res

    def gravity_feedforward(self, q) -> np.ndarray:
        """Pinocchio gravity term for the arm-only model.

        It does NOT know about the gripper, the adapter or a held object, so it
        under-compensates; the residual droop is part of what is measured.
        """
        try:
            return self.kin.kin.gravity_torques(q)
        except Exception:      # a model failure must not stop a control tick
            return np.zeros(NUM_JOINTS)

    # -- gripper ------------------------------------------------------------
    def gripper_position(self, metres: float, *, mode=GripperCommandMode.POSITION
                         ) -> None:
        self.gripper.write_command(
            GripperCommand(mode=mode, position=metres / FINGER_TRAVEL_M))

    def gripper_open(self) -> None:
        self.gripper_position(GRIPPER_OPEN_M)

    def gripper_close_to_grasp(self) -> None:
        self.gripper_position(GRIPPER_GRASP_M)

    def squeeze_further(self, extra_m: float) -> None:
        self.gripper_position(min(GRIPPER_GRASP_M + abs(extra_m),
                                  FINGER_TRAVEL_M))

    # -- IK planning --------------------------------------------------------
    def sampler(self) -> WorkspaceSampler:
        """One FK sample set per rig: building it costs seconds, not per plan."""
        if self._sampler is None:
            self._sampler = WorkspaceSampler(self.kin)
        return self._sampler

    def to_base_frame(self, position) -> np.ndarray:
        """World point -> the arm's own base frame.

        The scene puts the robot on a deck, so MuJoCo world coordinates and the
        Pinocchio model the IK works in differ by the base offset.  Reading it from
        the compiled model rather than assuming a shared origin is what keeps a
        grasp target from being silently wrong by the deck height; the check below
        also refuses a rotated base, because a pure subtraction would be the wrong
        transform for one.
        """
        base = self.scene.robot_base_position()
        quat = self.scene.robot_base_quat()
        if abs(abs(float(quat[0])) - 1.0) > 1e-9:
            raise RuntimeError(
                f"the arm base is rotated in the scene ({quat}); the IK frame "
                f"conversion here only handles a translated base")
        return np.asarray(position, dtype=float) - base

    def plan_grasp(self, object_pos, *, approach=None, jaw=None,
                   extra_seeds=(), tip_tol_m: float = 0.05,
                   roll_weight: float = 0.2,
                   position_tol: float = 2e-4,
                   orientation_tol: float = 5e-3) -> IkResult:
        """Reachable arm configuration whose FINGERTIP PLANE sits on `object_pos`.

        `object_pos` is a point in the scene's world frame, the same frame
        `GripperScene.object_pose` reports; it is converted to the arm's base frame
        before IK sees it (see `to_base_frame`).

        The approach and jaw directions are preferences, resolved the honest way:
        the workspace sampler supplies the seeds the reachable set actually
        contains (this arm cannot point straight down near its own base height, and
        guessing that it can only makes IK pin J2 at its zero lower bound), and
        DLS then refines the pose exactly.  Nothing here invents a target the arm
        cannot reach; if none exists the result comes back not-ok with its
        problems listed.

        `position_tol` / `orientation_tol` are part of the *grasp specification*,
        not solver tuning: demanding an exactly vertical tool asks the arm for a
        pose it does not contain.  Measured on 60k in-limit samples around
        x = 0.32 m on the work surface, the best fingertip-plane pose points the
        tool 18 deg off vertical, and asking for 0 deg makes the solver slide off
        that pose into a limit-pinned one 95 mm away.  So a trial asks for what
        the arm can do and reports the tilt it actually got (`tilt_error_rad`).
        """
        obj = self.to_base_frame(object_pos)
        target = self.grasp_target(obj, approach)
        seeds = [np.asarray(s, dtype=float) for s in extra_seeds]
        seeds += self.sampler().seeds_for(obj, jaw_axis=jaw,
                                          approach_axis=approach,
                                          tip_tol_m=tip_tol_m)
        seeds += default_seeds(6)
        return self.solver.solve_from_seeds(
            make_pose(target, approach if approach is not None
                      else (0.0, 0.0, -1.0),
                      jaw if jaw is not None else (0.0, 1.0, 0.0)),
            seeds, position_tol=position_tol, orientation_tol=orientation_tol,
            roll_weight=roll_weight)

    #: A grasp pose is accepted if the tool points within this much of the
    #: requested approach.  0.35 rad = 20 deg, comfortably above the 18 deg the
    #: workspace contains at the work surface and low enough that the jaws still
    #: close on opposing faces of a 40 mm object.
    GRASP_TILT_TOL_RAD = 0.35

    #: Approach directions worth trying, worst first.  Straight down is listed
    #: last because with this side mount the wrist hangs ~0.12 m BELOW the
    #: fingertip plane, so a vertical approach buries Link7 in the work surface;
    #: a tilted approach keeps the wrist clear.  `None` means "let the arm point
    #: wherever it likes and judge the pose by its jaw axis".
    GRASP_APPROACHES = (
        (0.45, 0.0, -0.89), (0.70, 0.0, -0.71), (0.87, 0.0, -0.50),
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0), None,
    )

    @staticmethod
    def _jaw_for(approach, jaw) -> tuple:
        """A jaw axis perpendicular to the approach, closest to the request."""
        if approach is None:
            return tuple(jaw)
        a = np.asarray(approach, dtype=float)
        a = a / np.linalg.norm(a)
        j = np.asarray(jaw, dtype=float)
        j = j - np.dot(j, a) * a
        if np.linalg.norm(j) < 1e-6:
            j = np.array([0.0, 0.0, 1.0]) - np.dot([0.0, 0.0, 1.0], a) * a
        return tuple(j / np.linalg.norm(j))

    def find_grasp(self, object_pos, *, jaw=(0.0, 1.0, 0.0),
                   approaches=None, max_penetration_m: float = 0.002,
                   roll_weight: float = 0.2,
                   position_tol: float = 5e-3,
                   orientation_tol: float | None = None,
                   max_candidates: int = 6,
                   report: list | None = None) -> IkResult | None:
        """The cleanest grasp pose the workspace actually contains.

        "Reachable" is IK's judgement; "clean" is the physics model's: the same
        joint values can sit inside every limit and still bury the gripper body in
        the work surface, which IK cannot see because it has no collisions.
        Every rejection is written into `report` so a failed trial says why.

        The orientation asked for is taken from the sampled reachable set
        (`WorkspaceSampler.grasp_candidates`) rather than from a fixed list: at a
        point whose best in-limit pose tilts 35 deg off vertical, insisting on
        vertical makes the solver trade position for orientation and lose - 176 mm
        of error against a target 9 mm from a valid configuration, measured.  The
        fixed list is kept as a fallback for the case where sampling finds nothing
        near the point.

        Position tolerance is 5 mm because the object is 40 mm wide and the jaws
        open 52 mm: a grasp centred 5 mm off still straddles it, and the approach
        phase lowers the tool onto the block from there.  Tilt tolerance is
        `GRASP_TILT_TOL_RAD`, for the reason given on `plan_grasp`.
        """
        tilt_tol = (self.GRASP_TILT_TOL_RAD if orientation_tol is None
                    else float(orientation_tol))
        obj = self.to_base_frame(object_pos)
        attempts = []
        for cq, capp, cjaw, dist in self.sampler().grasp_candidates(
                obj, count=int(max_candidates)):
            # Cheap pre-filter on the physics model, before paying for IK: a
            # sampled pose that already buries the tool in the work surface cannot
            # be refined into a clean grasp, only into a different penetration.
            # Measured - without it, candidates at a too-low target converged to
            # 3 mm while penetrating the surface by 27-36 mm, so every one of them
            # was a false accept.
            pen0 = self.scene.penetration_at(cq)
            if pen0 > max_penetration_m:
                if report is not None:
                    report.append(f"sampled ({dist * 1e3:.1f} mm): seed penetrates "
                                  f"{pen0 * 1e3:.1f} mm")
                continue
            # A sampled approach/jaw pair is a DIRECTION, and the base is only
            # translated relative to the scene (checked by `to_base_frame`).
            attempts.append((tuple(float(v) for v in capp),
                             tuple(float(v) for v in cjaw), cq,
                             f"sampled ({dist * 1e3:.1f} mm)"))
        if approaches is None:
            approaches = self.GRASP_APPROACHES
        for ap in approaches:
            attempts.append((ap, self._jaw_for(ap, jaw), None, f"approach {ap}"))

        best: tuple | None = None
        for approach, jaw_axis, seed, label in attempts:
            res = self.plan_grasp(object_pos, approach=approach, jaw=jaw_axis,
                                  extra_seeds=() if seed is None else (seed,),
                                  tip_tol_m=0.08, roll_weight=roll_weight,
                                  position_tol=position_tol,
                                  orientation_tol=tilt_tol)
            if not res.ok:
                if report is not None:
                    report.append(f"{label}: {res.problems[:1]} "
                                  f"({res.position_error_m * 1e3:.1f} mm, "
                                  f"{res.tilt_error_rad:.2f} rad off)")
                continue
            if res.tilt_error_rad > tilt_tol:
                if report is not None:
                    report.append(f"{label}: tilt {res.tilt_error_rad:.2f} rad "
                                  f"over {tilt_tol}")
                continue
            pen = self.scene.penetration_at(res.q)
            if pen > max_penetration_m:
                if report is not None:
                    report.append(f"{label}: penetr {pen * 1e3:.1f} mm")
                continue
            key = (pen, res.tilt_error_rad, res.position_error_m)
            if report is not None:
                report.append(f"{label}: accepted, penetration {pen * 1e3:.2f} "
                              f"mm, tilt {res.tilt_error_rad:.2f} rad, "
                              f"{res.position_error_m * 1e3:.1f} mm off centre")
            if best is None or key < best[0]:
                best = (key, res)
        return best[1] if best else None

    def plan_pose(self, position, approach, opening_deg) -> IkResult:
        a = math.radians(float(opening_deg))
        tgt = make_pose(position, approach if approach is not None
                        else (0.0, 0.0, -1.0), (math.cos(a), math.sin(a), 0.0))
        return self.solver.solve_from_seeds(tgt, default_seeds(10))

    # -- measurement --------------------------------------------------------
    def tip_offset_m(self) -> float:
        """Distance from `tcp_site` to the fingertip plane, toward the tips.

        Measured on the compiled model rather than typed in.  In this model the
        tool point sits BEHIND the fingertips by ~83 mm along the approach axis,
        so aiming `tcp_site` straight at an object centre leaves the fingers
        short of it - the failure this offset exists to prevent.
        """
        p, R = self.scene.tcp_pose()
        v = R.T @ (self.scene.finger_midpoint() - p)
        side = -1.0 if gripper_ik.TCP_APPROACH_AXIS[2] < 0.0 else 1.0
        return float(side * v[2])

    def grasp_target(self, object_pos, approach=None) -> np.ndarray:
        """TCP waypoint that puts the fingertip plane on the object centre."""
        a = (np.array([0.0, 0.0, -1.0]) if approach is None
             else np.asarray(approach, dtype=float))
        return (np.asarray(object_pos, dtype=float)
                - self.tip_offset_m() * a / np.linalg.norm(a))

    def object_z(self, name: str) -> float:
        return self.scene.object_height(name)

    def tcp(self) -> np.ndarray:
        return self.scene.tcp_pose()[0]

    def safe_stop(self) -> None:
        self.robot.emergency_stop()
        self.gripper.stop()

    def grasp_force_n(self, object_name: str) -> float:
        return self.scene.finger_contact_force(object_name)

    def path_clear(self, q_from, q_to, *, steps: int = 16,
                   max_penetration_m: float = 0.002) -> tuple[bool, float, float]:
        """Is the straight joint-space path between two waypoints free?

        Checking only the endpoint is not enough, and this repository found that
        the hard way: a plan whose grasp pose is clean can still take the elbow
        through the object on the way there, and the penetration the simulator
        then resolves throws the block across the scene - which looks like a failed
        grasp but is really an unchecked transit.  Returns (clear, worst depth,
        fraction along the path where it happens).
        """
        a = np.asarray(q_from, dtype=float)
        b = np.asarray(q_to, dtype=float)
        worst, where = 0.0, 0.0
        for i in range(1, int(steps) + 1):
            f = i / steps
            pen = self.scene.penetration_at(a + (b - a) * f)
            if pen > worst:
                worst, where = pen, f
        return (worst <= max_penetration_m, worst, where)

    def goto(self, name: str, q_target, *, trial: "TrialResult",
             steps: int = 16, timeout: float = 20.0) -> bool:
        """Collision-checked move: check the transit, then drive it."""
        clear, worst, where = self.path_clear(self.scene.arm_joint_positions,
                                             q_target, steps=steps)
        if not clear:
            trial.steps.append(
                StepResult(name, False,
                           f"transit blocked: {worst * 1e3:.1f} mm of penetration "
                           f"at {where * 100:.0f}% of the path"))
            return False
        res = self.move_to(q_target, timeout=timeout)
        trial.steps.append(StepResult(name, res.reached, res.reason, res.cycles,
                                      res.refused, res.saturated))
        return res.reached

    def axes_of(self, q) -> tuple[tuple, tuple]:
        """The approach and jaw directions the arm actually holds at `q`.

        Read from FK rather than from the pose IK was asked for, because the next
        waypoint has to be planned in the frame the tool is really in: a grasp pose
        that converged 20 deg off the requested approach is fine to keep working
        from, but only if the approach direction used downstream is the achieved
        one.
        """
        R = self.kin.fk(np.asarray(q, dtype=float))[:3, :3]
        return tuple(-R[:, 2]), tuple(-R[:, 1])

    def plan_at(self, point, approach, jaw, warm_start=None, *,
                position_tol: float = 5e-3) -> IkResult:
        """IK for a waypoint at `point`, keeping the tool axes it already has."""
        seeds = () if warm_start is None else (np.asarray(warm_start, float),)
        return self.plan_grasp(point, approach=approach, jaw=jaw,
                               extra_seeds=seeds, tip_tol_m=0.10,
                               position_tol=position_tol,
                               orientation_tol=self.GRASP_TILT_TOL_RAD)

    def run_pick_and_place(self, object_name: str, *, surface_z: float,
                           object_height: float, object_width: float,
                           place_xy, lift_m: float = 0.10,
                           hold_s: float = 2.0, clear_m: float = 0.07,
                           squeeze_m: float = 0.002,
                           place_tol_m: float = 0.05,
                           q_home=None) -> TrialResult:
        """The ten steps, each one checked against what the physics reported.

        Nothing here asserts success by construction: an object is grasped because
        the contact force rises and it comes up with the fingers, it is lifted
        because its measured height rises, and it is placed because it ends up
        inside the target square at rest.  A step that fails stops the sequence and
        the reason is kept.

        The hand-off between IK and the scene is all in joint values and the axes
        the arm actually holds; the gripping plane is searched for, from the lowest
        height that still leaves 10 mm of pad on the object upward, and a height is
        only taken if the physics model reports no penetration at the pose that
        uses it.
        """
        home = np.asarray(SAFE_HOME if q_home is None else q_home,
                          dtype=float).reshape(NUM_JOINTS)
        self.log_object = object_name
        trial = TrialResult()
        m0 = self.scene.object_pose(object_name)[0].copy()
        obj_xy = self.scene.object_pose(object_name)[0][:2]
        # Where to put the fingertip plane, searched from the lowest grippable
        # height upward.  Not the object centre: this gripper's CAD envelope
        # reaches past its own fingertip sites (`robot_dip_below_tip_m` measures
        # it), so a plane low on a short object drives the tool into the work
        # surface before the pads touch anything.  The penetration test inside
        # `find_grasp` is the arbiter - the search just gives it heights to
        # choose between, and an object too short to offer a clean height is
        # reported as such instead of being grasped at a pose that penetrates.
        min_overlap = 0.010
        heights = [surface_z + h for h in
                   np.arange(0.5 * object_height,
                             object_height - min_overlap + 1e-9, 0.01)]
        grasp = None
        plane_z = float("nan")
        for pz in heights:
            trial.rejections.append(f"plane {pz - surface_z:.3f} above surface:")
            grasp = self.find_grasp((float(obj_xy[0]), float(obj_xy[1]), pz),
                                    roll_weight=0.0, max_candidates=3,
                                    approaches=(), report=trial.rejections)
            if grasp is not None:
                plane_z = pz
                break
        trial.metrics["grip_plane_z"] = plane_z
        if grasp is None:
            trial.steps.append(
                StepResult("plan gripping plane", False,
                           f"no clean grasp for a {object_height * 1e3:.0f} mm "
                           f"object on a {surface_z:.2f} m surface between "
                           f"{heights[-1] - surface_z:.3f} m and "
                           f"{heights[0] - surface_z:.3f} m above it"))
            return trial

        # 1. safe pose
        if not self.goto("1 safe pose", home, trial=trial):
            return trial
        # 2. open the jaws wide enough for the object
        self.gripper_position(0.5 * object_width + 0.006)
        self.scene.step(0.8)
        gap = self.scene.gripper_gap()
        opened = gap >= object_width + 0.008
        trial.steps.append(StepResult("2 open", opened,
                                      f"gap {gap * 1e3:.1f} mm against a "
                                      f"{object_width * 1e3:.0f} mm object"))
        if not opened:
            return trial

        # 3. pre-grasp pose: the gripping plane, raised along the approach
        app, jaw = self.axes_of(grasp.q)
        zu = np.array([0.0, 0.0, 1.0])
        trial.metrics["approach_off_vertical_rad"] = float(
            math.acos(min(1.0, abs(float(np.dot(app, zu))))))
        trial.metrics["jaw_off_horizontal_rad"] = float(
            math.asin(min(1.0, abs(float(np.dot(jaw, zu))))))
        trial.metrics["grasp_tilt_rad"] = grasp.tilt_error_rad
        trial.metrics["grasp_position_error_m"] = grasp.position_error_m
        pre = self.plan_at((float(obj_xy[0]), float(obj_xy[1]), plane_z + clear_m),
                           app, jaw, grasp.q)
        if not pre.ok:
            trial.steps.append(StepResult("3 plan pre-grasp", False,
                                          f"{pre.problems[:1]} "
                                          f"({pre.position_error_m * 1e3:.1f} mm)"))
            return trial
        # A hover high over the object, reached first: the arm cannot go from its
        # home pose straight to a point 100 mm over the bench without the transit
        # passing through the work volume, and `path_clear` says so rather than
        # letting the simulator resolve 40 mm of overlap as an impulse.
        hover = self.plan_at((float(obj_xy[0]), float(obj_xy[1]),
                              plane_z + lift_m + clear_m), app, jaw, pre.q)
        trial.metrics["grasp_tilt_rad"] = grasp.tilt_error_rad
        trial.metrics["grasp_position_error_m"] = grasp.position_error_m
        trial.steps.append(StepResult("3 plan pre-grasp", hover.ok,
                                      f"tilt {grasp.tilt_error_rad:.2f} rad, "
                                      f"{grasp.position_error_m * 1e3:.1f} mm off"))
        if not hover.ok:
            trial.steps.append(StepResult("3 plan hover", False,
                                          f"{hover.problems[:1]}"))
            return trial

        # 4. travel over the object, then descend along the approach direction
        if not self.goto("4 move to hover", hover.q, trial=trial):
            return trial
        if not self.goto("4 approach", pre.q, trial=trial):
            return trial
        # 5. the last 70 mm onto the object
        if not self.goto("5 descend", grasp.q, trial=trial):
            return trial

        # 6. close into contact, ramping until the grip can actually hold the
        #    payload.  Two things are measured rather than assumed here.  The
        #    fingertip SITES are reference points on the CAD link, not the pad
        #    surfaces, so a travel computed from the object width alone closes short
        #    of any load at all (at a 35.9 mm site gap across a 40 mm bar the finger
        #    force was 0.009 N) - a real gripper closes until it feels force, and so
        #    does this.  And the force a grasp has to reach is the payload's own:
        #    weight / (2 pads x friction), computed from the model, not a constant
        #    someone preferred.  SIMULATION ASSUMPTION: the friction is the value the
        #    generator guessed, so this threshold is a simulation statement about the
        #    simulated grasp, never about a real DM4310.
        required = self.hold_force_n(object_name)
        force = 0.0
        travel = 0.5 * object_width - squeeze_m
        for cmd in np.arange(travel, 0.005, -0.001):
            travel = float(cmd)
            self.gripper_position(travel)
            # Settle, then judge.  Measuring while the fingers are still moving
            # reads a transient: the first pass broke out of this ramp on 0.3 N of
            # in-motion force that relaxed to 0.10 N once the grip settled, and the
            # trial then reported a grasp that could not have held the payload.
            self.scene.step(0.4)
            force = self.scene.finger_contact_force(object_name)
            if force >= required:
                break
        trial.grasped = bool(self.scene.finger_touching(object_name)
                             and force >= required)
        trial.metrics["close_force_n"] = force
        trial.metrics["required_grip_force_n"] = required
        trial.metrics["close_travel_m"] = travel
        trial.metrics["closed_gap_m"] = self.scene.gripper_gap()
        trial.metrics["site_to_pad_offset_m"] = float(
            self.scene.gripper_gap() - object_width)
        trial.steps.append(StepResult("6 close", trial.grasped,
                                      f"{force:.2f} N of {required:.2f} N needed, "
                                      f"at {travel * 1e3:.1f} mm travel, site gap "
                                      f"{trial.metrics['closed_gap_m'] * 1e3:.1f} mm"))
        if not trial.grasped:
            return trial

        # 7. lift and hold
        lifted_point = (float(obj_xy[0]), float(obj_xy[1]), plane_z + lift_m)
        lift = self.plan_at(lifted_point, app, jaw, grasp.q)
        if not lift.ok:
            trial.steps.append(StepResult("7 plan lift", False,
                                          f"{lift.problems[:1]}"))
            return trial
        if not self.goto("7 lift", lift.q, trial=trial):
            return trial
        forces: list = []
        heights: list = []
        z_after = self._hold_and_watch(lift.q, hold_s, object_name, forces,
                                       heights)
        rose = z_after - float(m0[2])
        trial.lifted = bool(rose >= 0.8 * lift_m
                            and self.scene.finger_touching(object_name))
        trial.metrics["lift_m"] = rose
        trial.metrics["hold_force_mean_n"] = (float(np.mean(forces))
                                              if forces else 0.0)
        trial.metrics["hold_force_peak_n"] = (float(np.max(forces))
                                              if forces else 0.0)
        # Slip is the object moving in the grip while the setpoint is fixed: the
        # spread of its measured height during the hold, not its rise.
        trial.metrics["slip_m"] = (float(np.max(heights) - np.min(heights))
                                   if heights else 0.0)
        trial.steps.append(StepResult(f"7 hold {hold_s:.0f} s", trial.lifted,
                                      f"rose {rose * 1e3:.0f} mm of "
                                      f"{lift_m * 1e3:.0f} mm, grip "
                                      f"{trial.metrics['hold_force_peak_n']:.2f} N"))
        if not trial.lifted:
            return trial

        # 8. carry to the target and set down
        carry = self.plan_at((float(place_xy[0]), float(place_xy[1]),
                              plane_z + lift_m), app, jaw, lift.q)
        if not carry.ok:
            trial.steps.append(StepResult("8 plan carry", False,
                                          f"{carry.problems[:1]}"))
            return trial
        if not self.goto("8 carry", carry.q, trial=trial):
            return trial
        down = self.plan_at((float(place_xy[0]), float(place_xy[1]), plane_z),
                            app, jaw, carry.q)
        if not down.ok:
            trial.steps.append(StepResult("8 plan set-down", False,
                                          f"{down.problems[:1]}"))
            return trial
        if not self.goto("8 set down", down.q, trial=trial):
            return trial

        # 9. release
        self.gripper_position(0.5 * object_width + 0.006)
        self.scene.step(1.2)
        end = self.scene.object_pose(object_name)[0]
        placed_dx = float(np.linalg.norm(end[:2] - np.asarray(place_xy, float)))
        settled = abs(end[2] - (surface_z + 0.5 * object_height)) < 0.03
        trial.placed = bool(placed_dx <= place_tol_m and settled)
        trial.metrics["place_error_m"] = placed_dx
        trial.metrics["final_object_xyz"] = [round(float(v), 6) for v in end]
        trial.steps.append(StepResult("9 release", trial.placed,
                                      f"{placed_dx * 1e3:.0f} mm from the target, "
                                      f"z {end[2]:.3f} (settled: {settled})"))

        # 10. retract and return to the safe pose
        away = self.plan_at((float(place_xy[0]), float(place_xy[1]),
                             plane_z + clear_m), app, jaw, down.q)
        if away.ok:
            self.goto("10 retract", away.q, trial=trial)
        self.goto("10 safe pose", home, trial=trial)
        return trial

    def hold_force_n(self, object_name: str) -> float:
        """Normal force each pad needs to hold this object against gravity.

        weight / (2 pads x tangential friction), with the mass read from the scene
        and the friction from the compiled model - so the grasp threshold is
        derived from what is being lifted instead of being a number that happens to
        make the test pass.  Both inputs are simulation assumptions.
        """
        spec = next((o for o in self.scene.objects if o.name == object_name), None)
        mass = float(getattr(spec, "mass", 0.05))
        mu = 0.9
        for gi in range(self.scene.model.ngeom):
            if self.scene._body_name(gi) == FINGER_LEFT + "_link" and \
                    int(self.scene.model.geom_contype[gi]):
                mu = float(self.scene.model.geom_friction[gi][0])
                break
        return mass * 9.81 / (2.0 * max(mu, 1e-3))

    def _hold_and_watch(self, q_ref, seconds: float, object_name: str,
                        forces: list, heights: list) -> float:
        """Hold a fixed setpoint, sampling the grip force and object height."""
        for _ in range(max(1, int(round(seconds / self.dt)))):
            self.command(q_ref, self.gravity_feedforward(q_ref))
            self.scene.step(self.dt)
            forces.append(self.scene.finger_contact_force(object_name))
            heights.append(self.scene.object_height(object_name))
            self.snapshot(object_name)
        return self.scene.object_height(object_name)

        self.robot.emergency_stop()
        self.gripper.stop()

    def snapshot(self, object_name: str | None = None) -> dict:
        """One row of experiment telemetry (Task D)."""
        q = self.scene.arm_joint_positions
        dq = self.scene.arm_joint_velocities
        tau = self.scene.arm_joint_torques
        gq = self.scene.gripper_metres()
        row = {
            "t_s": round(self.scene.sim_time, 6),
            "q": [round(float(v), 6) for v in q],
            "dq": [round(float(v), 6) for v in dq],
            "tau": [round(float(v), 6) for v in tau],
            "gripper_m": round(gq, 6),
            "gripper_norm": round(gq / FINGER_TRAVEL_M, 6),
            "gripper_rate_m_s": round(self.scene.gripper_rate(), 6),
            "gripper_channel_force_n": round(self.scene.gripper_channel_force_n(),
                                             6),
            "gripper_gap_m": round(self.scene.gripper_gap(), 6),
            "tcp": [round(float(v), 6) for v in self.tcp()],
            "max_joint_tau_ratio": round(float(np.max(np.abs(tau)
                                                      / R1_MAX_TORQUE)), 6),
        }
        if object_name is not None:
            pos, quat = self.scene.object_pose(object_name)
            row["object_xyz"] = [round(float(v), 6) for v in pos]
            row["object_quat_wxyz"] = [round(float(v), 6) for v in quat]
            row["contact_force_n"] = round(
                self.scene.finger_contact_force(object_name), 6)
            row["touching"] = bool(self.scene.finger_touching(object_name))
            row["max_penetration_m"] = round(self.scene.max_penetration_m(), 6)
        self.log.append(row)
        return row
