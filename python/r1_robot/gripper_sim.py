"""Simulation scene for the R1 + reBot gripper: a MuJoCo world plus two adapters.

The scene owns physics.  `SceneArmBackend` and `SceneGripper` expose it through
the SAME abstract interfaces the offline stack already uses (`ArmBackend` with a
fixed seven-joint `ArmCommand`, and `Gripper` with a normalised opening in
[0, 1]), so an experiment driven through these adapters still runs the existing
safety checks: the point of the adapters is that they are the only way in.
Nothing here re-implements gating, limits or trajectory execution.

Everything about the plant is a simulation assumption.  Contact stiffness,
friction, damping, armature and the gripper force ceiling come from
`gripper_mjcf.GRIPPER_PROVENANCE`; none is measured on hardware, and the forces
reported here are simulated contact forces, not claimed DM4310 grasp forces.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from r1_robot.backends import ArmBackend
from r1_robot.gripper import (
    Gripper,
    GripperCommand,
    GripperCommandMode,
    GripperLimits,
    GripperState,
)
from r1_robot.gripper_mjcf import (
    ARM_JOINTS,
    FINGER_LEFT,
    FINGER_RIGHT,
    FINGER_TRAVEL_M,
    GRIPPER_CHANNEL,
    GRIPPER_MODEL_PATH,
    ObjectSpec,
)
from r1_robot.types import (
    NUM_JOINTS,
    ArmCommand,
    ControlMode,
    RobotState,
    zeros_joints,
)

#: The two finger bodies named in the generated model.
FINGER_BODIES = (f"{FINGER_LEFT}_link", f"{FINGER_RIGHT}_link")


@dataclass
class ContactReport:
    """Aggregated contact evidence for one pair of bodies."""

    geom1: str
    geom2: str
    count: int
    normal_force_n: float
    tangent_force_n: float
    distance_m: float

    @property
    def pair(self) -> tuple[str, str]:
        return (self.geom1, self.geom2)


class GripperScene:
    """The MuJoCo world: gripper-equipped arm, table, floor and free objects."""

    def __init__(self, model_path: Path | str | None = None,
                 objects: Iterable[ObjectSpec] | None = None, *,
                 timestep: float = 0.002,
                 gravity: tuple = (0.0, 0.0, -9.81),
                 seed: int = 0) -> None:
        import mujoco  # lazy optional dependency, like sim.py

        self._mj = mujoco
        self.model_path = Path(model_path or GRIPPER_MODEL_PATH)
        self.objects = [ObjectSpec(**o) if isinstance(o, dict) else o
                        for o in (objects or [])]
        self.object_names = [o.name for o in self.objects]
        self.timestep = float(timestep)
        self._rng = np.random.default_rng(seed)

        self.model = mujoco.MjModel.from_xml_string(self._scene_xml())
        self.model.opt.timestep = self.timestep
        self.model.opt.gravity[:] = gravity
        self.data = mujoco.MjData(self.model)

        self._resolve_channels()
        self._resolve_joints()
        self.reset()

    # -- scene construction -------------------------------------------------
    def _scene_xml(self) -> str:
        """The generated model plus this experiment's free objects, in memory.

        Two rewrites, both deliberate:

        * `compiler/@meshdir` becomes absolute, because `from_xml_string` has no
          base directory and the model's portable relative mesh path would not
          resolve.  The artifact on disk keeps its relative path.
        * objects come from the experiment spec, never from editing the
          generated robot model, so the plant stays a regenerable artifact.
        """
        import xml.etree.ElementTree as ET

        root = ET.fromstring(self.model_path.read_text())
        compiler = root.find("compiler")
        if compiler is not None and compiler.get("meshdir"):
            compiler.set("meshdir",
                         str((self.model_path.parent
                              / compiler.get("meshdir")).resolve()))
        world = root.find("worldbody")
        for obj in self.objects:
            body = ET.SubElement(
                world, "body",
                {"name": obj.name,
                 "pos": " ".join(f"{v:.9g}" for v in obj.pos)})
            attrs = {"name": f"{obj.name}_geom", "class": "object"}
            attrs.update(obj.geom_attrs())
            if obj.shape == "box":
                half = [v / 2.0 for v in obj.size]
                i1, i2, i3 = _box_inertia(obj.mass, half)
            elif obj.shape == "cylinder":
                r, h = obj.size
                i1 = i2 = obj.mass * (3 * r * r + 4 * h * h) / 12.0
                i3 = obj.mass * r * r / 2.0
            else:
                raise ValueError(f"unknown object shape {obj.shape!r}")
            ET.SubElement(body, "inertial",
                          {"pos": "0 0 0",
                           "mass": f"{obj.mass:.9g}",
                           "diaginertia": " ".join(f"{v:.9g}"
                                                   for v in (i1, i2, i3))})
            ET.SubElement(body, "freejoint", {"name": f"{obj.name}_free"})
            geom = ET.SubElement(body, "geom", attrs)
            geom.set("rgba", " ".join(f"{v:.3g}" for v in obj.rgba))
            ET.SubElement(body, "site",
                          {"name": f"{obj.name}_site", "type": attrs["type"],
                           "size": attrs["size"], "group": "4",
                           "rgba": "0 0 1 0"})
        return ET.tostring(root, encoding="unicode")

    # -- model resolution ---------------------------------------------------
    def _resolve_channels(self) -> None:
        mj, m = self._mj, self.model
        arm, gripper = {}, None
        for i in range(m.nu):
            nm = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, i)
            if nm == GRIPPER_CHANNEL:
                gripper = i
            elif nm and nm.endswith("_ctrl"):
                arm[nm[:-5]] = i
        if sorted(arm) != sorted(ARM_JOINTS):
            raise RuntimeError(
                f"expected arm channels {ARM_JOINTS}, resolved {sorted(arm)}")
        if gripper is None:
            raise RuntimeError(f"the model has no {GRIPPER_CHANNEL!r} channel")
        #: arm channel index per joint name, in URDF order
        self.arm_index = {n: arm[n] for n in ARM_JOINTS}
        self.gripper_index = gripper
        self._arm_ctrl_names = list(ARM_JOINTS)

    def _resolve_joints(self) -> None:
        mj, m = self._mj, self.model
        self._qadr, self._dofadr = {}, {}
        for n in list(ARM_JOINTS) + [FINGER_LEFT, FINGER_RIGHT]:
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)
            if jid < 0:
                raise RuntimeError(f"model is missing joint {n!r}")
            self._qadr[n] = int(m.jnt_qposadr[jid])
            self._dofadr[n] = int(m.jnt_dofadr[jid])
        self._arm_q = [self._qadr[n] for n in self._arm_ctrl_names]
        self._arm_d = [self._dofadr[n] for n in self._arm_ctrl_names]

    # -- time ---------------------------------------------------------------
    def reset(self) -> None:
        self._mj.mj_forward(self.model, self.data)
        self.sim_time = 0.0
        self.saturation_events = 0

    def step(self, dt: float) -> None:
        """Advance physics by ``dt`` seconds (whole 2 ms steps).

        `saturation_events` counts physics steps on which at least one arm
        channel sat at its `forcerange` bound - i.e. the plan asked for more
        torque than this plant can deliver.  Reported, never smoothed over.
        """
        if dt <= 0.0:
            return
        steps = max(1, int(round(dt / self.timestep)))
        for _ in range(steps):
            self._mj.mj_step(self.model, self.data)
            if self.saturated_channels():
                self.saturation_events += 1
        self.sim_time += steps * self.timestep

    def settle(self, seconds: float) -> None:
        self.step(seconds)

    # -- actuation ----------------------------------------------------------
    def write_arm_command(self, cmd: ArmCommand) -> None:
        """Install one command frame's PD law INTO the arm actuators.

        The gains go to `actuator_biasprm` and the feed-forward to `ctrl`, the
        same way `sim.SimArm` does it, because the command loop must not be what
        evaluates the spring.  Measured with the alternative (torque computed
        here at 50 Hz and held across ten 2 ms steps): a wrist inertia of
        ~1.5e-4 kg m^2 with kp=80 diverges within four ticks, J6 running away
        0.6 rad while the setpoint sits still.  Inside the actuator MuJoCo
        re-evaluates the PD every 2 ms and linearises the damping term
        implicitly, so the same gains are stable.

        Mode STOP clears the gains as well as the feed-forward: that is the
        "motor output disabled" behaviour of guarantee 7 - no holding torque, the
        arm is left to gravity, and a gravity-loaded arm is not braked.
        """
        for i, name in enumerate(self._arm_ctrl_names):
            idx = self.arm_index[name]
            bias = self.model.actuator_biasprm[idx]
            if cmd.mode[i] != ControlMode.FOC:
                bias[1] = bias[2] = 0.0
                self.data.ctrl[idx] = 0.0
                continue
            kp, kd = float(cmd.kp[i]), float(cmd.kd[i])
            bias[1], bias[2] = -kp, -kd
            self.data.ctrl[idx] = (kp * float(cmd.q[i])
                                   + kd * float(cmd.dq[i]) + float(cmd.tau[i]))

    def saturated_channels(self) -> list[str]:
        """Arm channels currently pinned at their `forcerange` bound."""
        out = []
        for name, idx in self.arm_index.items():
            lo, hi = self.model.actuator_forcerange[idx]
            f = self.data.actuator_force[idx]
            if self.model.actuator_forcelimited[idx] and (
                    f >= hi - 1e-6 or f <= lo + 1e-6):
                out.append(name)
        return out

    def arm_force(self, name: str) -> float:
        """Torque actually produced on one arm channel (post-clamp, N m)."""
        return float(self.data.actuator_force[self.arm_index[name]])

    def set_gripper_metres(self, target_m: float) -> None:
        """Command the driven finger setpoint in metres of opening."""
        lo, hi = self.model.actuator_ctrlrange[self.gripper_index]
        self.data.ctrl[self.gripper_index] = float(np.clip(target_m, lo, hi))

    def gripper_metres(self) -> float:
        return float(self.data.qpos[self._qadr[FINGER_LEFT]])

    def gripper_rate(self) -> float:
        return float(self.data.qvel[self._dofadr[FINGER_LEFT]])

    def gripper_channel_force_n(self) -> float:
        """Force the servo channel is currently delivering (N, signed).

        For the prismatic finger joint `actuator_force` is a force, not a
        torque.  It is what the simulated servo pushes with, which is not the
        same thing as the force at the fingertip and is not measured hardware.
        """
        return float(self.data.actuator_force[self.gripper_index])

    # -- configuration ------------------------------------------------------
    def set_arm_positions(self, q) -> None:
        """Teleport the arm configuration.  Setup only: never an experiment step."""
        q = np.asarray(q, dtype=float)
        if q.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} joint values, got {q.shape}")
        if not np.all(np.isfinite(q)):
            raise ValueError("configuration contains NaN/Inf")
        for i, name in enumerate(self._arm_ctrl_names):
            self.data.qpos[self._qadr[name]] = float(q[i])
        self.data.qvel[self._arm_d] = 0.0
        self._mj.mj_forward(self.model, self.data)

    def set_gripper_position(self, metres: float) -> None:
        """Teleport the driven finger (and its mimic) for scene setup only."""
        self.data.qpos[self._qadr[FINGER_LEFT]] = float(metres)
        self.data.qpos[self._qadr[FINGER_RIGHT]] = -float(metres)
        self.data.qvel[self._dofadr[FINGER_LEFT]] = 0.0
        self.data.qvel[self._dofadr[FINGER_RIGHT]] = 0.0
        self._mj.mj_forward(self.model, self.data)

    def place_object(self, name: str, pos, quat_wxyz=None) -> None:
        """Teleport a free object for scene setup only.

        Reachable from the harness, never from a control path: an experiment
        must move the block by pushing on it, not by editing its pose.
        """
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"no body named {name!r}")
        jid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_JOINT,
                                  f"{name}_free")
        if jid < 0:
            raise KeyError(f"{name!r} has no free joint; it is not a placed object")
        adr = int(self.model.jnt_qposadr[jid])
        self.data.qpos[adr:adr + 3] = np.asarray(pos, dtype=float)
        if quat_wxyz is not None:
            self.data.qpos[adr + 3:adr + 7] = np.asarray(quat_wxyz, dtype=float)
        self.data.qvel[[self.model.jnt_dofadr[jid] + k for k in range(6)]] = 0.0
        self._mj.mj_forward(self.model, self.data)

    # -- frame queries ------------------------------------------------------
    def site_xpos(self, name: str) -> np.ndarray:
        sid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_SITE, name)
        if sid < 0:
            raise KeyError(f"no site named {name!r}")
        return self.data.site_xpos[sid].copy()

    def tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Position and 3x3 rotation of `tcp_site` in the world frame."""
        sid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_SITE,
                                  "tcp_site")
        if sid < 0:
            raise KeyError("the model has no tcp_site")
        return (self.data.site_xpos[sid].copy(),
                self.data.site_xmat[sid].reshape(3, 3).copy())

    def tcp_quat(self) -> np.ndarray:
        """TCP orientation as (w, x, y, z), straight from the site frame."""
        sid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_SITE,
                                  "tcp_site")
        if sid < 0:
            raise KeyError("the model has no tcp_site")
        return self.data.site_xquat[sid].copy()

    def body_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"no body named {name!r}")
        return self.data.xpos[bid].copy(), self.data.xquat[bid].copy()

    def robot_base_position(self) -> np.ndarray:
        """World position of the arm's own base frame (the body carrying J1).

        Pinocchio's model and the joint table in this repository are both expressed
        in that frame, while MuJoCo reports world coordinates, so every hand-off
        between IK and the scene needs this offset.  It is read from the compiled
        model rather than assumed to be the origin: the generator stands the arm on
        a deck height, and a hard-coded zero there silently moves every grasp
        target by that height.
        """
        mj, m = self._mj, self.model
        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, ARM_JOINTS[0])
        if jid < 0:
            raise KeyError(f"model has no joint {ARM_JOINTS[0]!r}")
        # A MuJoCo joint belongs to its CHILD body; the arm's base frame is the
        # link the joint hangs off, which is one level up.  Reading the joint's own
        # body instead offsets every target by the shoulder rise (0.069 m here).
        bid = int(m.body_parentid[int(m.jnt_bodyid[jid])])
        return self.data.xpos[bid].copy()

    def robot_base_quat(self) -> np.ndarray:
        """Orientation (w, x, y, z) of the arm base frame in the world."""
        mj, m = self._mj, self.model
        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, ARM_JOINTS[0])
        bid = int(m.body_parentid[int(m.jnt_bodyid[jid])])
        return self.data.xquat[bid].copy()

    def object_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        return self.body_pose(name)

    def object_height(self, name: str) -> float:
        return float(self.body_pose(name)[0][2])

    def gripper_gap(self) -> float:
        """Distance between the two fingertip sites (m)."""
        return float(np.linalg.norm(self.site_xpos("fingertip_left")
                                    - self.site_xpos("fingertip_right")))

    def finger_midpoint(self) -> np.ndarray:
        return 0.5 * (self.site_xpos("fingertip_left")
                      + self.site_xpos("fingertip_right"))

    @property
    def arm_joint_positions(self) -> np.ndarray:
        return self.data.qpos[self._arm_q].copy()

    @property
    def arm_joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self._arm_d].copy()

    @property
    def arm_joint_torques(self) -> np.ndarray:
        """Torque actually delivered per arm channel (N m), not the setpoint."""
        return np.array([self.arm_force(n) for n in self._arm_ctrl_names])

    # -- contact evidence ---------------------------------------------------
    def _body_name(self, geomid: int) -> str:
        bid = int(self.model.geom_bodyid[geomid])
        return self._mj.mj_id2name(self.model, self._mj.mjtObj.mjOBJ_BODY, bid) or ""

    def contacts(self, other: str | None = None) -> list[ContactReport]:
        """Contact reports grouped by the two bodies involved.

        `other` filters to reports whose pair contains that body name.
        Forces are resolved with `mj_contactForce`: normal component first, then
        the two tangential components.
        """
        grouped: dict[tuple[str, str], ContactReport] = {}
        for i, c in enumerate(self.data.contact[:self.data.ncon]):
            b1, b2 = self._body_name(int(c.geom1)), self._body_name(int(c.geom2))
            if other and other not in (b1, b2):
                continue
            f = np.zeros(6)  # mj_contactForce writes (normal, tan1, tan2, ...)
            self._mj.mj_contactForce(self.model, self.data, i, f)
            key = tuple(sorted((b1, b2)))
            rep = grouped.get(key)
            if rep is None:
                grouped[key] = rep = ContactReport(
                    key[0], key[1], 0, 0.0, 0.0, float(c.dist))
            rep.count += 1
            rep.normal_force_n += float(abs(f[0]))
            rep.tangent_force_n += float(np.hypot(f[1], f[2]))
            rep.distance_m = min(rep.distance_m, float(c.dist))
        return list(grouped.values())

    def normal_force_between(self, name_a: str, name_b: str) -> float:
        for rep in self.contacts():
            if name_a in rep.pair and name_b in rep.pair:
                return rep.normal_force_n
        return 0.0

    def object_contact_force(self, object_name: str) -> float:
        """Total normal force currently applied to a free object by anything."""
        return float(sum(r.normal_force_n for r in self.contacts()
                         if object_name in r.pair))

    def finger_contact_force(self, object_name: str | None = None) -> float:
        """Normal force between either finger and the object (N).

        This is the simulated grasp force.  With `object_name=None` it measures
        finger-to-anything contact, which is how an empty-close is detected.
        """
        total = 0.0
        for rep in self.contacts():
            if not any(f in rep.pair for f in FINGER_BODIES):
                continue
            if object_name is not None and object_name not in rep.pair:
                continue
            total += rep.normal_force_n
        return total

    def touching(self, object_name: str) -> bool:
        return any(object_name in r.pair for r in self.contacts())

    def finger_touching(self, object_name: str | None = None) -> bool:
        for rep in self.contacts():
            if any(f in rep.pair for f in FINGER_BODIES):
                if object_name is None or object_name in rep.pair:
                    return True
        return False

    def max_penetration_m(self, restrict: tuple = (), exclude: tuple = ()) -> float:
        """Deepest contact penetration (positive number of metres).

        `restrict` keeps pairs touching one of those bodies, `exclude` drops pairs
        involving any of them; they are not redundant.  A pre-flight check that
        counts the fingers inside the object it is about to grasp is measuring the
        grasp rather than the scene, and would refuse every plan that ends with a
        squeeze.

        Used as the stability check: a quasi-static grasp should stay within a
        fraction of the contact softness, and nothing should tunnel through.
        """
        worst = 0.0
        for c in self.data.contact[:self.data.ncon]:
            pair = (self._body_name(int(c.geom1)),
                    self._body_name(int(c.geom2)))
            if restrict and not any(r in pair for r in restrict):
                continue
            if exclude and any(e in pair for e in exclude):
                continue
            worst = max(worst, -float(c.dist))
        return worst

    def _vertical_extent(self, gi) -> float:
        """Half-height of a geom in the world z direction, exactly for the
        primitive types this model uses."""
        m = self.model
        size = m.geom_size[gi]
        kind = int(m.geom_type[gi])
        if kind == int(self._mj.mjtGeom.mjGEOM_BOX):
            r = self.data.geom_xmat[gi].reshape(3, 3)
            return float(abs(r[2, 0]) * size[0] + abs(r[2, 1]) * size[1]
                         + abs(r[2, 2]) * size[2])
        if kind in (int(self._mj.mjtGeom.mjGEOM_CYLINDER),
                    int(self._mj.mjtGeom.mjGEOM_CAPSULE)):
            r = self.data.geom_xmat[gi].reshape(3, 3)
            radius, half_len = float(size[0]), float(size[2])
            lateral = math.hypot(r[2, 0], r[2, 1])
            return float(lateral * radius + abs(r[2, 2]) * half_len)
        if kind == int(self._mj.mjtGeom.mjGEOM_SPHERE):
            return float(size[0])
        return float(np.max(size))

    #: Bodies that make up the tool, used for the reach-past-the-fingertips
    #: measurement below.  Link7 is included because the gripper is mounted on it
    #: and its collision box is part of what has to clear the work surface.
    TOOL_BODIES = ("Link7", "prototype_adapter", "rebot_end_link",
                   "rebot_finger_left_link", "rebot_finger_right_link")

    def robot_dip_below_tip_m(self, q, *, open_fingers_m: float = 0.026,
                              bodies: tuple = TOOL_BODIES) -> float:
        """How far the TOOL hangs BELOW its own fingertip plane at `q`.

        The number that decides whether a top-down grasp of an object lying on a
        flat surface is possible at all: with this side-mounted gripper the CAD
        envelope of the gripper body reaches past the fingertip sites, so placing
        the tip plane on the object centre drives the body into the surface before
        the pads reach the object.  Measured from the compiled collision geoms, so
        it follows any change to the proxies instead of being a constant someone
        once typed in.  Positive means the robot extends below the tips.
        """
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        try:
            self.set_arm_positions(q)
            self.set_gripper_position(open_fingers_m)
            self._mj.mj_forward(self.model, self.data)
            tip = 0.5 * (self.site_xpos("fingertip_left")[2]
                         + self.site_xpos("fingertip_right")[2])
            object_ids = set()
            for name in self.object_names:
                bid = self._mj.mj_name2id(self.model,
                                          self._mj.mjtObj.mjOBJ_BODY, name)
                object_ids.add(bid)
            lowest = float("inf")
            for gi in range(self.model.ngeom):
                if int(self.model.geom_group[gi]) != 3:
                    continue
                bid = int(self.model.geom_bodyid[gi])
                if bid == 0 or bid in object_ids:
                    continue          # world (floor, table) and experiment objects
                lowest = min(lowest, float(self.data.geom_xpos[gi][2])
                             - self._vertical_extent(gi))
            return float(tip - lowest) if np.isfinite(lowest) else float("nan")
        finally:
            self.data.qpos[:] = qpos
            self.data.qvel[:] = qvel
            self._mj.mj_forward(self.model, self.data)

    def penetration_at(self, q, *, open_fingers_m: float = 0.026,
                       against_objects: bool = False) -> float:
        """Worst contact penetration (m) with the arm held at `q`, jaws open.

        A pre-flight check for a planned pose: joint values can satisfy every
        limit and still bury the gripper body in the work surface.  The jaws are
        set to the approach opening first, because checking a closed gripper
        against an object it has not reached yet reports the fingers inside the
        block, not a real collision.  State is saved and restored, so this
        neither advances nor disturbs the experiment.

        The experiment objects are left out by default, for the same reason: a
        plan that ends with the pads around a 120 mm bar must be judged against
        the bench and the arm, not against the object it is going to squeeze.
        Pass `against_objects=True` to include them.
        """
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        try:
            self.set_arm_positions(q)
            self.set_gripper_position(open_fingers_m)
            self._mj.mj_forward(self.model, self.data)
            return self.max_penetration_m(
                exclude=() if against_objects else tuple(self.object_names))
        finally:
            self.data.qpos[:] = qpos
            self.data.qvel[:] = qvel
            self._mj.mj_forward(self.model, self.data)

    def any_nonfinite(self) -> bool:
        return bool(not (np.all(np.isfinite(self.data.qpos))
                         and np.all(np.isfinite(self.data.qvel))))


def _box_inertia(mass: float, half: list) -> tuple:
    return tuple(mass * (s * s + t * t) / 3.0
                 for s, t in ((half[1], half[2]), (half[0], half[2]),
                              (half[0], half[1])))


class SceneArmBackend(ArmBackend):
    """Seven-joint view of a :class:`GripperScene`, through the existing ABC.

    The gripper is deliberately invisible here: the frozen `ArmCommand` is a
    fixed seven-joint frame, so the fingers can only be moved by `SceneGripper`
    on their own channel.
    """

    def __init__(self, scene: GripperScene, *, calibration_ok: bool = True,
                 feedback_age_s: float = 0.0, voltage_v: float = 48.0) -> None:
        self.scene = scene
        self._connected = False
        self._calibration_ok = calibration_ok
        self._feedback_age_s = feedback_age_s
        self._voltage_v = voltage_v
        self._mode = np.full(NUM_JOINTS, ControlMode.STOP, dtype=np.int64)
        self.command_count = 0

    @property
    def name(self) -> str:
        return "gripper-scene-arm"

    @property
    def is_real_hardware(self) -> bool:
        return False

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def has_valid_calibration(self) -> bool:
        """A method, not a property: `ArmBackend` declares it that way."""
        return self._calibration_ok

    def read_state(self) -> RobotState:
        st = RobotState()
        st.connected = self._connected
        st.stamp_ns = time.monotonic_ns() - int(self._feedback_age_s * 1e9)
        st.q = self.scene.arm_joint_positions
        st.dq = self.scene.arm_joint_velocities
        st.tau = self.scene.arm_joint_torques
        st.mode = self._mode.copy()
        st.valid = np.ones(NUM_JOINTS, dtype=bool)
        st.error_code = np.zeros(NUM_JOINTS, dtype=np.uint32)
        st.voltage = np.full(NUM_JOINTS, self._voltage_v)
        st.temperature_c = np.full(NUM_JOINTS, 35.0)
        return st

    def write_command(self, cmd: ArmCommand) -> None:
        if not self._connected:
            raise RuntimeError("SceneArmBackend: refused, not connected")
        if not cmd.all_finite():
            raise ValueError("SceneArmBackend: refused, non-finite command frame")
        self.scene.write_arm_command(cmd)
        self._mode = np.asarray(cmd.mode, dtype=np.int64).copy()
        self.command_count += 1

    def stop(self) -> None:
        """Mode 0 on every joint: the arm is left to gravity, not braked."""
        self.scene.write_arm_command(ArmCommand.stop())
        self._mode = np.full(NUM_JOINTS, ControlMode.STOP, dtype=np.int64)

    def advance(self, dt_s: float) -> None:
        """Only the experiment harness drives time, never a backend caller."""
        self.scene.step(dt_s)


class SceneGripper(Gripper):
    """Gripper view of the same scene, on the normalised [0, 1] channel.

    Normalisation lives here, at the adapter edge: `position` 1.0 is fully open
    (driven finger at +50 mm), 0.0 fully closed; the plant keeps its metres.
    Commands are validated the way `MockGripper` validates them - refused, not
    clipped - and `CURRENT` mode is refused outright because there is no motor
    current model behind this channel.
    """

    def __init__(self, scene: GripperScene,
                 limits: GripperLimits | None = None) -> None:
        self.scene = scene
        self._connected = False
        self._limits = limits or GripperLimits(
            position_min=0.0, position_max=1.0,
            max_current=0.0, max_velocity=0.0, default_current_limit=0.0,
            source=("normalised opening of the simulated finger: 1.0 open = "
                    "+-50 mm travel, 0.0 closed. NOT DM4310 data."))
        self._state = GripperState(position=1.0)
        self._stamp_ns = 0
        self.command_count = 0

    @property
    def name(self) -> str:
        return "gripper-scene"

    @property
    def is_real_hardware(self) -> bool:
        return False

    def connect(self) -> None:
        self._connected = True
        self._refresh()
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

    def _refresh(self) -> None:
        self._state.position = self.scene.gripper_metres() / FINGER_TRAVEL_M
        self._state.velocity = self.scene.gripper_rate()
        self._state.stamp_ns = time.monotonic_ns()

    def state(self) -> GripperState:
        self._refresh()
        return self._state

    def drive_force_n(self) -> float:
        """Force the servo channel is applying - a simulation quantity only."""
        return self.scene.gripper_channel_force_n()

    def write_command(self, cmd: GripperCommand) -> None:
        if not self._connected:
            raise RuntimeError("SceneGripper: refused, not connected")
        limits = self._limits
        if cmd.mode == GripperCommandMode.POSITION:
            p = float(cmd.position)
            if not np.isfinite(p):
                raise ValueError("refused: position is not finite")
            if not (limits.position_min <= p <= limits.position_max):
                raise ValueError(
                    f"refused: position {p} outside normalized range "
                    f"[{limits.position_min}, {limits.position_max}]")
            self.scene.set_gripper_metres(p * FINGER_TRAVEL_M)
        elif cmd.mode == GripperCommandMode.STOP:
            # Hold where the finger is; a stop must not add grip force.
            self.scene.set_gripper_metres(self.scene.gripper_metres())
        else:
            raise NotImplementedError(
                "SceneGripper: CURRENT mode is not modelled. The simulated "
                "channel is a position servo with a force ceiling; there is no "
                "motor current model, so torque-mode commands are refused "
                "rather than faked (HARDWARE_UNVERIFIED for the DM4310 too).")
        self.command_count += 1

    def stop(self) -> None:
        self.scene.set_gripper_metres(self.scene.gripper_metres())
