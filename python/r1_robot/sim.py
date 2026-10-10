# ---------------------------------------------------------------------------
# r1_robot/sim.py
#
# MuJoCo-backed simulation backend for the R1-7A.
#
# DESIGN
#   * `SimArm` implements the same `ArmBackend` interface as `MockArm`, so the
#     whole Python stack (R1Robot + SafetyMonitor + trajectories) runs against
#     it unchanged:  MockArm -> quick logic tests, SimArm -> physics.
#   * The MuJoCo model is *generated* from the official URDF at runtime, so it
#     cannot silently drift from the hardware model.  A copy can be written to
#     `sim/models/r1_7a_sim.xml` for inspection (`write_sim_model()`).
#   * The official URDF and meshes are only READ, never modified.
#   * MuJoCo is an optional dependency: it is imported lazily so that the core
#     `import r1_robot` keeps working on a machine without it.
#
# CONTROL MODEL
#   Each joint gets one `<general>` actuator whose force is
#       f = ctrl + biasprm[1]*qpos + biasprm[2]*qvel
#   SimArm sets, per command and per joint:
#       biasprm[1] = -kp,  biasprm[2] = -kd,  ctrl = kp*q_des + kd*dq_des + tau_ff
#   i.e. f = kp*(q_des - q) + kd*(dq_des - qvel) + tau_ff, the same impedance
#   law MockArm integrates.  A mode-0 (stop) command sets the gains to zero and
#   ctrl to zero, which leaves the joint free (motor output disabled).
# ---------------------------------------------------------------------------
from __future__ import annotations

import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .backends import ArmBackend
from .kinematics import (
    LINK_NAMES,
    OFFICIAL_MESH_DIR,
    REPO_ROOT,
    official_urdf_path,
)
from .types import JOINT_NAMES, NUM_JOINTS, ArmCommand, ControlMode, RobotState

SIM_DIR = REPO_ROOT / "sim"
SIM_MODEL_PATH = SIM_DIR / "models" / "r1_7a_sim.xml"

#: Supply voltage reported by the simulation [V] (matches MockArm's default).
SIM_SUPPLY_VOLTAGE = 48.0
#: Actuator control range.  Wide, because `ctrl` carries kp*q_des + tau_ff.
_CTRL_RANGE = 1.0e5

# ---------------------------------------------------------------------------
# Collision modes (Task 5).
#
# IMPORTANT: in MuJoCo 3.x the contact masks MUST be authored in the XML.  A
# runtime write to `model.geom_contype` after `from_xml_string` is silently
# ignored (the model is compiled on load), which is why the old `enable_collision`
# flag was a no-op and no collision was ever actually detected here.
#
# Modes:
#   * "none"        - no geom collides.  For PURE KINEMATICS / FK cross-checks.
#                     MUST NOT be used for contact or grasp experiments: it hides
#                     real collisions (a test asserts this explicitly).
#   * "environment" - arm links collide with the world (floor + optional table)
#                     but NOT with each other (arm-arm bitmask disabled).  This
#                     reliably detects desktop / fixture collisions and is immune
#                     to the convex-hull false positives of the STL meshes.
#   * "self"        - arm links ALSO collide with each other.  The meshes are
#                     CONCAVE, so their convex hulls overlap for geometrically
#                     adjacent pairs regardless of pose; those pairs (and the one
#                     concentric artifact Link5<->Link7) are removed with explicit
#                     <contact><exclude> entries so a nominal arm reads 0 contacts
#                     and a genuine fold (wrist onto elbow/base) is detected.  The
#                     hull approximation makes this CONSERVATIVE: it can miss thin
#                     interpenetrations and over-report near-adjacent ones.
# ---------------------------------------------------------------------------
COLLISION_MODES = ("none", "environment", "self")

#: Body-name pairs excluded from self-collision (jointless/adjacent + the one
#: concentric hull artifact).  Everything else that overlaps is a real contact.
SELF_COLLISION_EXCLUDES: tuple[tuple[str, str], ...] = (
    ("base_link", "Link1"),
    ("Link1", "Link2"),
    ("Link2", "Link3"),
    ("Link3", "Link4"),
    ("Link4", "Link5"),
    ("Link5", "Link6"),
    ("Link6", "Link7"),
    ("Link5", "Link7"),
)

#: A tabletop used for environment-collision validation (world-fixed body).
_TABLE_BOX = (
    '<body name="table_static">\n'
    '      <geom name="tabletop" type="box" size="0.2 0.2 0.02" '
    'pos="0.25 0 0.22" contype="2" conaffinity="1" rgba="0.6 0.4 0.2 1"/>\n'
    "    </body>\n    "
)


def _collision_class_geom(mode: str) -> str:
    """contype/conaffinity for the 'collision' default class, per mode."""
    if mode == "none":
        return '<geom group="3" contype="0" conaffinity="0"/>'
    if mode == "environment":
        # collides with the world (bit 1) only, never with other arm links.
        return '<geom group="3" contype="1" conaffinity="2"/>'
    if mode == "self":
        return '<geom group="3" contype="1" conaffinity="1"/>'
    raise ValueError(f"unknown collision_mode {mode!r} (expected one of {COLLISION_MODES})")


def _floor_geom(mode: str) -> str:
    """The floor plane: participates in contact only outside kinematics mode."""
    masks = "" if mode == "none" else ' contype="2" conaffinity="1"'
    return (
        '<geom name="floor" type="plane" size="2 2 0.1" pos="0 0 -0.02" '
        'rgba="0.35 0.35 0.4 1" condim="3"' + masks + "/>"
    )


def _contact_block(mode: str) -> str:
    if mode != "self":
        return ""
    excludes = "".join(
        f'    <exclude body1="{a}" body2="{b}"/>\n'
        for a, b in SELF_COLLISION_EXCLUDES
    )
    return f"\n  <contact>\n{excludes}  </contact>\n"


# ---------------------------------------------------------------------------
# Dynamics-parameter provenance (Task 5).
#
# Every dynamics quantity is tagged so a reader knows which numbers are traceable
# to the official model and which are assumptions pending real-arm identification.
# ---------------------------------------------------------------------------
DYNAMICS_PROVENANCE: dict[str, str] = {
    "link_mass": "official (URDF inertial <mass>)",
    "link_inertia": "official (URDF inertial <inertia>, rotated into body frame)",
    "joint_range": "official (URDF <limit lower/upper>)",
    "joint_axis_origin": "official (URDF <origin>/<axis>)",
    "gravity": "official constant 9.81 m/s^2",
    "joint_damping": "simulation assumption (0.2 Nm/(rad/s)); to be identified on hardware [HARDWARE_UNVERIFIED]",
    "joint_armature": "simulation assumption (0.01 kg m^2); to be identified on hardware [HARDWARE_UNVERIFIED]",
    "friction": "simulation assumption (1 0.005 0.0001); to be identified on hardware [HARDWARE_UNVERIFIED]",
    "gear_ratio_and_stiffness": "to be identified on hardware - the IM6014 ratio is inside the motor firmware, not the SDK [HARDWARE_UNVERIFIED]",
    "actuator_torque_limit": "to be identified on hardware - ctrlrange is intentionally wide here [HARDWARE_UNVERIFIED]",
    "collision_geometry": "simulation assumption - convex hull of the visual STL; conservative for self-collision",
}


def dynamics_provenance() -> dict[str, str]:
    """Return the provenance tag for each dynamics parameter group."""
    return dict(DYNAMICS_PROVENANCE)



# ---------------------------------------------------------------------------
# URDF -> MJCF
# ---------------------------------------------------------------------------
def _rot_from_rpy(rpy: np.ndarray) -> np.ndarray:
    """URDF rpy (R = Rz(y) Ry(p) Rx(r)) -> 3x3 rotation matrix."""
    import pinocchio as pin

    return pin.rpy.rpyToMatrix(np.asarray(rpy, dtype=np.float64))


def _quat_from_rpy(rpy: np.ndarray) -> tuple[float, float, float, float]:
    """URDF rpy -> MuJoCo quat (w, x, y, z)."""
    import pinocchio as pin

    q = pin.Quaternion(_rot_from_rpy(rpy))
    return (float(q.w), float(q.x), float(q.y), float(q.z))


def _fmt(values) -> str:
    return " ".join(f"{float(v):.10g}" for v in values)


def generate_mjcf(
    meshdir: str | Path | None = None,
    collision_mode: str = "none",
    add_table: bool = False,
) -> str:
    """Build a MuJoCo MJCF string from the official R1-7A URDF.

    ``meshdir`` is where the STL files live (default: the SDK mesh directory).
    ``collision_mode`` is one of COLLISION_MODES ("none" / "environment" /
    "self") and must be authored here - see the collision-mode note above for
    why a runtime mask write cannot enable contacts in MuJoCo 3.x.
    ``add_table`` places a fixed tabletop used for environment-collision tests.
    """
    if collision_mode not in COLLISION_MODES:
        raise ValueError(
            f"unknown collision_mode {collision_mode!r} (expected one of {COLLISION_MODES})"
        )
    urdf = official_urdf_path()
    meshdir = Path(meshdir) if meshdir is not None else OFFICIAL_MESH_DIR
    root = ET.parse(str(urdf)).getroot()

    links = {link.get("name"): link for link in root.findall("link")}
    joints = root.findall("joint")
    child_of: dict[str, ET.Element] = {}
    for joint in joints:
        child_of[joint.find("child").get("link")] = joint

    # Collect the mesh files referenced anywhere (one <asset> entry each).
    mesh_files: list[str] = []
    for link in links.values():
        for geom in list(link.findall("visual")) + list(link.findall("collision")):
            node = geom.find("geometry/mesh")
            if node is not None:
                name = Path(node.get("filename", "")).name
                if name not in mesh_files:
                    mesh_files.append(name)

    def origin(node: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
        """Read an ``<origin xyz=... rpy=.../>`` element (URDF attribute form)."""
        if node is None:
            return np.zeros(3), np.zeros(3)
        pos = np.array([float(v) for v in (node.get("xyz", "0 0 0")).split()])
        rpy = np.array([float(v) for v in (node.get("rpy", "0 0 0")).split()])
        return pos, rpy

    def inertial(node: ET.Element | None, pad: str) -> str:
        if node is None:
            return ""
        mass = float(node.find("mass").get("value"))
        it = node.find("inertia")
        ixx, ixy, ixz = (float(it.get(k)) for k in ("ixx", "ixy", "ixz"))
        iyy, iyz, izz = (float(it.get(k)) for k in ("iyy", "iyz", "izz"))
        pos, rpy = origin(node.find("origin"))
        # The URDF inertia is given in the inertial frame.  MuJoCo's
        # `fullinertia` is expressed in the *body* frame and cannot be combined
        # with `quat`, so rotate the tensor into the body frame: I_b = R I R^T.
        inertia = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=np.float64
        )
        rot = _rot_from_rpy(rpy)
        inertia = rot @ inertia @ rot.T
        return (
            f'{pad}<inertial pos="{_fmt(pos)}" '
            f'mass="{mass:.10g}" fullinertia="{inertia[0,0]:.10g} {inertia[1,1]:.10g} '
            f'{inertia[2,2]:.10g} {inertia[0,1]:.10g} {inertia[0,2]:.10g} '
            f'{inertia[1,2]:.10g}"/>\n'
        )

    def geoms(link: ET.Element, link_name: str, pad: str) -> str:
        out = ""
        for kind in ("visual", "collision"):
            for geom in link.findall(kind):
                mesh = geom.find("geometry/mesh")
                if mesh is None:
                    continue
                mesh_name = Path(mesh.get("filename", "")).name
                pos, rpy = origin(geom.find("origin"))
                w, x, y, z = _quat_from_rpy(rpy)
                out += (
                    f'{pad}<geom name="{link_name}_{kind}" type="mesh" mesh="{Path(mesh_name).stem}" '
                    f'class="{kind}" pos="{_fmt(pos)}" '
                    f'quat="{w:.10g} {x:.10g} {y:.10g} {z:.10g}"/>\n'
                )
        return out

    def body_descendants(parent_link: str, indent: int) -> str:
        out = ""
        for child, joint in child_of.items():
            if joint.find("parent").get("link") == parent_link:
                out += body(child, indent)
        return out

    def body(link_name: str, indent: int) -> str:
        link = links[link_name]
        joint = child_of.get(link_name)
        pad = "  " * indent
        if joint is None:
            # Root link: fixed to the world at its URDF frame.
            header = f'{pad}<body name="{link_name}">\n'
            tail = f"{pad}</body>\n"
            return (
                header
                + inertial(link.find("inertial"), pad + "  ")
                + geoms(link, link_name, pad + "  ")
                + body_descendants(link_name, indent + 1)
                + tail
            )

        pos, rpy = origin(joint.find("origin"))
        axis = joint.find("axis")
        axis_vec = np.array(
            [float(v) for v in (axis.get("xyz", "0 0 1") if axis is not None else "0 0 1").split()]
        )
        lim = joint.find("limit")
        low = float(lim.get("lower", "0")) if lim is not None else 0.0
        high = float(lim.get("upper", "0")) if lim is not None else 0.0
        w, x, y, z = _quat_from_rpy(rpy)
        jname = joint.get("name")
        header = (
            f'{pad}<body name="{link_name}" pos="{_fmt(pos)}" '
            f'quat="{w:.10g} {x:.10g} {y:.10g} {z:.10g}">\n'
            f'{pad}  <joint name="{jname}" type="hinge" pos="0 0 0" '
            f'axis="{_fmt(axis_vec)}" range="{low:.10g} {high:.10g}" limited="true"/>\n'
        )
        tail = f"{pad}</body>\n"
        return (
            header
            + inertial(link.find("inertial"), pad + "  ")
            + geoms(link, link_name, pad + "  ")
            + body_descendants(link_name, indent + 1)
            + tail
        )

    asset = "".join(
        f'    <mesh name="{Path(f).stem}" file="{f}"/>\n' for f in mesh_files
    )
    actuators = "".join(
        f'    <general name="{JOINT_NAMES[i]}_ctrl" joint="{JOINT_NAMES[i]}" '
        f'gaintype="fixed" gainprm="1 0 0" biastype="affine" biasprm="0 0 0" '
        f'ctrllimited="true" ctrlrange="{-_CTRL_RANGE:.0f} {_CTRL_RANGE:.0f}"/>\n'
        for i in range(NUM_JOINTS)
    )

    collision_geom = _collision_class_geom(collision_mode)
    floor_geom = _floor_geom(collision_mode)
    table_geom = f"    {_TABLE_BOX}" if add_table else ""
    robot_body = body(LINK_NAMES[0], 2)
    contact_xml = _contact_block(collision_mode)

    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<mujoco model="R1_Robotic_Arm_sim">\n'
        "  <!-- GENERATED from the official URDF by r1_robot.sim.generate_mjcf().\n"
        "       Do not edit by hand; regenerate instead. The official model is\n"
        "       read-only. -->\n"
        f'  <compiler angle="radian" autolimits="true" meshdir="{Path(meshdir).as_posix()}" '
        'boundmass="0" boundinertia="0"/>\n'
        '  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>\n'
        "  <default>\n"
        '    <joint damping="0.2" armature="0.01" frictionloss="0.0"/>\n'
        '    <geom contype="0" conaffinity="0" condim="3" friction="1 0.005 0.0001"/>\n'
        '    <default class="visual">\n'
        '      <geom group="2" contype="0" conaffinity="0"/>\n'
        "    </default>\n"
        '    <default class="collision">\n'
        f"      {collision_geom}\n"
        "    </default>\n"
        "  </default>\n"
        f"  <asset>\n{asset}  </asset>\n"
        "  <worldbody>\n"
        '    <light name="key" pos="0.5 -0.5 1.5" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>\n'
        f"    {floor_geom}\n"
        f"    {table_geom}{robot_body}"
        "  </worldbody>\n"
        f"{contact_xml}"
        f"  <actuator>\n{actuators}  </actuator>\n"
        "</mujoco>\n"
    )


def write_sim_model(path: str | Path | None = None, force: bool = True) -> Path:
    """Write the generated MJCF (with a relative meshdir) to disk."""
    target = Path(path) if path is not None else SIM_MODEL_PATH
    rel = Path(
        __import__("os").path.relpath(OFFICIAL_MESH_DIR, start=target.parent)
    ).as_posix()
    target.parent.mkdir(parents=True, exist_ok=True)
    if force or not target.exists():
        target.write_text(generate_mjcf(rel), encoding="utf-8")
    return target


# ---------------------------------------------------------------------------
# SimArm
# ---------------------------------------------------------------------------
@dataclass
class SimArmConfig:
    """Configuration for the MuJoCo backend."""

    #: MJCF/URDF to load.  ``None`` -> generated in memory from the official URDF.
    model_path: Path | None = None
    #: Seconds of simulated time per physics step.
    timestep: float = 0.002
    #: Gravity vector [m/s^2]; zero disables gravity (useful for FK-only tests).
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)
    #: Contact handling, authored into the generated model (see COLLISION_MODES):
    #:   "none"        - pure kinematics, no contact (NOT for grasp experiments)
    #:   "environment" - arm collides with floor / table, never with itself
    #:   "self"        - arm also self-collides (hull meshes, adjacent pairs
    #:                   excluded so a nominal arm reads zero contacts)
    collision_mode: str = "none"
    #: Place a fixed tabletop in front of the arm for environment-collision tests.
    add_table: bool = False
    #: Reported by has_valid_calibration(); False exercises the refusal path.
    calibration_ok: bool = True
    #: Artificial feedback age [s] reported in RobotState.stamp_ns.
    feedback_delay_s: float = 0.0
    #: Verify FK against MuJoCo at connect() time (cheap, catches model drift).
    verify_model_on_connect: bool = True


class SimArm(ArmBackend):
    """MuJoCo physics backend implementing the :class:`ArmBackend` interface."""

    def __init__(self, config: SimArmConfig | None = None) -> None:
        import mujoco  # lazy: optional dependency

        self._mj = mujoco
        self.config = config or SimArmConfig()

        if self.config.model_path is not None:
            self.model = mujoco.MjModel.from_xml_path(str(self.config.model_path))
        else:
            # Collision masks and contact filters MUST be authored in the XML: a
            # runtime write to geom_contype is ignored once the model is compiled.
            self.model = mujoco.MjModel.from_xml_string(
                generate_mjcf(
                    OFFICIAL_MESH_DIR,
                    collision_mode=self.config.collision_mode,
                    add_table=self.config.add_table,
                )
            )
        self.model.opt.timestep = float(self.config.timestep)
        self.model.opt.gravity[:] = np.asarray(self.config.gravity, dtype=np.float64)

        self.data = mujoco.MjData(self.model)

        # Map joint names -> qpos/actuator addresses (robust to model reorder).
        self._qpos_adr = np.array(
            [self.model.jnt_qposadr[self.model.joint(n).id] for n in JOINT_NAMES]
        )
        self._dof_adr = np.array(
            [self.model.jnt_dofadr[self.model.joint(n).id] for n in JOINT_NAMES]
        )
        try:
            self._act_ids = np.array(
                [self.model.actuator(f"{n}_ctrl").id for n in JOINT_NAMES]
            )
        except KeyError as exc:  # pragma: no cover - model generator bug
            raise RuntimeError(f"sim model has no actuator for {exc}") from exc

        self._connected = False
        self._cmd = ArmCommand.stop()
        self.command_count = 0
        self._kin = None
        mujoco.mj_forward(self.model, self.data)

    # -- backend interface --------------------------------------------------
    @property
    def name(self) -> str:
        return "sim"

    @property
    def is_real_hardware(self) -> bool:
        return False

    def connect(self) -> None:
        # Start from a safe, in-limits pose with all joints limp.
        self.set_joint_positions(np.zeros(NUM_JOINTS))
        self._apply(ArmCommand.stop())
        self._connected = True
        self.command_count = 0
        if self.config.verify_model_on_connect:
            self.verify_against_model()

    def disconnect(self) -> None:
        self._connected = False
        self._apply(ArmCommand.stop())

    @property
    def is_connected(self) -> bool:
        return self._connected

    def has_valid_calibration(self) -> bool:
        return bool(self.config.calibration_ok)

    def read_state(self) -> RobotState:
        now = time.monotonic_ns()
        delay_ns = int(max(0.0, self.config.feedback_delay_s) * 1e9)
        stamp = now - delay_ns if now > delay_ns else 0

        state = RobotState(connected=self._connected, stamp_ns=stamp)
        if self._connected:
            state.q = self.data.qpos[self._qpos_adr].copy()
            state.dq = self.data.qvel[self._dof_adr].copy()
            state.tau = self.data.actuator_force[self._act_ids].copy()
            state.mode = self._cmd.mode.copy()
            state.valid[:] = True
        else:
            state.valid[:] = False
        state.voltage[:] = SIM_SUPPLY_VOLTAGE
        state.temperature_c[:] = 30.0
        return state

    def write_command(self, cmd: ArmCommand) -> None:
        if not self._connected:
            raise RuntimeError("SimArm: not connected")
        self._apply(cmd)
        self.command_count += 1

    def stop(self) -> None:
        if self._connected:
            self.write_command(ArmCommand.stop())

    # -- physics ------------------------------------------------------------
    def advance(self, dt_s: float) -> None:
        """Step the physics forward by ``dt_s`` seconds of simulated time."""
        if dt_s <= 0.0 or not self._connected:
            return
        step = self.model.opt.timestep
        substeps = max(1, int(np.ceil(dt_s / step)))
        h = dt_s / substeps
        saved = self.model.opt.timestep
        self.model.opt.timestep = h
        try:
            for _ in range(substeps):
                self._mj.mj_step(self.model, self.data)
        finally:
            self.model.opt.timestep = saved

    # -- contact inspection -------------------------------------------------
    def contact_count(self) -> int:
        """Number of active contact pairs after the latest forward/step.

        Call ``set_joint_positions`` (which runs mj_forward) first to refresh.
        """
        return int(self.data.ncon)

    def colliding_geoms(self) -> set[tuple[str, str]]:
        """Sorted (geom_a, geom_b) name pairs currently in contact.

        Geom names carry the ``_visual`` / ``_collision`` / ``tabletop`` / ``floor``
        suffix, so a caller can tell an arm-arm contact from an arm-table contact.
        """
        pairs: set[tuple[str, str]] = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            a = self._mj.mj_id2name(
                self.model, self._mj.mjtObj.mjOBJ_GEOM, c.geom1
            )
            b = self._mj.mj_id2name(
                self.model, self._mj.mjtObj.mjOBJ_GEOM, c.geom2
            )
            pairs.add(tuple(sorted((a, b))))
        return pairs

    def self_contact_pairs(self) -> set[tuple[str, str]]:
        """Contact pairs where BOTH geoms are arm collision geoms."""
        return {
            pair
            for pair in self.colliding_geoms()
            if all(g.endswith("_collision") for g in pair)
        }

    @property
    def simulated_time_s(self) -> float:
        return float(self.data.time)

    @property
    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self._qpos_adr].copy()

    @property
    def joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self._dof_adr].copy()

    @property
    def joint_torques(self) -> np.ndarray:
        return self.data.actuator_force[self._act_ids].copy()

    def set_joint_positions(self, q) -> None:
        q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS)
        self.data.qpos[self._qpos_adr] = q
        self.data.qvel[self._dof_adr] = 0.0
        self._mj.mj_forward(self.model, self.data)

    def end_effector_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(position, quat[w,x,y,z]) of Link7 from MuJoCo; cross-checks Pinocchio."""
        body = self.model.body("Link7").id
        return self.data.xpos[body].copy(), self.data.xquat[body].copy()

    # -- internals ----------------------------------------------------------
    def _apply(self, cmd: ArmCommand) -> None:
        self._cmd = cmd
        bias = self.model.actuator_biasprm  # (nu, 3)
        ctrl = np.zeros(NUM_JOINTS)
        for i, act in enumerate(self._act_ids):
            if cmd.mode[i] == ControlMode.FOC:
                kp, kd, tau = float(cmd.kp[i]), float(cmd.kd[i]), float(cmd.tau[i])
                bias[act, 1] = -kp
                bias[act, 2] = -kd
                ctrl[i] = kp * float(cmd.q[i]) + kd * float(cmd.dq[i]) + tau
            else:
                bias[act, 1] = 0.0
                bias[act, 2] = 0.0
                ctrl[i] = 0.0
        self.data.ctrl[self._act_ids] = ctrl

    def verify_against_model(self) -> None:
        """Cross-check MuJoCo FK against Pinocchio at a non-trivial pose.

        Raises RuntimeError on mismatch, so a model-generation regression is
        caught the moment a simulation is created.
        """
        from .kinematics import R1Kinematics

        kin = getattr(self, "_kin", None)
        if kin is None:
            kin = R1Kinematics()
            self._kin = kin
        q = np.array([0.3, 0.5, -0.2, 0.4, -0.3, 0.1, 0.2])
        self.set_joint_positions(q)
        p_mj, _ = self.end_effector_pose()
        p_pin = kin.position(q)
        if not np.allclose(p_mj, p_pin, atol=1e-6):
            raise RuntimeError(
                "MuJoCo/Pinocchio FK mismatch for Link7: "
                f"mujoco={p_mj} pinocchio={p_pin}"
            )
