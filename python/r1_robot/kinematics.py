# ---------------------------------------------------------------------------
# r1_robot/kinematics.py
#
# Pinocchio kinematics for the official R1-7A URDF.
#
# WHAT IT PROVIDES
#   * the exact joint/limit/inertia facts of the official URDF, in one place,
#   * forward kinematics (FK) for any link frame,
#   * the 6x7 geometric Jacobian at any link frame,
#   * helpers used by the numerical-consistency test (analytic vs finite diff).
#
# WHY PINOCCHIO
#   The URDF has revolute joints with *unaligned* axes (J1 = (0,0,-1),
#   J4 = (-1,0,0), J6 = (-1,0,0), J7 = (0,0,-1) with a rotated frame), so a
#   hand-written DH table would be error-prone.  Pinocchio parses the same URDF
#   the hardware uses, so the model cannot silently drift from the SDK.
#
# The official URDF is read-only; this module never writes to the SDK.
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .limits import R1_LOWER, R1_MAX_TORQUE, R1_MAX_VELOCITY, R1_UPPER
from .types import JOINT_NAMES, NUM_JOINTS

# r1_robot/python/r1_robot/kinematics.py -> repo root is parents[2].
REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "third_party" / "r1-7a_sdk"
_URDF_REL = (
    "urdf/R1_Robotic_Arm_V1.1-0915/R1_Robotic_Arm_V1.1_0915/"
    "R1_Robotic_Arm/urdf/R1_Robotic_Arm.urdf"
)
OFFICIAL_URDF = SDK_ROOT / _URDF_REL
OFFICIAL_MESH_DIR = OFFICIAL_URDF.parent.parent / "meshes"

#: Link frames, base first.  The arm tip is ``Link7``.
LINK_NAMES: tuple[str, ...] = ("base_link",) + tuple(f"Link{i}" for i in range(1, 8))
END_EFFECTOR_FRAME = "Link7"


def official_urdf_path() -> Path:
    """Absolute path of the official URDF (raises if the submodule is missing)."""
    if not OFFICIAL_URDF.is_file():
        raise FileNotFoundError(
            "official R1-7A URDF not found at "
            f"{OFFICIAL_URDF}\n"
            "Initialise the pinned submodule from the repository root:\n"
            "    git submodule update --init --recursive third_party/r1-7a_sdk"
        )
    return OFFICIAL_URDF


@dataclass(frozen=True)
class UrdfFacts:
    """The URDF facts we can check without a physics engine."""

    joint_names: tuple[str, ...]
    joint_types: tuple[str, ...]
    axes: np.ndarray          # (7, 3)
    lower: np.ndarray         # (7,)
    upper: np.ndarray         # (7,)
    effort: np.ndarray        # (7,)
    velocity: np.ndarray      # (7,)
    masses: np.ndarray        # (7,) link masses (Link1..Link7)
    inertia_matrices: np.ndarray  # (7, 3, 3) link inertia about the COM
    mesh_files: tuple[str, ...]   # (8,) referenced mesh file names
    n_collision: int              # number of <collision> blocks

    @property
    def total_mass(self) -> float:
        return float(self.masses.sum())


def urdf_facts(urdf_path: str | Path | None = None) -> UrdfFacts:
    """Read the URDF directly (no Pinocchio) for model validation."""
    import xml.etree.ElementTree as ET

    path = Path(urdf_path) if urdf_path else official_urdf_path()
    root = ET.parse(str(path)).getroot()

    def _xyz(text: str | None) -> np.ndarray:
        if not text:
            return np.zeros(3)
        return np.array([float(v) for v in text.split()], dtype=np.float64)

    link_mass: dict[str, float] = {}
    link_inertia: dict[str, np.ndarray] = {}
    for link in root.findall("link"):
        name = link.get("name", "")
        inertial = link.find("inertial")
        if inertial is None:
            link_mass[name] = 0.0
            link_inertia[name] = np.zeros((3, 3))
            continue
        link_mass[name] = float(inertial.find("mass").get("value"))
        it = inertial.find("inertia")
        ixx = float(it.get("ixx"))
        ixy = float(it.get("ixy"))
        ixz = float(it.get("ixz"))
        iyy = float(it.get("iyy"))
        iyz = float(it.get("iyz"))
        izz = float(it.get("izz"))
        link_inertia[name] = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
        )

    names, types, axes, lower, upper, effort, velocity = [], [], [], [], [], [], []
    meshes: list[str] = []
    n_collision = 0
    for joint in root.findall("joint"):
        names.append(joint.get("name", ""))
        types.append(joint.get("type", ""))
        axes.append(_xyz(joint.find("axis").get("xyz") if joint.find("axis") is not None else None))
        lim = joint.find("limit")
        lower.append(float(lim.get("lower", "0")))
        upper.append(float(lim.get("upper", "0")))
        effort.append(float(lim.get("effort", "0")))
        velocity.append(float(lim.get("velocity", "0")))
    for link in root.findall("link"):
        for geom in list(link.findall("visual")) + list(link.findall("collision")):
            if geom.tag == "collision":
                n_collision += 1
            mesh = geom.find("geometry/mesh") if geom.find("geometry") is not None else None
            if mesh is not None:
                meshes.append(Path(mesh.get("filename", "")).name)

    # Link1..Link7 masses, in joint order (J_k drives Link_k).
    link_order = [f"Link{i}" for i in range(1, NUM_JOINTS + 1)]
    masses = np.array([link_mass.get(n, 0.0) for n in link_order])
    inertias = np.array([link_inertia.get(n, np.zeros((3, 3))) for n in link_order])

    return UrdfFacts(
        joint_names=tuple(names),
        joint_types=tuple(types),
        axes=np.array(axes),
        lower=np.array(lower),
        upper=np.array(upper),
        effort=np.array(effort),
        velocity=np.array(velocity),
        masses=masses,
        inertia_matrices=inertias,
        mesh_files=tuple(dict.fromkeys(meshes)),
        n_collision=n_collision,
    )


class R1Kinematics:
    """FK / Jacobian wrapper around a Pinocchio model of the official URDF."""

    def __init__(self, urdf_path: str | Path | None = None) -> None:
        import pinocchio as pin  # imported lazily: optional dependency

        self._pin = pin
        self.urdf_path = Path(urdf_path) if urdf_path else official_urdf_path()
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        if self.model.nq != NUM_JOINTS or self.model.nv != NUM_JOINTS:
            raise RuntimeError(
                f"unexpected model size nq={self.model.nq} nv={self.model.nv}; "
                f"expected {NUM_JOINTS} (fixed-base, all-revolute)"
            )
        self.data = self.model.createData()
        self.facts = urdf_facts(self.urdf_path)

    # -- model facts --------------------------------------------------------
    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def nv(self) -> int:
        return self.model.nv

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(self.model.names[i] for i in range(1, self.model.njoints))

    def frame_id(self, frame_name: str = END_EFFECTOR_FRAME) -> int:
        fid = self.model.getFrameId(frame_name)
        if fid >= len(self.model.frames):
            raise ValueError(f"unknown frame {frame_name!r}")
        return fid

    def neutral(self) -> np.ndarray:
        return self._pin.neutral(self.model)

    def configuration_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self.model.lowerPositionLimit.copy(), self.model.upperPositionLimit.copy()

    # -- kinematics ---------------------------------------------------------
    def forward_kinematics(
        self, q, frame_name: str = END_EFFECTOR_FRAME
    ) -> np.ndarray:
        """4x4 homogeneous transform of ``frame_name`` in the world frame."""
        q = self._as_configuration(q)
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id(frame_name)].homogeneous.copy()

    def position(self, q, frame_name: str = END_EFFECTOR_FRAME) -> np.ndarray:
        return self.forward_kinematics(q, frame_name)[:3, 3].copy()

    def jacobian(
        self, q, frame_name: str = END_EFFECTOR_FRAME
    ) -> np.ndarray:
        """6x7 geometric Jacobian at ``frame_name`` (LOCAL_WORLD_ALIGNED)."""
        q = self._as_configuration(q)
        pin = self._pin
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        jac = pin.getFrameJacobian(
            self.model,
            self.data,
            self.frame_id(frame_name),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return jac.copy()

    def gravity_torques(self, q) -> np.ndarray:
        """Joint torques that hold the arm against gravity at configuration q.

        Used as a feed-forward term by the simulation: it removes the
        steady-state droop of a pure PD position controller.
        """
        q = self._as_configuration(q)
        return self._pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    def numerical_jacobian(
        self, q, frame_name: str = END_EFFECTOR_FRAME, eps: float = 1e-6
    ) -> np.ndarray:
        """Finite-difference Jacobian, used to validate ``jacobian()``.

        The result is expressed the same way as ``jacobian()``:
        linear velocity in the world frame and angular velocity such that
        dR/dt = [omega]_x R.
        """
        pin = self._pin
        q = self._as_configuration(q)
        out = np.zeros((6, NUM_JOINTS))
        h0 = self.forward_kinematics(q, frame_name)
        p0, r0 = h0[:3, 3], h0[:3, :3]
        for j in range(NUM_JOINTS):
            dq = q.copy()
            dq[j] += eps
            h1 = self.forward_kinematics(dq, frame_name)
            p1, r1 = h1[:3, 3], h1[:3, :3]
            out[:3, j] = (p1 - p0) / eps
            out[3:, j] = pin.log3(r1 @ r0.T) / eps
        return out

    # -- helpers ------------------------------------------------------------
    def _as_configuration(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(NUM_JOINTS).copy()
        if not np.all(np.isfinite(q)):
            raise ValueError("configuration contains NaN/Inf")
        return q


def joint_limits_are_consistent_with_constants() -> bool:
    """The Python limit constants and the URDF must agree (drift guard)."""
    facts = urdf_facts()
    return (
        np.allclose(facts.lower, R1_LOWER)
        and np.allclose(facts.upper, R1_UPPER)
        and np.allclose(facts.effort, R1_MAX_TORQUE)
        and np.allclose(facts.velocity, R1_MAX_VELOCITY)
        and tuple(facts.joint_names) == JOINT_NAMES
    )
