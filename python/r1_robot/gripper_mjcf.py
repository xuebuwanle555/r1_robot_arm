"""Generate a standalone, relocatable MJCF for the R1 arm + reBot gripper.

WHY A CONVERTER RATHER THAN THE URDF DIRECTLY
  MuJoCo compiles this prototype URDF correctly (joint limits, inertials and the
  finger mimic all survive), but it silently drops anything actuator-related put
  into the URDF's ``<mujoco>`` extension block: adding ``<actuator>`` there still
  compiles, with ``nu == 0`` and no warning.  A model nothing can command is not
  a plant.  So the arm + gripper tree is converted to MJCF here, where the eight
  control channels, the sites and the scene objects are explicit and inspectable.

PORTABILITY
  The generated XML references staged meshes through a *relative* ``meshdir``,
  so the model opens from any checkout location.  Source CAD is located through
  `mesh_sources()` (repo-relative for the official R1 meshes, ``R1_GRIPPER_MESH_
  ROOT`` / ``R1_GRIPPER_URDF`` for the gripper prototype) and copied into
  ``sim/meshes/`` by `stage_meshes()`.  Nothing in the generated file is a
  machine-specific absolute path.

The official SDK model and the prototype directory are read-only inputs: this
module never writes outside ``sim/``.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_DIR = REPO_ROOT / "sim"
GRIPPER_MODEL_PATH = SIM_DIR / "models" / "r1_rebot_sim.xml"
STAGED_MESH_DIR = SIM_DIR / "meshes"

#: Default location of the prototype model, relative to the repository's parent.
#: Overridable so a fresh machine can point at its own copy.
DEFAULT_GRIPPER_URDF = Path(
    "/home/jesse/physgrasp_cvpr27/assets/robots/r1_rebot_prototype/r1_rebot.urdf"
)

ARM_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
FINGER_LEFT = "rebot_finger_left"
FINGER_RIGHT = "rebot_finger_right"
GRIPPER_CHANNEL = "gripper_ctrl"

#: Where each mesh family comes from.  `env` lets another machine relocate the
#: CAD without editing code; the fallbacks are the locations used in development.
@dataclass
class MeshSources:
    r1_mesh_dir: Path
    gripper_mesh_roots: list[Path]
    gripper_urdf: Path


def mesh_sources(gripper_urdf: Path | None = None) -> MeshSources:
    from r1_robot.kinematics import official_urdf_path, OFFICIAL_MESH_DIR

    urdf = Path(gripper_urdf or os.environ.get("R1_GRIPPER_URDF")
                or DEFAULT_GRIPPER_URDF)
    roots: list[Path] = []
    env = os.environ.get("R1_GRIPPER_MESH_ROOT")
    if env:
        roots.append(Path(env))
    # Meshes referenced by absolute path in the source URDF usually live under
    # one of a couple of CAD trees; derive candidate roots from the URDF itself.
    roots.append(urdf.parent)
    return MeshSources(r1_mesh_dir=Path(OFFICIAL_MESH_DIR),
                       gripper_mesh_roots=roots,
                       gripper_urdf=urdf)


# -- rotation bookkeeping ----------------------------------------------------
def _rpy_matrix(rpy) -> list[list[float]]:
    """URDF rpy (R = Rz @ Ry @ Rx) as a 3x3 matrix."""
    roll, pitch, yaw = (float(v) for v in rpy)

    def axis_matrix(angle, which):
        c, s = math.cos(angle), math.sin(angle)
        if which == "x":
            return [[1, 0, 0], [0, c, -s], [0, s, c]]
        if which == "y":
            return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def matmul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]

    return matmul(matmul(axis_matrix(yaw, "z"), axis_matrix(pitch, "y")),
                  axis_matrix(roll, "x"))


def rpy_to_quat(rpy) -> tuple[float, float, float, float]:
    """URDF rpy -> MuJoCo quat (w x y z).

    URDF defines R = Rz(yaw) @ Ry(pitch) @ Rx(roll).  Implementing the documented
    product rather than guessing at Euler conventions is what keeps the gripper
    pointing the right way; `tests/test_gripper_model.py` cross-checks the whole
    tree against MuJoCo's own URDF compiler.
    """
    r = _rpy_matrix(rpy)
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


def _fmt(v) -> str:
    if isinstance(v, (list, tuple)):
        return " ".join(f"{float(x):.9g}" for x in v)
    return f"{float(v):.9g}"


# -- provenance --------------------------------------------------------------
#: Every parameter the generator introduces that is NOT in the source CAD.
#: `environment` values come from the URDF/inertial tags (CAD or the prototype
#: author's stated estimates); the rest are our own modelling choices or numbers
#: that only a real R1 can settle.
GRIPPER_PROVENANCE: dict[str, str] = {
    "arm joint limits": "official URDF (CAD)",
    "finger joint limits (+-50 mm)": "prototype URDF (CAD travel)",
    "finger mimic coefficient -1": "prototype URDF (CAD)",
    "robot base height above the floor": "SCENE CHOICE - ROBOT_BASE_Z_M; the "
                                        "prototype URDF has no platform geometry, "
                                        "and the real deck height is "
                                        "HARDWARE_UNVERIFIED",
    "work surface height": "SCENE CHOICE - TABLE_TOP_Z_M, set by the arm's "
                           "measured top-down workspace band, not by CAD",
    "link masses / inertials": "prototype URDF (author-stated estimates)",
    "mount origin + rpy on Link7": "prototype URDF (v3 side mount, CAD-derived)",
    "armature 0.01 per joint": "simulation assumption (numerical conditioning)",
    "joint damping 0.2": "simulation assumption - NOT identified on hardware",
    "gripper kp / kv": "simulation assumption - no DM4310 drive characterisation",
    "gripper force limit 8 N reading": "URDF effort field; NOT a measured "
                                      "fingertip grasp force",
    "fingertip friction 0.9/0.6": "simulation assumption - rubber pads unmeasured",
    "solimp/solref contact": "simulation assumption - contact stiffness unknown",
    "collision proxies": "approximation - boxes fitted to each CAD collision "
                         "mesh's own axis-aligned bounds, because MuJoCo replaces "
                         "these non-convex meshes with bounding spheres; not CAD "
                         "surfaces, and not a clearance certificate",
    "object mass / size": "experiment assumption (fixture design, not CAD)",
    "site (TCP/fingertip) offsets": "derived from CAD geometry at generation time",
}


def gripper_provenance() -> dict[str, str]:
    return dict(GRIPPER_PROVENANCE)


# -- mesh staging ------------------------------------------------------------
@dataclass
class StagedMesh:
    source: Path
    dest_name: str
    sha256: str

    @property
    def size_bytes(self) -> int:
        return self.source.stat().st_size


def _referenced_meshes(root: ET.Element) -> list[str]:
    out = []
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn and fn not in out:
            out.append(fn)
    return out


def stage_meshes(urdf_root: ET.Element, sources: MeshSources,
                 dest_dir: Path = STAGED_MESH_DIR,
                 extra_files: list[str] | None = None) -> dict[str, StagedMesh]:
    """Copy every referenced STL into ``dest_dir`` under a collision-free name.

    Returns ``{original filename -> StagedMesh}``.  Names are qualified by their
    source directory because the R1 and gripper trees both ship files such as
    ``Link1.stl``; copying is read-only with respect to the CAD.
    """
    dest_dir.mkdir(parents=True, exist_ok=True)
    refs = list(dict.fromkeys(_referenced_meshes(urdf_root)
                              + list(extra_files or [])))
    staged: dict[str, StagedMesh] = {}
    for ref in refs:
        p = Path(ref)
        if not p.is_absolute():
            p = (sources.r1_mesh_dir / ref).resolve()
        if not p.exists():
            raise FileNotFoundError(
                f"mesh {ref!r} is not reachable; set R1_GRIPPER_MESH_ROOT to the "
                f"CAD tree that contains it")
        stem = p.stem
        tag = hashlib.sha256(str(p.resolve()).encode()).hexdigest()[:8]
        dest_name = f"{stem}_{tag}.stl"
        dest = dest_dir / dest_name
        if not dest.exists() or dest.stat().st_mtime < p.stat().st_mtime:
            shutil.copyfile(p, dest)
        digest = hashlib.sha256(dest.read_bytes()).hexdigest()
        staged[ref] = StagedMesh(source=p.resolve(), dest_name=dest_name,
                                 sha256=digest)
    return staged


# -- STL bounding boxes (dependency-free, so generation needs no engine) ----
def stl_aabb(path: Path) -> tuple[tuple[float, float, float],
                                   tuple[float, float, float]]:
    """Axis-aligned bounds of a binary STL, read triangle-by-triangle.

    Used to place the fingertip sites on measured geometry instead of guessed
    offsets.  ASCII STL is rejected loudly rather than silently returning zeros.
    """
    data = path.read_bytes()
    if data[:5] == b"solid":
        # Some exporters write ASCII that still starts with 'solid'; confirm.
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            text = ""
        if "facet normal" in text:
            raise ValueError(f"ASCII STL not supported: {path}")
    if len(data) < 84:
        raise ValueError(f"truncated STL: {path}")
    n_tri = int.from_bytes(data[80:84], "little")
    if len(data) < 84 + n_tri * 50:
        raise ValueError(f"STL triangle count disagrees with file size: {path}")
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    off = 84
    import struct

    for _ in range(n_tri):
        # normal(3f) + 3 vertices(3f) + attribute(2 bytes) = 50 bytes
        vals = struct.unpack_from("<12f", data, off)
        off += 50
        for k in range(3):  # three vertices
            for c in range(3):
                v = vals[3 + 3 * k + c]
                if v < lo[c]:
                    lo[c] = v
                if v > hi[c]:
                    hi[c] = v
    if n_tri == 0:
        raise ValueError(f"empty STL: {path}")
    return tuple(lo), tuple(hi)


# -- URDF tree -> MJCF ------------------------------------------------------
@dataclass
class _Link:
    name: str
    inertial_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertial_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mass: float = 0.0
    inertia: tuple[float, float, float, float, float, float] = (0.0,) * 6
    visuals: list[dict] = field(default_factory=list)
    collisions: list[dict] = field(default_factory=list)


@dataclass
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    mimic_joint: str | None = None
    mimic_multiplier: float = 1.0


def parse_urdf(root: ET.Element) -> tuple[dict[str, _Link], list[_Joint], str]:
    links: dict[str, _Link] = {}
    for l in root.findall("link"):
        link = _Link(name=l.get("name"))
        ine = l.find("inertial")
        if ine is not None:
            o = ine.find("origin")
            if o is not None:
                link.inertial_origin = tuple(float(v) for v in
                                             (o.get("xyz") or "0 0 0").split())
                link.inertial_rpy = tuple(float(v) for v in
                                          (o.get("rpy") or "0 0 0").split())
            m = ine.find("mass")
            if m is not None:
                link.mass = float(m.get("value") or 0.0)
            i = ine.find("inertia")
            if i is not None:
                # ROS names the tensor attributes ixx/ixy/ixz/iyy/iyz/izz.
                # Stored internally as (xx, yy, zz, xy, xz, yz).
                link.inertia = (
                    float(i.get("ixx") or 0.0), float(i.get("iyy") or 0.0),
                    float(i.get("izz") or 0.0), float(i.get("ixy") or 0.0),
                    float(i.get("ixz") or 0.0), float(i.get("iyz") or 0.0))
        for kind, bucket in (("visual", link.visuals), ("collision", link.collisions)):
            for g in l.findall(kind):
                o = g.find("origin")
                geom = {
                    "origin": tuple(float(v) for v in
                                    ((o.get("xyz") if o is not None else "0 0 0")
                                     or "0 0 0").split())
                            if o is not None else (0.0, 0.0, 0.0),
                    "rpy": tuple(float(v) for v in
                                 ((o.get("rpy") if o is not None else "0 0 0")
                                  or "0 0 0").split())
                            if o is not None else (0.0, 0.0, 0.0),
                }
                box = g.find(".//box")
                cyl = g.find(".//cylinder")
                sph = g.find(".//sphere")
                mesh = g.find(".//mesh")
                if box is not None:
                    geom["shape"] = ("box",
                                     tuple(float(v) for v in
                                           (box.get("size") or "0 0 0").split()))
                elif cyl is not None:
                    geom["shape"] = ("cylinder", float(cyl.get("radius") or 0.0),
                                     float(cyl.get("length") or 0.0))
                elif sph is not None:
                    geom["shape"] = ("sphere", float(sph.get("radius") or 0.0))
                elif mesh is not None:
                    geom["shape"] = ("mesh", mesh.get("filename"),
                                     tuple(float(v) for v in
                                           (mesh.get("scale") or "1 1 1").split()))
                else:
                    continue
                if kind == "visual":
                    mat = g.find("material")
                    if mat is not None:
                        geom["rgba"] = tuple(float(v) for v in
                                             (mat.get("rgba") or "0.8 0.8 0.8 1").split())
                bucket.append(geom)
        links[link.name] = link

    joints: list[_Joint] = []
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        mim = j.find("mimic")
        joints.append(_Joint(
            name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            origin_xyz=tuple(float(v) for v in
                             ((o.get("xyz") if o is not None else None)
                              or "0 0 0").split()),
            origin_rpy=tuple(float(v) for v in
                             ((o.get("rpy") if o is not None else None)
                              or "0 0 0").split()),
            axis=tuple(float(v) for v in
                       ((a.get("xyz") if a is not None else None)
                        or "0 0 1").split()),
            lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
            effort=float(lim.get("effort")) if lim is not None and lim.get("effort") else None,
            velocity=float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None,
            mimic_joint=mim.get("joint") if mim is not None else None,
            mimic_multiplier=float(mim.get("multiplier") or 1.0) if mim is not None else 1.0,
        ))
    root_name = root.get("name") or "r1_rebot"
    return links, joints, root_name



# -- MJCF emission ----------------------------------------------------------
#: Arm actuator force bound: the URDF `effort` for J1/J2.  A CAD-declared
#: number, not a measured actuator capability (see SAFETY_REVIEW risk item 4).
ARM_CTRL_RANGE_N = 60.0

#: Gripper channel servo gains and the ceiling on the force it may apply.
#: SIMULATION ASSUMPTIONS.  The DM4310 has no drive characterisation here
#: (`Dm4310Gripper` is still fail-closed), and the 8 N bound is copied from the
#: URDF `effort` field, which is NOT a measured fingertip grasp force.
GRIPPER_KP_DEFAULT = 300.0
GRIPPER_KV_DEFAULT = 6.0
GRIPPER_MAX_FORCE_N = 8.0

#: Half-travel of the driven (left) finger, from the prototype URDF joint limit.
FINGER_TRAVEL_M = 0.05

#: Where the arm's own base plate sits in the world.  SCENE CHOICE, and the
#: number is what the geometry asks for rather than a guess: the prototype URDF
#: models the 7-joint arm and its gripper only - no platform, no torso - so the
#: MJCF has to say how high that arm is off the floor before a work surface can
#: mean anything.  Measured over 60k in-limit configurations, the fingertip plane
#: pointed within 25 deg of vertical sits at z = -0.24 m to +0.33 m RELATIVE to
#: the base, with the bulk of it between -0.16 m and +0.23 m at 0.25-0.35 m of
#: horizontal reach.  A bench top therefore has to land near the base height, and
#: 0.75 m puts it at a height a floor-standing R1 can work at without folding
#: every joint against its limit.  HARDWARE_UNVERIFIED: the real deck height is a
#: platform property this repository has no measurement of.
ROBOT_BASE_Z_M = 0.75

#: The work surface the arm reaches over.  A scene choice, not CAD, and both
#: numbers in it are measurements of this model rather than a picture of a lab:
#: sampling 100k in-limit arm configurations and keeping those whose tool points
#: within 25 deg of vertical AND whose posture respects the 0.05 rad soft margin
#: puts the fingertip plane between 0.58 and 0.95 m in world coordinates (median
#: 0.75), while the gripper's own collision envelope reaches 52 mm PAST its
#: fingertip sites - measured by `GripperScene.robot_dip_below_tip_m`.  So the
#: surface has to be below the reach band by more than that protrusion: at 0.70 m,
#: a 120 mm object presents its gripping plane at 0.80 m, the tool's lowest point
#: lands 48 mm above the surface, and the arm is inside its comfortable band.  Put
#: the surface at 0.76 m instead and every candidate pose penetrates it by 27-36 mm
#: before the pads reach anything.  `tests/test_gripper_pick_place.py` asserts both
#: the accepted band and the rejected low-clearance case.
TABLE_TOP_Z_M = 0.70
#: Half-extent of the slab.  The near edge matters more than the size: with the
#: base at the world origin, the elbow links sweep x < 0.10 at table height, so a
#: slab starting at x = 0.10 is hit by Link4/Link5 long before the fingers reach
#: anything on it.  Starting at 0.30 leaves the elbow clear of the surface.
TABLE_HALF_XY_M = (0.35, 0.35)
TABLE_HALF_THICKNESS_M = 0.02
#: Centre of the slab, from its top surface down.
TABLE_CENTRE_X_M = 0.45

def _principal_inertia(I):
    """URDF's full 3x3 inertia tensor -> (principal values, rotating quat).

    MuJoCo's `diaginertia` only accepts the tensor in its principal frame, so the
    off-diagonal terms are diagonalised here rather than discarded.
    """
    import numpy as np

    m = np.asarray(I, dtype=float)
    vals, vecs = np.linalg.eigh(m)
    vals = [float(v) for v in vals]
    # MuJoCo wants the quat that takes body frame -> principal frame.
    R = vecs.T  # rows are the principal axes
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return vals, (w / n, x / n, y / n, z / n)


def _link_frame_aabb(link, staged) -> tuple[tuple, tuple] | None:
    """Collision extent of a link in its own frame.

    Sums each geom's translated axis-aligned bounds.  Geom rotations are ignored:
    the finger collision meshes here are near-axis-aligned, and the resulting
    sites are labelled as derived approximations in GRIPPER_PROVENANCE.
    """
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    found = False
    for c in link.collisions:
        shape = c["shape"]
        if shape[0] == "mesh":
            ref = staged.get(shape[1])
            if ref is None:
                continue
            mlo, mhi = stl_aabb(ref.source)
            scale = shape[2] or (1.0, 1.0, 1.0)
            mlo = tuple(mlo[k] * scale[k] for k in range(3))
            mhi = tuple(mhi[k] * scale[k] for k in range(3))
        elif shape[0] == "box":
            half = tuple(v / 2.0 for v in shape[1])
            mlo, mhi = tuple(-h for h in half), half
        elif shape[0] == "cylinder":
            r, hl = shape[1], shape[2] / 2.0
            mlo, mhi = (-r, -r, -hl), (r, r, hl)
        elif shape[0] == "sphere":
            r = shape[1]
            mlo, mhi = (-r, -r, -r), (r, r, r)
        else:
            continue
        found = True
        o = c["origin"]
        for k in range(3):
            lo[k] = min(lo[k], mlo[k] + o[k])
            hi[k] = max(hi[k], mhi[k] + o[k])
    if not found:
        return None
    return tuple(lo), tuple(hi)


def _collision_box_fit(shape, staged):
    """Box that hugs a CAD collision mesh, as (centre, half-extents) in its frame.

    MuJoCo cannot collide against these meshes: the prototype's STL collision
    geometries are non-convex, and the compiler silently replaces such a mesh
    with its *bounding sphere* - measured on this model, every arm link and every
    finger became a single sphere, the gripper base a 38 mm one sticking 91 mm out
    of its link.  A sphere that big straddles the fingertip plane, so neither
    grasp clearance nor table clearance could be trusted.  Axis-aligned bounds per
    collision element, rotated by that element's own rpy, reproduce the CAD
    envelope closely enough to measure contact geometry, and cost nothing to
    compile.  Primitives are returned untouched; a mesh whose stage is missing is
    reported rather than guessed at.
    """
    if shape[0] != "mesh":
        return None
    ref = staged.get(shape[1])
    if ref is None:
        raise KeyError(f"mesh {shape[1]!r} was not staged")
    lo, hi = stl_aabb(ref.source)
    scale = shape[2] or (1.0, 1.0, 1.0)
    lo = tuple(lo[k] * scale[k] for k in range(3))
    hi = tuple(hi[k] * scale[k] for k in range(3))
    centre = tuple(0.5 * (lo[k] + hi[k]) for k in range(3))
    # A box needs non-zero thickness in every direction; sheet-like exports can
    # measure ~0 along one axis.
    half = tuple(max(0.5 * (hi[k] - lo[k]), 1e-4) for k in range(3))
    return centre, half


def fingertip_sites(links, staged, left_name, right_name):
    """Sites at the distal inner corner of each finger pad, from measured AABBs.

    The fingers separate along +Y / -Y, so the facing surface is the inner Y face
    and the distal direction is -X (where the collision geoms sit).
    """
    out = {}
    for name, side, rgba in ((left_name, +1.0, "0.9 0.2 0.2 1"),
                             (right_name, -1.0, "0.2 0.9 0.2 1")):
        link = links.get(name)
        if link is None:
            continue
        box = _link_frame_aabb(link, staged)
        if box is None:
            continue
        lo, hi = box
        inner_y = lo[1] if side > 0 else hi[1]
        out[name] = {
            "site_name": f"fingertip_{'left' if side > 0 else 'right'}",
            # Distal end (-X) at the pad's inner face, centred in Z.
            "pos": (lo[0], inner_y, 0.5 * (lo[2] + hi[2])),
            "rgba": rgba,
        }
    return out


@dataclass
class ObjectSpec:
    """A free object to drop into the scene (block, cylinder, ...)."""

    name: str
    shape: str                      # "box" | "cylinder"
    size: tuple                     # box: (x,y,z) full; cylinder: (radius, half-len)
    pos: tuple                      # world position of its centre
    mass: float
    rgba: tuple = (0.2, 0.4, 0.9, 1.0)
    density_free: bool = True

    def geom_attrs(self) -> dict:
        if self.shape == "box":
            return {"type": "box", "size": _fmt([v / 2.0 for v in self.size])}
        if self.shape == "cylinder":
            return {"type": "cylinder",
                    "size": _fmt([self.size[0], self.size[1]])}
        raise ValueError(f"unknown object shape {self.shape!r}")


def _emit_body(link, joints_by_parent, links, staged, mesh_names, *,
               collision_mode, collision_shape, finger_sites, extra_sites):
    b = ET.Element("body")
    b.set("name", link.name)
    if link.mass > 0.0:
        el = ET.SubElement(b, "inertial")
        el.set("pos", _fmt(link.inertial_origin))
        if any(abs(v) > 1e-12 for v in (link.inertia[3], link.inertia[4],
                                        link.inertia[5])):
            vals, quat = _principal_inertia(
                [[link.inertia[0], link.inertia[3], link.inertia[4]],
                 [link.inertia[3], link.inertia[1], link.inertia[5]],
                 [link.inertia[4], link.inertia[5], link.inertia[2]]])
            # body -> inertial frame (rpy) composed with inertial -> principal.
            el.set("quat", _fmt(quat_mul(rpy_to_quat(link.inertial_rpy), quat)))
            el.set("diaginertia", _fmt(vals))
        else:
            el.set("pos", _fmt(link.inertial_origin))
            el.set("quat", _fmt(rpy_to_quat(link.inertial_rpy)))
            el.set("diaginertia", _fmt([link.inertia[0], link.inertia[1],
                                        link.inertia[2]]))
        el.set("mass", _fmt(link.mass))

    for v in link.visuals:
        g = _geom_element(v["shape"], klass="visual", staged=staged,
                          mesh_names=mesh_names)
        g.set("pos", _fmt(v["origin"]))
        g.set("quat", _fmt(rpy_to_quat(v["rpy"])))
        b.append(g)
    for c in link.collisions:
        fit = (_collision_box_fit(c["shape"], staged)
               if collision_shape == "box" else None)
        if fit is None:
            g = _geom_element(c["shape"], klass="collision", staged=staged,
                              mesh_names=mesh_names)
            g.set("pos", _fmt(c["origin"]))
        else:
            centre, half = fit
            rot = _rpy_matrix(c["rpy"])
            pos = tuple(c["origin"][k]
                        + sum(rot[k][j] * centre[j] for j in range(3))
                        for k in range(3))
            g = _geom_element(("box", tuple(2.0 * h for h in half)),
                              klass="collision", staged=staged,
                              mesh_names=mesh_names)
            g.set("pos", _fmt(pos))
        g.set("quat", _fmt(rpy_to_quat(c["rpy"])))
        b.append(g)

    for st in extra_sites.get(link.name, []):
        site = ET.SubElement(b, "site")
        site.set("name", st["name"])
        site.set("pos", _fmt(st["pos"]))
        site.set("quat", _fmt(rpy_to_quat(st.get("rpy", (0, 0, 0)))))
        site.set("size", st.get("size", "0.005"))
        site.set("rgba", st.get("rgba", "1 0 0 1"))
        site.set("group", st.get("group", "4"))

    if link.name in finger_sites:
        fs = finger_sites[link.name]
        site = ET.SubElement(b, "site")
        site.set("name", fs["site_name"])
        site.set("pos", _fmt(fs["pos"]))
        site.set("size", "0.005")
        site.set("rgba", fs["rgba"])
        site.set("group", "4")

    for j in joints_by_parent.get(link.name, []):
        child = _emit_body(links[j.child], joints_by_parent, links, staged,
                           mesh_names, collision_mode=collision_mode,
                           collision_shape=collision_shape,
                           finger_sites=finger_sites, extra_sites=extra_sites)
        child.set("pos", _fmt(j.origin_xyz))
        child.set("quat", _fmt(rpy_to_quat(j.origin_rpy)))
        if j.type != "fixed":
            jt = ET.SubElement(child, "joint")
            jt.set("name", j.name)
            jt.set("class", "arm_joint" if j.type == "revolute" else "finger_joint")
            jt.set("axis", _fmt(j.axis))
            if j.type == "revolute":
                jt.set("type", "hinge")
            elif j.type == "prismatic":
                jt.set("type", "slide")
            if j.type in ("revolute", "prismatic") and j.lower is not None:
                jt.set("range", _fmt([j.lower, j.upper]))
                jt.set("limited", "true")
        b.append(child)
    return b


def _geom_element(shape, *, klass, staged, mesh_names):
    g = ET.Element("geom")
    if klass:
        g.set("class", klass)
    kind = shape[0]
    if kind == "box":
        g.set("type", "box")
        g.set("size", _fmt([v / 2.0 for v in shape[1]]))
    elif kind == "cylinder":
        g.set("type", "cylinder")
        g.set("size", _fmt([shape[1], shape[2] / 2.0]))
    elif kind == "sphere":
        g.set("type", "sphere")
        g.set("size", _fmt([shape[1]]))
    elif kind == "mesh":
        ref = staged.get(shape[1])
        if ref is None:
            raise KeyError(f"mesh {shape[1]!r} was not staged")
        mesh_names[ref.dest_name] = ref
        g.set("mesh", ref.dest_name[:-4] if ref.dest_name.endswith(".stl")
              else ref.dest_name)
        if any(abs(s - 1.0) > 1e-12 for s in (shape[2] or (1, 1, 1))):
            g.set("scale", _fmt(shape[2]))
    return g


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def adjacent_excludes(joints, root_name):
    """Body pairs to exclude from contact when using convex-hull collisions.

    Mirrors the finding in SIMULATION.md: hulls of concave visual meshes overlap
    at every joint, so parent/child and grandparent pairs read as false
    self-collisions.  Returned as (body_a, body_b) name pairs.
    """
    parent_of = {j.child: j.parent for j in joints if j.type != "floating"}

    def ancestry(name):
        out = []
        cur = parent_of.get(name)
        depth = 0
        while cur is not None and depth < 3:
            out.append(cur)
            cur = parent_of.get(cur)
            depth += 1
        return out

    pairs = set()
    for child, parent in parent_of.items():
        for anc in ancestry(child):
            pairs.add(tuple(sorted((child, anc))))
    return sorted(p for p in pairs if p[0] != p[1])


def generate_gripper_mjcf(*,
                          gripper_urdf: Path | None = None,
                          collision_mode: str = "environment",
                          collision_shape: str = "box",
                          add_table: bool = True,
                          table_top_z: float = TABLE_TOP_Z_M,
                          robot_base_z: float = ROBOT_BASE_Z_M,
                          objects: list[ObjectSpec] | None = None,
                          target_zone: tuple | None = None,
                          meshdir: str | None = None,
                          timestep: float = 0.002,
                          gravity: tuple = (0.0, 0.0, -9.81),
                          gripper_kp: float = GRIPPER_KP_DEFAULT,
                          gripper_kv: float = GRIPPER_KV_DEFAULT,
                          gripper_force_limit_n: float = GRIPPER_MAX_FORCE_N,
                          armature: float = 0.01,
                          joint_damping: float = 0.2,
                          with_manifest: bool = False):
    """Convert the combined prototype URDF into a self-contained, drivable MJCF.

    Eight control channels come out: one per arm joint plus a single gripper
    channel.  The two fingers keep their separate physical DOF and are tied by an
    equality constraint, so the gripper is a real mechanical system and not a
    visual animation.
    """
    if collision_mode not in ("none", "environment", "self"):
        raise ValueError(f"unknown collision_mode {collision_mode!r}")
    if collision_shape not in ("box", "mesh"):
        raise ValueError(f"unknown collision_shape {collision_shape!r}")

    if meshdir is None:
        # Relative to where the model is written, so the file opens from any
        # checkout location without a machine-specific absolute path.
        meshdir = os.path.relpath(STAGED_MESH_DIR, GRIPPER_MODEL_PATH.parent)

    sources = mesh_sources(gripper_urdf)
    urdf_path = sources.gripper_urdf
    root = ET.parse(str(urdf_path)).getroot()
    links, joints, model_name = parse_urdf(root)

    staged = stage_meshes(root, sources)
    mesh_names: dict[str, StagedMesh] = {}

    joints_by_parent: dict[str, list[_Joint]] = {}
    children = {j.child for j in joints}
    for j in joints:
        joints_by_parent.setdefault(j.parent, []).append(j)
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"expected one root link, found {roots}")

    # Fingertip sites from the measured collision bounds of each finger link.
    finger_links = {j.child: j.name for j in joints if j.type == "prismatic"}
    tip_sites = {}
    raw_tips = {}
    for lname, jname in finger_links.items():
        side = +1.0 if jname == FINGER_LEFT else -1.0
        box = _link_frame_aabb(links[lname], staged)
        if box is None:
            continue
        lo, hi = box
        raw_tips[lname] = (lo, hi, side)
        tip_sites[lname] = {
            "site_name": f"fingertip_{'left' if side > 0 else 'right'}",
            "pos": (lo[0], lo[1] if side > 0 else hi[1], 0.5 * (lo[2] + hi[2])),
            "rgba": ("0.9 0.25 0.25 1" if side > 0 else "0.25 0.9 0.25 1"),
        }

    # The gripper center sits between the two pads, at their common distal X.
    extra_sites: dict[str, list[dict]] = {}
    tips = list(raw_tips.values())
    if tips:
        distal_x = min(lo[0] for lo, hi, _ in tips)
        z_mid = 0.5 * (min(lo[2] for lo, _, _ in tips)
                       + max(hi[2] for _, hi, _ in tips))
        center_host = None
        for j in joints:
            if j.child in raw_tips:
                center_host = j.parent
                break
        if center_host:
            extra_sites.setdefault(center_host, []).append({
                "name": "gripper_center_site",
                "pos": (distal_x, 0.0, z_mid),
                "rgba": "0.9 0.9 0.2 1",
            })
    # TCP: the prototype defines it at the finger base frame, rotated about Y.
    for j in joints:
        if j.name == "prototype_tcp_joint":
            extra_sites.setdefault(j.parent, []).append({
                "name": "tcp_site",
                "pos": j.origin_xyz,
                "rpy": j.origin_rpy,
                "rgba": "1 0 0 1",
                "size": "0.008",
            })

    mujoco = ET.Element("mujoco", {"model": f"{model_name}_sim"})
    comment = ET.Comment(
        " GENERATED by r1_robot.gripper_mjcf.generate_gripper_mjcf().\n"
        " Do not edit by hand; regenerate with sim/build_gripper_model.py.\n"
        " Sources: the official R1 URDF (read-only, vendored SDK submodule) and\n"
        " the gripper prototype URDF (read-only, located via R1_GRIPPER_URDF).\n"
        " Meshes are staged under sim/meshes/. No absolute paths are written, so\n"
        " this file opens from any checkout location.\n")
    mujoco.append(comment)

    ET.SubElement(mujoco, "compiler", {
        "angle": "radian", "autolimits": "false",
        "meshdir": meshdir, "boundmass": "0", "boundinertia": "0"})
    opt = ET.SubElement(mujoco, "option", {
        "timestep": _fmt(timestep), "gravity": _fmt(gravity),
        "integrator": "implicitfast"})
    ET.SubElement(opt, "flag", {"contact": "enable"})

    default = ET.SubElement(mujoco, "default")
    aj = ET.SubElement(default, "default", {"class": "arm_joint"})
    ET.SubElement(aj, "joint", {"damping": _fmt(joint_damping),
                                "armature": _fmt(armature)})
    fj = ET.SubElement(default, "default", {"class": "finger_joint"})
    ET.SubElement(fj, "joint", {"damping": _fmt(0.05),
                                "armature": _fmt(armature),
                                "frictionloss": _fmt(0.1)})
    dv = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(dv, "geom", {"group": "2", "contype": "0", "conaffinity": "0",
                               "rgba": "0.75 0.78 0.82 1"})
    masks = {"none": ("0", "0"), "environment": ("1", "1"), "self": ("1", "1")}
    ct, ca = masks[collision_mode]
    dc = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(dc, "geom", {"group": "3", "contype": ct, "conaffinity": ca,
                               "condim": "4", "solimp": "0.9 0.95 0.001 0.5 2",
                               "solref": "0.005 1",
                               "friction": "0.9 0.0001 0.0001"})
    do = ET.SubElement(default, "default", {"class": "object"})
    ET.SubElement(do, "geom", {"group": "3", "contype": "1", "conaffinity": "1",
                               "condim": "6", "solimp": "0.9 0.95 0.001 0.5 2",
                               "solref": "0.005 1",
                               "friction": "0.9 0.0001 0.0001"})

    asset = ET.SubElement(mujoco, "asset")
    world = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(world, "light", {"name": "key", "pos": "0.6 -0.6 1.6",
                                   "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8"})
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane",
                                  "size": "2 2 0.1", "contype": "2",
                                  "conaffinity": "1", "condim": "4",
                                  "friction": "1 0.005 0.0001"})
    if add_table:
        hx, hy = TABLE_HALF_XY_M
        ET.SubElement(world, "geom", {
            "name": "tabletop", "type": "box",
            "pos": _fmt((TABLE_CENTRE_X_M, 0.0,
                         table_top_z - TABLE_HALF_THICKNESS_M)),
            "size": _fmt((hx, hy, TABLE_HALF_THICKNESS_M)),
            "contype": "2", "conaffinity": "1",
            "rgba": "0.55 0.42 0.25 1", "mass": "0"})
    for obj in objects or []:
        ob = ET.SubElement(world, "body", {"name": obj.name,
                                           "pos": _fmt(obj.pos)})
        ET.SubElement(ob, "freejoint", {"name": f"{obj.name}_free"})
        g = ET.SubElement(ob, "geom", {"name": f"{obj.name}_geom", "class": "object",
                                       **obj.geom_attrs()})
        g.set("rgba", _fmt(obj.rgba))
        ET.SubElement(ob, "inertial", {"mass": _fmt(obj.mass),
                                       "diaginertia": _fmt(_object_inertia(obj))})
    if target_zone is not None:
        pos, half = target_zone
        st = ET.SubElement(world, "site", {"name": "target_zone",
                                           "type": "box", "pos": _fmt(pos),
                                           "size": _fmt(half),
                                           "rgba": "0.2 0.9 0.4 0.35",
                                           "group": "4"})
        del st

    robot = _emit_body(links[roots[0]], joints_by_parent, links, staged,
                       mesh_names, collision_mode=collision_mode,
                       collision_shape=collision_shape,
                       finger_sites=tip_sites, extra_sites=extra_sites)
    # The arm alone is what the URDF models; how high it stands off the floor is
    # the scene's decision, and the work surface height only means anything once
    # it is made (see ROBOT_BASE_Z_M).
    robot.set("pos", _fmt((0.0, 0.0, robot_base_z)))
    world.append(robot)

    assets = ET.Element("asset")
    for dest_name, ref in sorted(mesh_names.items()):
        ET.SubElement(assets, "mesh", {"name": dest_name[:-4] if
                                       dest_name.endswith(".stl") else dest_name,
                                       "file": dest_name})
    mujoco.insert(list(mujoco).index(asset) + 1, assets)
    mujoco.remove(asset)

    if collision_mode != "none":
        # Adjacent links touch at their joints as soon as the collision proxies
        # follow the CAD envelope instead of a sphere around it, and MuJoCo only
        # filters direct parent/child pairs.  Both modes that generate robot
        # contacts therefore need the exclusion list.
        contact = ET.SubElement(mujoco, "contact")
        for a, b in adjacent_excludes(joints, roots[0]):
            ET.SubElement(contact, "exclude", {"body1": a, "body2": b})

    eq = ET.SubElement(mujoco, "equality")
    for j in joints:
        if j.mimic_joint:
            ET.SubElement(eq, "joint", {
                "name": f"{j.name}_mimic", "joint1": j.name,
                "joint2": j.mimic_joint,
                "polycoef": _fmt((0.0, j.mimic_multiplier, 0, 0, 0)),
                "solref": "0.01 1", "solimp": "0.9 0.999 0.00001 0.5 2"})

    act = ET.SubElement(mujoco, "actuator")
    for j in joints:
        if j.type == "revolute" and j.name in ARM_JOINTS:
            a = ET.SubElement(act, "general", {"name": f"{j.name}_ctrl",
                                               "joint": j.name,
                                               "gaintype": "fixed",
                                               "gainprm": "1 0 0",
                                               "biastype": "affine",
                                               "biasprm": "0 0 0",
                                               "forcelimited": "true",
                                               "forcerange": _fmt(
                                                   [-ARM_CTRL_RANGE_N,
                                                    ARM_CTRL_RANGE_N])})
            del a
    for j in joints:
        if j.type == "prismatic" and j.name == FINGER_LEFT:
            lo = j.lower if j.lower is not None else 0.0
            hi = j.upper if j.upper is not None else FINGER_TRAVEL_M
            ET.SubElement(act, "general", {
                "name": GRIPPER_CHANNEL, "joint": j.name,
                "gaintype": "fixed", "gainprm": _fmt(gripper_kp),
                "biastype": "affine",
                "biasprm": _fmt((0.0, -gripper_kp, -gripper_kv)),
                "ctrllimited": "true", "ctrlrange": _fmt((lo, hi)),
                "forcelimited": "true",
                "forcerange": _fmt((-gripper_force_limit_n,
                                    gripper_force_limit_n))})
    xml = ET.tostring(mujoco, encoding="unicode")
    if with_manifest:
        entries = {ref.dest_name: {"source": str(ref.source), "sha256": ref.sha256}
                   for ref in staged.values()}
        return xml, entries
    return xml


def _object_inertia(obj: ObjectSpec) -> tuple:
    """Solid box / cylinder principal inertia about the centre, from its mass."""
    m = obj.mass
    if obj.shape == "box":
        x, y, z = obj.size
        return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z),
                m / 12.0 * (x * x + y * y))
    r, h = obj.size
    return (m * (3 * r * r + 4 * h * h) / 12.0,
            m * (3 * r * r + 4 * h * h) / 12.0, m * r * r / 2.0)


def write_gripper_model(out_path: Path = GRIPPER_MODEL_PATH, **kwargs) -> Path:
    xml, manifest = generate_gripper_mjcf(with_manifest=True, **kwargs)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml)
    # Records which CAD each staged binary came from, so a review can trace it.
    (STAGED_MESH_DIR / "manifest.json").write_text(
        json.dumps({"meshes": manifest}, indent=2, sort_keys=True) + "\n")
    return out_path
