"""Inventory of the gripper-equipped R1 prototype model.

The combined model (7 arm joints + reBot gripper) lives outside this repository,
in the PhysGrasp prototype directory that generates it.  This module does not
drive it and does not modify it; it parses it and reports the facts the next
phase has to build on, so that phase starts from measurements rather than from
assumptions about what the URDF contains.

Deliberately standard-library only (``xml.etree``): reading the inventory needs
neither MuJoCo nor Pinocchio, so it runs in the offline test job.

Nothing here is validated on hardware.  The masses, offsets, adapter geometry
and finger inertias are the source model's own declared assumptions, and the
URDF ``effort`` fields are not measured grasp forces.
"""
from __future__ import annotations

import hashlib
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

#: Where the generator writes the combined model.  Override with
#: ``R1_GRIPPER_URDF`` when the prototype directory moves.
DEFAULT_URDF = Path(
    "/home/jesse/physgrasp_cvpr27/assets/robots/r1_rebot_prototype/r1_rebot.urdf"
)

ARM_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def urdf_path() -> Path:
    return Path(os.environ.get("R1_GRIPPER_URDF", str(DEFAULT_URDF)))


@dataclass
class JointFacts:
    name: str
    type: str
    parent: str
    child: str
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    mimic_of: str | None = None
    mimic_multiplier: float | None = None


@dataclass
class GripperModelFacts:
    path: Path
    model_name: str
    sha256: str
    links: list[str] = field(default_factory=list)
    joints: list[JointFacts] = field(default_factory=list)
    mesh_files: list[str] = field(default_factory=list)
    meshes_missing: list[str] = field(default_factory=list)

    @property
    def arm_joints(self) -> list[JointFacts]:
        return [j for j in self.joints if j.name in ARM_JOINT_NAMES]

    @property
    def prismatic_joints(self) -> list[JointFacts]:
        return [j for j in self.joints if j.type == "prismatic"]

    @property
    def mimic_joints(self) -> list[JointFacts]:
        return [j for j in self.joints if j.mimic_of is not None]

    @property
    def movable_joints(self) -> list[JointFacts]:
        return [j for j in self.joints if j.type in ("revolute", "prismatic",
                                                     "continuous")]

    @property
    def independent_dof(self) -> int:
        """Movable joints minus the ones a mimic joint merely echoes."""
        return len(self.movable_joints) - len(self.mimic_joints)

    def joint(self, name: str) -> JointFacts:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(name)


def load_facts(path: Path | None = None) -> GripperModelFacts:
    p = Path(path) if path is not None else urdf_path()
    raw = p.read_bytes()
    root = ET.fromstring(raw.decode("utf-8"))

    facts = GripperModelFacts(
        path=p,
        model_name=root.get("name") or "",
        sha256=hashlib.sha256(raw).hexdigest(),
        links=[l.get("name") for l in root.findall("link")],
    )

    for j in root.findall("joint"):
        origin = j.find("origin")
        lim = j.find("limit")
        mim = j.find("mimic")
        facts.joints.append(
            JointFacts(
                name=j.get("name"),
                type=j.get("type"),
                parent=(j.find("parent").get("link") if j.find("parent") is not None else ""),
                child=(j.find("child").get("link") if j.find("child") is not None else ""),
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                effort=float(lim.get("effort")) if lim is not None and lim.get("effort") else None,
                velocity=(float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None),
                mimic_of=mim.get("joint") if mim is not None else None,
                mimic_multiplier=(float(mim.get("multiplier", "1")) if mim is not None else None),
            )
        )
        del origin  # origins are covered by the generated-MJCF work, not here.

    for mesh in root.iter("mesh"):
        fn = mesh.get("filename") or ""
        facts.mesh_files.append(fn)
        if not Path(fn).exists():
            facts.meshes_missing.append(fn)
    return facts
