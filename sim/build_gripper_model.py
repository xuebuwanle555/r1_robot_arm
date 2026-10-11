#!/usr/bin/env python3
"""Regenerate sim/models/r1_rebot_sim.xml from the read-only source CAD.

    python3 sim/build_gripper_model.py [--collision-mode environment|self|none]
                                       [--no-table] [--out PATH] [--mesh-dir DIR]

Inputs are never modified.  Meshes referenced by the source URDF are copied into
sim/meshes/ so the generated model opens with relative paths only.  Set
R1_GRIPPER_URDF / R1_GRIPPER_MESH_ROOT when the prototype lives elsewhere.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))

from r1_robot.gripper_mjcf import (  # noqa: E402
    GRIPPER_MODEL_PATH,
    gripper_provenance,
    write_gripper_model,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(GRIPPER_MODEL_PATH))
    ap.add_argument("--collision-mode", default="environment",
                    choices=("none", "environment", "self"))
    ap.add_argument("--no-table", action="store_true")
    ap.add_argument("--table-top-z", type=float, default=None,
                    help="height of the work surface; default is the scene "
                         "constant TABLE_TOP_Z_M in r1_robot.gripper_mjcf")
    ap.add_argument("--robot-base-z", type=float, default=None,
                    help="how high the arm base sits off the floor; default is "
                         "the scene constant ROBOT_BASE_Z_M")
    ap.add_argument("--collision-shape", default="box", choices=("box", "mesh"),
                    help="box = oriented boxes fitted to each CAD collision "
                         "mesh (default; MuJoCo turns the raw meshes into "
                         "bounding spheres, which cannot show grasp geometry). "
                         "mesh = keep the mesh geoms and see what compiles")
    ap.add_argument("--gripper-kp", type=float, default=None)
    ap.add_argument("--gripper-force-limit-n", type=float, default=None)
    ap.add_argument("--show-provenance", action="store_true")
    a = ap.parse_args()

    if a.show_provenance:
        for k, v in gripper_provenance().items():
            print(f"  {k:38s} {v}")
        return 0

    kwargs = {"collision_mode": a.collision_mode,
              "collision_shape": a.collision_shape,
              "add_table": not a.no_table}
    if a.table_top_z is not None:
        kwargs["table_top_z"] = a.table_top_z
    if a.robot_base_z is not None:
        kwargs["robot_base_z"] = a.robot_base_z
    if a.gripper_kp is not None:
        kwargs["gripper_kp"] = a.gripper_kp
    if a.gripper_force_limit_n is not None:
        kwargs["gripper_force_limit_n"] = a.gripper_force_limit_n
    path = write_gripper_model(Path(a.out), **kwargs)
    print(f"wrote {path} ({path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
