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
    ap.add_argument("--gripper-kp", type=float, default=None)
    ap.add_argument("--gripper-force-limit-n", type=float, default=None)
    ap.add_argument("--show-provenance", action="store_true")
    a = ap.parse_args()

    if a.show_provenance:
        for k, v in gripper_provenance().items():
            print(f"  {k:38s} {v}")
        return 0

    kwargs = {"collision_mode": a.collision_mode, "add_table": not a.no_table}
    if a.gripper_kp is not None:
        kwargs["gripper_kp"] = a.gripper_kp
    if a.gripper_force_limit_n is not None:
        kwargs["gripper_force_limit_n"] = a.gripper_force_limit_n
    path = write_gripper_model(Path(a.out), **kwargs)
    print(f"wrote {path} ({path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
