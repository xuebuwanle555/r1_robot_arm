#!/usr/bin/env python3
"""Task C/D: pick-and-place trials over a grid of object positions, logged.

    python3 sim/run_gripper_pick_place.py [--trials-per-shape 10] [--out DIR]

Everything is OFFLINE SIMULATION.  No real R1 is connected, no real DM4310 is
driven, and no number in the output is evidence about hardware: the contact
stiffness, the friction, the gripper gains and the 8 N channel ceiling are
simulation assumptions (`r1_robot.gripper_mjcf.GRIPPER_PROVENANCE`), and the
grasp forces are simulated contact forces.

Writes, per trial:
  <out>/rows_<shape>_<n>.jsonl   telemetry, one control cycle per line
  <out>/trials.csv               one line per trial: outcome + why
  <out>/config.json              seed, object spec, tolerances, model hash

The success rate reported is the measured one.  A trial that cannot plan a grasp,
cannot close to the force its own payload needs, or drops the object is counted as
a failure and keeps its log.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))

import numpy as np  # noqa: E402

from r1_robot.gripper_ik import GripperKinematics, WorkspaceSampler  # noqa: E402
from r1_robot.gripper_mjcf import (GRIPPER_MODEL_PATH, ObjectSpec,  # noqa: E402
                                   TABLE_TOP_Z_M, gripper_provenance)
from r1_robot.gripper_task import SceneRig  # noqa: E402
from r1_robot.poses import SAFE_HOME  # noqa: E402

#: Trial grid: positions on the work surface, in metres, world frame.  Chosen
#: inside the band `SceneRig.find_grasp` accepts (x 0.20-0.32, |y| <= 0.10 at the
#: gripping plane this bench height implies) rather than guessed at.
POSITIONS = [(0.22, 0.00), (0.25, 0.05), (0.28, -0.05), (0.30, 0.08),
             (0.24, -0.08), (0.27, 0.02), (0.20, 0.06), (0.31, -0.02),
             (0.26, -0.10), (0.23, 0.09)]

#: Objects: a tall square bar and a standing cylinder, 40 mm across, 120 mm tall,
#: 50 g.  Short objects are not graspable on this bench by this gripper - the
#: gripping-plane search in `run_pick_and_place` reports that rather than fudging
#: it - so the fixtures are the shape the measured clearance allows.
SHAPES = {
    "bar": {"shape": "box", "size": (0.04, 0.04, 0.12), "width": 0.04},
    "cylinder": {"shape": "cylinder", "size": (0.02, 0.06), "width": 0.04},
}
HEIGHT = 0.12
MASS = 0.05


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(REPO / "sim" / "results" / "pick_place"))
    ap.add_argument("--trials-per-shape", type=int, default=10)
    ap.add_argument("--samples", type=int, default=60000,
                    help="workspace samples used to seed grasp planning")
    ap.add_argument("--perturb", action="store_true",
                    help="second pass with friction and mass perturbed")
    a = ap.parse_args()

    out = Path(a.out)
    (out / "rows").mkdir(parents=True, exist_ok=True)
    kin = GripperKinematics()
    sampler = WorkspaceSampler(kin, samples=a.samples, seed=0)
    model_hash = hashlib.sha256(GRIPPER_MODEL_PATH.read_bytes()).hexdigest()[:12]

    rows = []
    passes = [("nominal", 1.0, 1.0)]
    if a.perturb:
        passes += [("low_friction", 0.6, 1.0), ("heavy", 1.0, 2.0)]
    t_start = time.time()
    for tag, friction_scale, mass_scale in passes:
        for shape_name, spec in SHAPES.items():
            for i, (x, y) in enumerate(POSITIONS[:a.trials_per_shape]):
                mass = MASS * mass_scale
                obj = ObjectSpec(name="obj", shape=spec["shape"],
                                 size=spec["size"],
                                 pos=(x, y, TABLE_TOP_Z_M + 0.5 * HEIGHT),
                                 mass=mass)
                rig = SceneRig.create(objects=[obj], seed=i, sampler=sampler)
                try:
                    rig.bringup(SAFE_HOME, settle_s=0.6)
                except (RuntimeError, PermissionError) as exc:
                    rows.append({"pass": tag, "shape": shape_name, "trial": i,
                                 "x": x, "y": y, "success": False,
                                 "failure": f"bringup: {exc}",
                                 "grasped": False, "lifted": False,
                                 "placed": False})
                    continue
                tr = rig.run_pick_and_place(
                    "obj", surface_z=TABLE_TOP_Z_M, object_height=HEIGHT,
                    object_width=spec["width"], place_xy=(x + 0.05, y - 0.05))
                row = {"pass": tag, "shape": shape_name, "trial": i, "x": x,
                       "y": y, "success": tr.success,
                       "grasped": tr.grasped, "lifted": tr.lifted,
                       "placed": tr.placed, "failure": tr.failure_reason(),
                       "steps": ";".join(f"{s.name}={int(s.ok)}"
                                         for s in tr.steps)}
                for k in ("grip_plane_z", "close_force_n",
                          "required_grip_force_n", "lift_m", "slip_m",
                          "place_error_m", "approach_off_vertical_rad",
                          "jaw_off_horizontal_rad", "site_to_pad_offset_m"):
                    v = tr.metrics.get(k)
                    row[k] = None if v is None or (isinstance(v, float)
                                                   and not np.isfinite(v)) else round(
                        float(v), 5)
                rows.append(row)
                with (out / "rows" / f"{tag}_{shape_name}_{i}.jsonl").open(
                        "w") as f:
                    for snapshot in rig.log:
                        f.write(json.dumps(snapshot) + "\n")
                print(f"[{tag:11s}] {shape_name:8s} #{i} x={x:+.2f} y={y:+.2f} "
                      f"success={tr.success} {tr.failure_reason()}", flush=True)
    with (out / "trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (out / "config.json").write_text(json.dumps({
        "model": str(GRIPPER_MODEL_PATH.relative_to(REPO)),
        "model_sha256_12": model_hash,
        "table_top_z_m": TABLE_TOP_Z_M,
        "samples": a.samples,
        "positions": POSITIONS[:a.trials_per_shape],
        "shapes": SHAPES, "height_m": HEIGHT, "mass_kg": MASS,
        "passes": [p[0] for p in passes],
        "provenance": gripper_provenance(),
        "wall_clock_s": round(time.time() - t_start, 1),
    }, indent=2, sort_keys=True) + "\n")

    print()
    for tag, _f, _m in passes:
        for shape_name in SHAPES:
            sel = [r for r in rows if r["pass"] == tag and r["shape"] == shape_name]
            n = len(sel)
            k = sum(1 for r in sel if r["success"])
            print(f"{tag}/{shape_name}: {k}/{n} succeeded "
                  f"({(100.0 * k / n if n else 0):.0f}%)")
    reasons = {}
    for r in rows:
        if not r["success"]:
            reasons[r["failure"].split(":")[0]] = \
                reasons.get(r["failure"].split(":")[0], 0) + 1
    print("failure stages:", json.dumps(reasons, sort_keys=True))
    print(f"logs in {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
