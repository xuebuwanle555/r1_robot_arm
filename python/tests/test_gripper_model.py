"""Tests for the generated R1 + reBot gripper model (Task A).

The model is produced by `r1_robot.gripper_mjcf.write_gripper_model()` from two
read-only CAD sources.  These tests are about faithfulness and structure, not
about grasping: they ask whether the conversion preserved the geometry the URDF
compiler would have produced, whether all eight control channels exist, whether
the fingers are still two real physical DOF tied by a constraint, and whether the
generated file can be opened anywhere.

Nothing here measures a real grasp force; `effort = 8` in the source URDF is a
CAD field, not a measurement.
"""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402
from r1_robot.gripper_mjcf import (  # noqa: E402
    GRIPPER_MODEL_PATH,
    gripper_provenance,
    mesh_sources,
    write_gripper_model,
)

try:
    import mujoco

    HAVE_MUJOCO = True
except Exception:  # pragma: no cover
    HAVE_MUJOCO = False

HAVE_SOURCES = mesh_sources().gripper_urdf.exists() and HAVE_MUJOCO


def _model():
    return mujoco.MjModel.from_xml_path(str(GRIPPER_MODEL_PATH))


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not available")
@unittest.skipUnless(HAVE_SOURCES, "gripper prototype CAD not reachable")
class TestGeneratedGripperModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = _model()
        cls.d = mujoco.MjData(cls.m)
        cls.jadr = {
            mujoco.mj_id2name(cls.m, mujoco.mjtObj.mjOBJ_JOINT, i):
                int(cls.m.jnt_qposadr[i]) for i in range(cls.m.njnt)
        }

    def _joint_range(self, name):
        for i in range(self.m.njnt):
            if mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, i) == name:
                return self.m.jnt_limited[i], self.m.jnt_range[i]
        raise KeyError(name)

    # -- structure ----------------------------------------------------------
    def test_dof_and_channel_counts(self):
        self.assertEqual(self.m.nq, 9, "7 arm + 2 finger coordinates")
        self.assertEqual(self.m.nv, 9, "both fingers keep a velocity DOF")
        self.assertEqual(self.m.nu, 8, "7 arm channels + 1 gripper channel")
        self.assertEqual(self.m.neq, 1, "the mimic is one equality constraint")
        names = sorted(mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                       for i in range(self.m.nu))
        self.assertIn("gripper_ctrl", names)
        for j in rr.JOINT_NAMES:
            self.assertIn(f"{j}_ctrl", names)

    def test_fingers_are_not_puppeted_by_a_second_actuator(self):
        """The right finger must have no actuator of its own."""
        act = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
               for i in range(self.m.nu)]
        self.assertFalse(any("right" in a for a in act),
                         f"only the left finger may be driven, got {act}")
        tr = self.m.actuator_trnid[:, 0]
        driven = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, int(t))
                  for t in tr}
        self.assertIn("rebot_finger_left", driven)
        self.assertNotIn("rebot_finger_right", driven)

    def test_arm_limits_are_the_frozen_official_values(self):
        defaults = rr.r1_default_limits()
        for i, name in enumerate(rr.JOINT_NAMES):
            limited, rng = self._joint_range(name)
            self.assertTrue(bool(limited), f"{name} must be limited")
            self.assertAlmostEqual(rng[0], defaults[i].lower, places=4)
            self.assertAlmostEqual(rng[1], defaults[i].upper, places=4)

    def test_finger_travel_matches_the_source_urdf(self):
        limited, rng = self._joint_range("rebot_finger_left")
        self.assertTrue(bool(limited))
        self.assertAlmostEqual(rng[0], 0.0, places=6)
        self.assertAlmostEqual(rng[1], 0.05, places=6)
        limited, rng = self._joint_range("rebot_finger_right")
        self.assertAlmostEqual(rng[0], -0.05, places=6)
        self.assertAlmostEqual(rng[1], 0.0, places=6)

    def test_gripper_ctrl_range_is_physical_travel_not_normalized(self):
        """The channel is in metres; [0,1] normalisation is the adapter's job."""
        for i in range(self.m.nu):
            if mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                 i) == "gripper_ctrl":
                lo, hi = self.m.actuator_ctrlrange[i]
                self.assertAlmostEqual(lo, 0.0, places=6)
                self.assertAlmostEqual(hi, 0.05, places=6)
                self.assertTrue(bool(self.m.actuator_forcelimited[i]))
                return
        self.fail("gripper_ctrl missing")

    # -- geometry fidelity vs the engine's own URDF compiler ----------------
    def _body_ids(self, model, name):
        """Name -> id, refusing the negative-index trap that hides a missing body."""
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        self.assertGreaterEqual(i, 0, f"{name} has no body id")
        return i

    def test_conversion_matches_mujoco_urdf_compiler(self):
        """Where both models have a body, the transform must agree.

        MuJoCo's URDF compiler merges bodies that hang off a fixed joint into
        their parent (its Link7 mass is ours plus the adapter plus the gripper
        base).  The comparison is therefore restricted to the bodies that exist
        on both sides; the merge itself is pinned by a separate test below.
        """
        src = str(mesh_sources().gripper_urdf)
        ref = mujoco.MjModel.from_xml_path(src)
        rd = mujoco.MjData(ref)
        ref_adr = {mujoco.mj_id2name(ref, mujoco.mjtObj.mjOBJ_JOINT, i):
                   int(ref.jnt_qposadr[i]) for i in range(ref.njnt)}
        shared = []
        for i in range(self.m.nbody):
            n = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, i)
            if n and mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_BODY, n) >= 0:
                shared.append(n)
        self.assertGreaterEqual(len(shared), 8,
                                "expected the arm links to exist on both sides")
        rng = np.random.default_rng(11)
        worst = 0.0
        for _ in range(8):
            q = rng.uniform(-0.4, 1.0, len(self.jadr))
            for (name, adr), v in zip(self.jadr.items(), q):
                self.d.qpos[adr] = v
                rd.qpos[ref_adr[name]] = v
            mujoco.mj_forward(self.m, self.d)
            mujoco.mj_forward(ref, rd)
            for b in shared:
                im = self._body_ids(self.m, b)
                ir = self._body_ids(ref, b)
                worst = max(worst, float(np.max(np.abs(self.d.xpos[im] -
                                                       rd.xpos[ir]))))
        self.assertLess(worst, 1e-9,
                        f"conversion drifted from the URDF compiler by {worst:g} m")

    def test_conversion_unrolls_the_mounts_the_compiler_would_merge_away(self):
        """The point of converting: the mount chain stays addressable.

        The engine folds prototype_adapter and rebot_end_link into Link7, which
        is why the source URDF has no TCP body.  Here they stay real, and Link7
        keeps only its own mass.
        """
        for b in ("base_link", "prototype_adapter", "rebot_end_link"):
            self.assertGreaterEqual(
                mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, b), 0,
                f"{b} must survive as a body")
        i7 = self._body_ids(self.m, "Link7")
        ref = mujoco.MjModel.from_xml_path(str(mesh_sources().gripper_urdf))
        r7 = mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_BODY, "Link7")
        self.assertLess(float(self.m.body_mass[i7]),
                        float(ref.body_mass[r7]),
                        "Link7 must not have absorbed the gripper's mass")
        # base_link hangs off no joint, so the compiler folds it into the
        # (massless) world body and its mass disappears from body_mass.  Adding
        # it back is the only way the totals can agree, and agreeing is the
        # point: the conversion must neither lose nor invent mass.
        base = self._body_ids(self.m, "base_link")
        mine = float(np.sum(self.m.body_mass))
        theirs = float(np.sum(ref.body_mass)) + float(self.m.body_mass[base])
        self.assertAlmostEqual(mine, theirs, places=4,
                               msg="total mass must be conserved by the conversion")

    def test_tcp_site_exists_where_the_documented_mount_puts_it(self):
        """The prototype README states TCP ~= (0, -199.44, 0) mm from Link7."""
        it = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
        self.assertGreaterEqual(it, 0, "tcp_site must survive as a site")
        i7 = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "Link7")
        self.d.qpos[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        off = self.d.site_xpos[it] - self.d.xpos[i7]
        self.assertAlmostEqual(float(np.linalg.norm(off)), 0.19945, delta=2e-3)
        self.assertLess(abs(off[0]), 5e-3)
        self.assertAlmostEqual(off[1], -0.1994, delta=2e-3)

    def test_reference_sites_present(self):
        sites = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_SITE, i)
                 for i in range(self.m.nsite)}
        for want in ("tcp_site", "gripper_center_site",
                     "fingertip_left", "fingertip_right"):
            self.assertIn(want, sites)

    # -- dynamics of the coupling -------------------------------------------
    def _settle(self, ctrl_value, steps=2500):
        self.d.qpos[:] = 0.0
        self.d.qvel[:] = 0.0
        gi = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               "gripper_ctrl")
        self.d.ctrl[gi] = ctrl_value
        for _ in range(steps):
            mujoco.mj_step(self.m, self.d)
        return (self.d.qpos[self.jadr["rebot_finger_left"]],
                self.d.qpos[self.jadr["rebot_finger_right"]])

    def test_mimic_follows_exactly_through_the_single_channel(self):
        left, right = self._settle(0.05)
        self.assertAlmostEqual(left, 0.05, delta=2e-3)
        self.assertAlmostEqual(right, -left, delta=2e-3,
                               msg="right finger must mirror the left one")

    def test_opening_increases_the_fingertip_gap(self):
        it_l = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE,
                                 "fingertip_left")
        it_r = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE,
                                 "fingertip_right")
        gaps = []
        for ctrl in (0.0, 0.025, 0.05):
            self._settle(ctrl)
            mujoco.mj_forward(self.m, self.d)
            gaps.append(float(np.linalg.norm(self.d.site_xpos[it_l] -
                                             self.d.site_xpos[it_r])))
        self.assertGreater(gaps[-1], gaps[0] + 0.02,
                          f"fingertip gap must grow with the command: {gaps}")
        self.assertTrue(all(np.isfinite(g) for g in gaps))

    def test_no_divergence_and_no_penetration_at_the_travel_ends(self):
        for ctrl in (0.0, 0.05):
            self._settle(ctrl, steps=1500)
            self.assertTrue(np.all(np.isfinite(self.d.qpos)))
            self.assertLess(float(np.max(np.abs(self.d.qvel))), 1.0)
            self.assertLessEqual(self.d.qpos[self.jadr["rebot_finger_left"]],
                                 0.05 + 1e-6)
            self.assertGreaterEqual(self.d.qpos[self.jadr["rebot_finger_right"]],
                                    -0.05 - 1e-6)

    # -- reproducibility and portability ------------------------------------
    def test_generation_is_deterministic(self):
        with tempfile.TemporaryDirectory() as td:
            a = write_gripper_model(Path(td) / "a.xml")
            b = write_gripper_model(Path(td) / "b.xml")
            self.assertEqual(a.read_text(), b.read_text(),
                             "regenerating the same inputs must be byte-identical")

    def test_committed_model_has_no_absolute_paths(self):
        text = GRIPPER_MODEL_PATH.read_text()
        for token in ("/home/", "/Users/", "C:\\"):
            self.assertNotIn(token, text,
                             f"generated XML must not contain {token!r}")
        import xml.etree.ElementTree as ET
        root = ET.fromstring(text)
        meshdir = root.find("compiler").get("meshdir")
        self.assertFalse(Path(meshdir).is_absolute(),
                         f"meshdir must be relative, got {meshdir!r}")

    def test_every_simulation_assumption_is_labelled(self):
        prov = gripper_provenance()
        joined = " ".join(prov.values()).lower()
        self.assertIn("simulation assumption", joined)
        for key in ("gripper kp / kv", "fingertip friction 0.9/0.6",
                    "solimp/solref contact", "armature 0.01 per joint"):
            self.assertIn(key, prov)
        # The 8 N bound must never be described as a measured grasp force.
        self.assertIn("not a measured", prov["gripper force limit 8 N reading"].lower())


if __name__ == "__main__":
    unittest.main()
