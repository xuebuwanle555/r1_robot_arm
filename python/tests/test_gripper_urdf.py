"""Offline inventory check for the gripper-equipped R1 prototype model.

Standard-library only: no MuJoCo, no Pinocchio, no physics.  These tests pin the
*structure* of the combined model - joint set, types, mimic relation, travel, and
above all that its arm section still agrees with the frozen 7-joint model this
repository controls.

The asset lives outside this repository and is generated elsewhere, so the suite
skips when it is not present.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402
from r1_robot.gripper_model import ARM_JOINT_NAMES, load_facts, urdf_path  # noqa: E402

ASSET = urdf_path()
HAVE_ASSET = ASSET.exists()


@unittest.skipUnless(HAVE_ASSET, f"gripper prototype URDF not present at {ASSET}")
class TestGripperModelInventory(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f = load_facts()

    def test_tree_shape(self):
        self.assertEqual(len(self.f.links), 13, self.f.links)
        self.assertEqual(len(self.f.joints), 12)
        self.assertEqual(self.f.independent_dof, 8,
                         "7 arm joints + 1 independent finger travel")
        self.assertEqual([j.name for j in self.f.arm_joints], ARM_JOINT_NAMES)

    def test_joint_types(self):
        kinds = {j.name: j.type for j in self.f.joints}
        for n in ARM_JOINT_NAMES:
            self.assertEqual(kinds[n], "revolute", n)
        self.assertEqual(len(self.f.prismatic_joints), 2)
        fixed = [n for n, k in kinds.items() if k == "fixed"]
        self.assertEqual(sorted(fixed),
                         ["prototype_adapter_joint", "prototype_gripper_mount",
                          "prototype_tcp_joint"])

    def test_mimic_is_a_single_negative_pair(self):
        mim = self.f.mimic_joints
        self.assertEqual(len(mim), 1)
        self.assertEqual(mim[0].name, "rebot_finger_right")
        self.assertEqual(mim[0].mimic_of, "rebot_finger_left")
        self.assertEqual(mim[0].mimic_multiplier, -1.0)

    def test_finger_travel_is_symmetric_50_mm(self):
        left = self.f.joint("rebot_finger_left")
        right = self.f.joint("rebot_finger_right")
        self.assertAlmostEqual(left.lower, 0.0)
        self.assertAlmostEqual(left.upper, 0.05)
        self.assertAlmostEqual(right.lower, -0.05)
        self.assertAlmostEqual(right.upper, 0.0)

    def test_arm_limits_match_the_frozen_model_this_repo_controls(self):
        """The combined model must not quietly re-define the arm."""
        defaults = rr.r1_default_limits()
        for i, name in enumerate(ARM_JOINT_NAMES):
            j = self.f.joint(name)
            ours = defaults[i]
            self.assertAlmostEqual(j.lower, ours.lower, places=4,
                                   msg=f"{name} lower limit drifted")
            self.assertAlmostEqual(j.upper, ours.upper, places=4,
                                   msg=f"{name} upper limit drifted")

    def test_every_mesh_reference_resolves(self):
        self.assertGreater(len(self.f.mesh_files), 0)
        self.assertEqual(self.f.meshes_missing, [],
                         "absolute mesh paths must exist on this machine")

    def test_the_arm_uses_the_same_joint_names_as_the_sim(self):
        self.assertEqual(list(rr.JOINT_NAMES), ARM_JOINT_NAMES)

    def test_report_asset_identity(self):
        # Informational: printed so a review can tell which generated model was
        # checked.  Not pinned, because the generator legitimately iterates.
        print(f"\n  model   : {self.f.model_name}")
        print(f"  path    : {self.f.path}")
        print(f"  sha256  : {self.f.sha256[:16]}...")
        print(f"  links   : {len(self.f.links)}  joints: {len(self.f.joints)}")


class TestGripperModelAbsentIsNotAnError(unittest.TestCase):
    def test_module_imports_without_the_asset(self):
        # The module must stay importable on a machine that has no prototype
        # directory at all; only load_facts() needs the file.
        from r1_robot import gripper_model

        self.assertEqual(gripper_model.ARM_JOINT_NAMES[0], "J1")
        if not HAVE_ASSET:
            with self.assertRaises(FileNotFoundError):
                gripper_model.load_facts(ASSET)


if __name__ == "__main__":
    unittest.main()
