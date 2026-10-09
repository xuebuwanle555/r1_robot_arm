# ---------------------------------------------------------------------------
# Kinematics tests: the official URDF facts and Pinocchio FK/Jacobian.
#
# These are *independent* checks: the FK/Jacobian consistency test compares the
# analytic Pinocchio Jacobian against a finite-difference of Pinocchio FK, and
# the URDF facts are read from the XML directly (not via Pinocchio).
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402

try:
    from r1_robot.kinematics import R1Kinematics, urdf_facts

    HAVE_KIN = True
except Exception as _exc:  # pragma: no cover - missing optional dependency
    HAVE_KIN = False
    _KIN_ERROR = _exc


@unittest.skipUnless(HAVE_KIN, "Pinocchio / official URDF not available")
class TestUrdfFacts(unittest.TestCase):
    def test_joint_names_types_and_axes(self):
        facts = urdf_facts()
        self.assertEqual(facts.joint_names, rr.JOINT_NAMES)
        self.assertEqual(set(facts.joint_types), {"revolute"})
        expected_axes = np.array(
            [
                [0, 0, -1],
                [1, 0, 0],
                [0, 1, 0],
                [-1, 0, 0],
                [0, 1, 0],
                [-1, 0, 0],
                [0, 0, -1],
            ],
            dtype=float,
        )
        self.assertTrue(np.allclose(facts.axes, expected_axes))

    def test_limits_match_the_python_constants(self):
        facts = urdf_facts()
        self.assertTrue(np.allclose(facts.lower, rr.limits.R1_LOWER))
        self.assertTrue(np.allclose(facts.upper, rr.limits.R1_UPPER))
        self.assertTrue(np.allclose(facts.effort, rr.limits.R1_MAX_TORQUE))
        self.assertTrue(np.allclose(facts.velocity, rr.limits.R1_MAX_VELOCITY))
        self.assertTrue(rr.joint_limits_are_consistent_with_constants())

    def test_masses_and_inertia(self):
        facts = urdf_facts()
        self.assertEqual(facts.masses.shape, (rr.NUM_JOINTS,))
        self.assertTrue(np.all(facts.masses > 0))
        self.assertAlmostEqual(facts.total_mass, 4.3052, places=3)
        self.assertEqual(facts.inertia_matrices.shape, (rr.NUM_JOINTS, 3, 3))
        # Every inertia matrix must be symmetric positive definite.
        for i in range(rr.NUM_JOINTS):
            m = facts.inertia_matrices[i]
            self.assertTrue(np.allclose(m, m.T))
            self.assertTrue(np.all(np.linalg.eigvalsh(m) > 0))

    def test_mesh_and_collision_geometry_present(self):
        facts = urdf_facts()
        self.assertEqual(facts.n_collision, 8)  # base_link + Link1..Link7
        self.assertIn("Link7.STL", facts.mesh_files)
        self.assertEqual(len(facts.mesh_files), 8)


@unittest.skipUnless(HAVE_KIN, "Pinocchio / official URDF not available")
class TestPinocchioKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = R1Kinematics()

    def test_model_is_fixed_base_seven_revolute_joints(self):
        self.assertEqual(self.kin.nq, rr.NUM_JOINTS)
        self.assertEqual(self.kin.nv, rr.NUM_JOINTS)
        self.assertEqual(self.kin.joint_names, rr.JOINT_NAMES)
        lo, hi = self.kin.configuration_limits()
        self.assertTrue(np.allclose(lo, rr.limits.R1_LOWER))
        self.assertTrue(np.allclose(hi, rr.limits.R1_UPPER))

    def test_forward_kinematics_is_deterministic_and_finite(self):
        q = np.array([0.1, 0.4, -0.2, 0.3, -0.1, 0.05, 0.2])
        t1 = self.kin.forward_kinematics(q)
        t2 = self.kin.forward_kinematics(q)
        self.assertTrue(np.allclose(t1, t2))
        self.assertEqual(t1.shape, (4, 4))
        self.assertTrue(np.all(np.isfinite(t1)))
        # Bottom row of a homogeneous transform.
        self.assertTrue(np.allclose(t1[3], [0, 0, 0, 1]))
        # Rotation block is orthonormal.
        rot = t1[:3, :3]
        self.assertTrue(np.allclose(rot @ rot.T, np.eye(3), atol=1e-9))

    def test_jacobian_shape_and_numerical_consistency(self):
        rng = np.random.default_rng(0)
        lo, hi = self.kin.configuration_limits()
        for _ in range(5):
            q = lo + (hi - lo) * rng.random(rr.NUM_JOINTS)
            analytic = self.kin.jacobian(q)
            numeric = self.kin.numerical_jacobian(q)
            self.assertEqual(analytic.shape, (6, rr.NUM_JOINTS))
            self.assertTrue(
                np.allclose(analytic, numeric, atol=1e-5),
                msg=f"Jacobian mismatch at q={q}",
            )

    def test_gravity_torques_are_finite_and_bounded(self):
        for q in (np.zeros(7), np.array([0.0, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0])):
            tau = self.kin.gravity_torques(q)
            self.assertEqual(tau.shape, (rr.NUM_JOINTS,))
            self.assertTrue(np.all(np.isfinite(tau)))
            self.assertLess(float(np.max(np.abs(tau))), 10.0)


class TestSoftLimitZeroPoseFinding(unittest.TestCase):
    """Regression/documentation for a real finding.

    J2 and J4 have URDF lower bound 0.  With the default soft-limit margin
    (0.05 rad) the *commandable* lower bound is therefore 0.05, so q = 0 is
    inside the URDF limits but gets rejected by the safety layer.  The
    documented 'preferred zero pose' must not assume q = 0 is commandable.
    """

    def test_q_zero_is_rejected_for_j2_and_j4_by_the_soft_margin(self):
        cfg = rr.SafetyConfig()
        self.assertGreater(cfg.soft_limit_margin, 0.0)
        q = np.zeros(rr.NUM_JOINTS)
        problems = rr.check_limits(q, margin=cfg.soft_limit_margin)
        # J2 (index 1) and J4 (index 3) are the ones with lower bound 0.
        self.assertEqual(len(problems), 2)
        self.assertTrue(any("joint 1" in p for p in problems))
        self.assertTrue(any("joint 3" in p for p in problems))
        # Slightly above the margin it is accepted.
        q2 = np.full(rr.NUM_JOINTS, cfg.soft_limit_margin + 1e-6)
        self.assertEqual(rr.check_limits(q2, margin=cfg.soft_limit_margin), [])


if __name__ == "__main__":
    unittest.main()
