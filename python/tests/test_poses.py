# ---------------------------------------------------------------------------
# Pose definitions: kinematic zero vs simulation initial pose vs safe home.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402


class TestPoseDefinitions(unittest.TestCase):
    def test_kinematic_zero_is_not_commandable(self):
        """q = 0 is a valid URDF pose but NOT a valid command (soft margin)."""
        self.assertFalse(rr.is_commandable(rr.KINEMATIC_ZERO))
        problems = rr.pose_problems(rr.KINEMATIC_ZERO)
        self.assertEqual(len(problems), 2)  # J2 and J4 (lower bound 0)
        self.assertTrue(any("joint 1" in p for p in problems))
        self.assertTrue(any("joint 3" in p for p in problems))
        self.assertTrue(np.allclose(rr.KINEMATIC_ZERO, 0.0))

    def test_simulation_initial_pose_is_commandable(self):
        self.assertTrue(rr.is_commandable(rr.SIM_INITIAL_POSE))
        self.assertEqual(rr.pose_problems(rr.SIM_INITIAL_POSE), [])

    def test_safe_home_is_commandable_with_margin_to_spare(self):
        self.assertTrue(rr.is_commandable(rr.SAFE_HOME))
        # Strictly more than the bare minimum margin.
        self.assertGreater(rr.min_soft_margin(rr.SAFE_HOME), 0.0)

    def test_safe_home_keeps_static_hold_torque_small(self):
        """A home pose should not sit near a torque limit if it fails."""
        self.assertTrue(rr.static_hold_within_torque_limits(rr.SAFE_HOME, 0.5))

    def test_every_demo_waypoint_respects_the_soft_limits(self):
        self.assertEqual(rr.validate_trajectory_poses(rr.DEMO_TRAJECTORY_POSES), [])
        for q in rr.DEMO_TRAJECTORY_POSES:
            self.assertTrue(rr.is_commandable(q))

    def test_demo_trajectory_starts_and_ends_at_home(self):
        self.assertTrue(np.allclose(rr.DEMO_TRAJECTORY_POSES[0], rr.SIM_INITIAL_POSE))
        self.assertTrue(np.allclose(rr.DEMO_TRAJECTORY_POSES[-1], rr.SAFE_HOME))

    def test_describe_pose_reports_commandability(self):
        self.assertIn("commandable", rr.describe_pose(rr.SAFE_HOME))
        self.assertIn("NOT commandable", rr.describe_pose(rr.KINEMATIC_ZERO))

    def test_pose_problems_reports_each_violating_joint(self):
        bad = rr.SAFE_HOME.copy()
        bad[6] = 9.0  # way outside J7
        problems = rr.pose_problems(bad)
        self.assertEqual(len(problems), 1)
        self.assertIn("joint 6", problems[0])


if __name__ == "__main__":
    unittest.main()
