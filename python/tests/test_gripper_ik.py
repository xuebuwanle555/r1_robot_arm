# ---------------------------------------------------------------------------
# Task C-0: is the kinematics the grasp planner uses actually right?
#
# The planner trusts three things it cannot see: that the tool frame offset came
# from the same XML the simulator compiled, that FK and the Jacobian agree with
# the engine, and that "converged" means converged rather than "stalled somewhere
# flattering".  Each is checked against an independent source here - MuJoCo for
# the geometry, finite differences for the Jacobian, FK-generated targets for the
# solver.
#
# Offline only.  Nothing in this file says anything about a real R1.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import mujoco

    from r1_robot.gripper_ik import (GripperKinematics, IKSolver, TCP_APPROACH_AXIS,
                                     TCP_OPENING_AXIS, WorkspaceSampler,
                                     default_seeds, make_pose, mat_to_quat_wxyz,
                                     pose_error, quat_wxyz_to_mat, tcp_offset)
    from r1_robot.gripper_mjcf import GRIPPER_MODEL_PATH
    from r1_robot.poses import SAFE_HOME, is_commandable
    HAVE = True
except Exception as exc:                                  # pragma: no cover
    HAVE = False
    _ERR = exc

RNG = np.random.default_rng(4)


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class ToolFrameTest(unittest.TestCase):
    def setUp(self):
        self.gk = GripperKinematics()
        self.m = mujoco.MjModel.from_xml_path(str(GRIPPER_MODEL_PATH))
        self.d = mujoco.MjData(self.m)
        self.qadr = {n: int(self.m.jnt_qposadr[mujoco.mj_name2id(
            self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in
            ("J1", "J2", "J3", "J4", "J5", "J6", "J7")}
        self.tcp_site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE,
                                          "tcp_site")
        self.tip = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, s)
                    for s in ("fingertip_left", "fingertip_right")]

    def _set(self, q):
        for name, adr in self.qadr.items():
            self.d.qpos[adr] = float(q[list(self.qadr).index(name)])
        mujoco.mj_forward(self.m, self.d)

    def test_fk_matches_mujoco_at_random_configurations(self):
        """Pinocchio's tool pose in the ARM frame == the engine's site in the world.

        The scene stands the arm on a deck, so the comparison is made after
        removing the base offset - a raw comparison differs by exactly that height,
        which is how this test first failed and why `SceneRig.to_base_frame` exists.
        """
        base_body = int(self.m.body_parentid[int(self.m.jnt_bodyid[
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "J1")])])
        worst_p = worst_R = 0.0
        for _ in range(12):
            q = RNG.uniform(-0.4, 0.9, 7)
            self._set(q)
            T = self.gk.fk(q)
            worst_p = max(worst_p, float(np.linalg.norm(
                T[:3, 3] - (self.d.site_xpos[self.tcp_site]
                            - self.d.xpos[base_body]))))
            worst_R = max(worst_R, float(np.abs(
                T[:3, :3] - self.d.site_xmat[self.tcp_site].reshape(3, 3)).max()))
            self.assertTrue(np.allclose(T[:3, 3],
                                        self.d.site_xpos[self.tcp_site]
                                        - self.d.xpos[base_body], atol=1e-9))
        self.assertLess(worst_p, 1e-6, f"FK TCP position off by {worst_p:g} m")
        self.assertLess(worst_R, 1e-6, f"FK TCP orientation off by {worst_R:g}")

    def test_tool_axes_are_orthonormal_and_rigid_in_the_tool_frame(self):
        """The approach and opening axes the planner hard-codes are real axes.

        They must stay unit length and mutually perpendicular at every posture -
        the planner multiplies them into target poses, and a non-orthogonal pair
        would make `make_pose` produce a rotation matrix that is not in SO(3).
        """
        for _ in range(8):
            q = RNG.uniform(-0.5, 1.0, 7)
            R = self.gk.fk(q)[:3, :3]
            a = R @ TCP_APPROACH_AXIS
            o = R @ TCP_OPENING_AXIS
            self.assertAlmostEqual(float(np.linalg.norm(a)), 1.0, places=12)
            self.assertAlmostEqual(float(np.linalg.norm(o)), 1.0, places=12)
            self.assertAlmostEqual(float(a @ o), 0.0, places=12)
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=9)

    def test_measured_tool_point_is_behind_the_fingertip_plane(self):
        """`tcp_site` sits BEHIND the pads, which is why targets are offset."""
        for _ in range(6):
            q = RNG.uniform(-0.3, 0.8, 7)
            self._set(q)
            tcp = self.d.site_xpos[self.tcp_site]
            tips = self.d.site_xpos[self.tip]
            mid = tips.mean(axis=0)
            R = self.d.site_xmat[self.tcp_site].reshape(3, 3)
            down = R @ TCP_APPROACH_AXIS
            behind = float((tcp - mid) @ -down)
            self.assertGreater(behind, 0.06,
                               f"tool point only {behind * 1e3:.0f} mm behind the "
                               f"tips - the offset the planner uses would be wrong")
            self.assertLess(behind, 0.11)

    def test_offset_agrees_with_the_compiled_model(self):
        """`tcp_offset` reads the XML; the engine compiles the same XML."""
        off = tcp_offset(GRIPPER_MODEL_PATH)
        q = np.zeros(7)
        self._set(q)
        link7 = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "Link7")
        T_link7 = np.eye(4)
        T_link7[:3, :3] = self.d.xmat[link7].reshape(3, 3)
        T_link7[:3, 3] = self.d.xpos[link7]
        mine = T_link7 @ off
        self.assertTrue(np.allclose(mine[:3, 3], self.d.site_xpos[self.tcp_site],
                                    atol=1e-9))


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class JacobianTest(unittest.TestCase):
    def setUp(self):
        self.gk = GripperKinematics()

    def test_jacobian_matches_finite_differences(self):
        q = RNG.uniform(-0.4, 0.9, 7)
        jac = self.gk.jacobian(q)
        h = 1e-6
        for j in range(7):
            qp, qm = q.copy(), q.copy()
            qp[j] += h
            qm[j] -= h
            dp = (self.gk.fk(qp)[:3, 3] - self.gk.fk(qm)[:3, 3]) / (2 * h)
            self.assertTrue(np.allclose(dp, jac[:3, j], atol=1e-5),
                            f"column {j} linear part: {dp} vs {jac[:3, j]}")


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class QuaternionRoundTripTest(unittest.TestCase):
    def test_random_rotations_survive_matrix_to_quat_to_matrix(self):
        for _ in range(300):
            a = RNG.normal(size=3)
            b = RNG.normal(size=3)
            c = RNG.normal(size=3)
            M, _ = np.linalg.qr(np.diag([1, 1, 1]) if abs(a @ b) < 1e-9
                                else np.outer(a, b) + np.outer(b, a))
            if np.linalg.det(M) < 0:
                M[:, 0] *= -1
            quat = mat_to_quat_wxyz(M)
            self.assertAlmostEqual(float(np.linalg.norm(quat)), 1.0, places=12)
            self.assertTrue(np.allclose(quat_wxyz_to_mat(quat), M, atol=1e-9),
                            f"round trip failed for {quat}")


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class SolverTest(unittest.TestCase):
    def setUp(self):
        self.gk = GripperKinematics()
        self.solver = IKSolver(self.gk)

    def test_targets_generated_by_fk_are_recovered(self):
        """The solver must reach poses the forward map produced, from a cold start."""
        for _ in range(5):
            q_true = np.clip(RNG.uniform(-0.4, 0.9, 7),
                             self.solver.lower, self.solver.upper)
            target = self.gk.fk(q_true)
            res = self.solver.solve_from_seeds(target, default_seeds(8))
            self.assertTrue(res.converged,
                            f"failed to recover a reachable pose: {res.problems}")
            self.assertLess(res.position_error_m, 2e-4)

    def test_an_unreachable_pose_is_reported_not_faked(self):
        target = self.gk.fk(np.zeros(7))
        target[:3, 3] += np.array([5.0, 0.0, 5.0])       # 7 m away: nothing reaches it
        res = self.solver.solve_from_seeds(target, default_seeds(4))
        self.assertFalse(res.ok)
        self.assertFalse(res.converged)
        self.assertGreater(res.position_error_m, 0.5)

    def test_a_pose_on_a_soft_limit_is_a_finding_even_when_reached(self):
        """`ok` is not just "the numbers matched": posture still has to be legal."""
        q = np.clip(self.solver.lower + 0.001, self.solver.lower,
                    self.solver.upper)
        res = self.solver.solve_from_seeds(self.gk.fk(q), (q,))
        self.assertTrue(res.converged, "the pose is reachable by construction")
        self.assertFalse(res.ok, "a pose inside the forbidden band must not be ok")
        self.assertTrue(any("outside" in p for p in res.problems))
        self.assertFalse(is_commandable(res.q))

    def test_pose_error_is_zero_for_the_same_pose(self):
        T = make_pose([0.3, 0.1, 0.8], (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
        dp, dth = pose_error(T, T)
        self.assertTrue(np.allclose(dp, np.zeros(3), atol=1e-12))
        self.assertTrue(np.allclose(dth, np.zeros(3), atol=1e-12))


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class WorkspaceSamplerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gk = GripperKinematics()
        cls.s = WorkspaceSampler(cls.gk, samples=8000, seed=0)

    def test_samples_are_in_limit_and_deterministic(self):
        self.assertTrue(np.all(self.s.q >= self.s.gk.kin.configuration_limits()[0]))
        self.assertTrue(np.all(self.s.q <= self.s.gk.kin.configuration_limits()[1]))
        again = WorkspaceSampler(self.gk, samples=8000, seed=0)
        self.assertTrue(np.array_equal(again.q, self.s.q))

    def test_reachable_reports_a_count_and_seeds_land_near_the_tip(self):
        """The tip-plane bookkeeping is what makes a grasp target mean anything."""
        q = np.asarray(SAFE_HOME, dtype=float)
        tip = self.s.tip[np.argmin(np.linalg.norm(self.s.q - q, axis=1))]
        self.assertGreaterEqual(self.s.reachable(tip, tip_tol_m=0.05), 1)
        seeds = self.s.seeds_for(tip, tip_tol_m=0.05)
        self.assertTrue(len(seeds) >= 1)
        for s in seeds:
            T = self.gk.fk(s)
            got = T[:3, 3] + 0.0826 * (T[:3, :3] @ TCP_APPROACH_AXIS)
            self.assertLess(float(np.linalg.norm(got - tip)), 0.06)

    def test_grasp_candidates_keep_the_jaw_horizontal_and_face_aligned(self):
        """A candidate must be a parallel-jaw grasp, not some passing orientation."""
        p = np.array([0.25, 0.05, 0.05])
        cands = self.s.grasp_candidates(p, count=8, tip_tol_m=0.12)
        # Whether the sampled set contains any downward, jaw-horizontal,
        # face-aligned pose near a point is a property of this arm and mount, and
        # the trials measure it; the loop below only constrains what a returned
        # candidate must satisfy.
        self.assertIsInstance(cands, list)
        for _q, app, jaw, _d in cands:
            self.assertLessEqual(abs(float(jaw[2])), 0.26 + 1e-9,
                                 f"jaw canted out of the horizontal plane: {jaw}")
            self.assertGreaterEqual(max(abs(float(jaw[0])), abs(float(jaw[1]))),
                                    0.98 - 1e-9,
                                    f"jaw not along a face normal: {jaw}")
            self.assertLessEqual(float(app[2]), -0.85 + 1e-9,
                                 f"pads not pointing at the table: {app}")

    def test_an_impossible_region_returns_no_candidates(self):
        self.assertEqual(self.s.grasp_candidates(np.array([9.0, 9.0, 9.0])), [])


if __name__ == "__main__":
    unittest.main()
