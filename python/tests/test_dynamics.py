# ---------------------------------------------------------------------------
# Dynamics cross-validation: Pinocchio vs MuJoCo, torque limits, trajectory
# time scaling, collision configuration and control robustness.
#
# CONVENTIONS (verified here, do not change silently)
#   * Joint order is URDF order J1..J7 for both engines; nq = nv = 7, fixed base.
#   * `pin.computeGeneralizedGravity(model, data, q)` and MuJoCo's
#     `data.qfrc_bias` at qvel = 0 use the SAME sign convention: both are the
#     generalized force that the actuators must apply to HOLD q statically
#     (MuJoCo: qacc = M^-1 (tau - qfrc_bias), so tau = qfrc_bias at rest).
#     They agree to ~1e-13 in this suite - no sign flip, no index permutation.
#   * Angles in radians, torques in Nm; MuJoCo is configured with
#     angle="radian".
#   * MuJoCo's mass matrix additionally contains `dof_armature` (0.01 kg·m² per
#     joint, added in the generated model for numerical conditioning).
#     Pinocchio's CRBA does not know about it, so the comparison subtracts it.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402

try:
    import mujoco
    import pinocchio as pin

    from r1_robot.kinematics import R1Kinematics
    from r1_robot.sim import SIM_MODEL_PATH, SimArm, SimArmConfig

    HAVE_DEPS = True
except Exception as _exc:  # pragma: no cover
    HAVE_DEPS = False
    _DEPS_ERROR = _exc

NUM_CONFIGS = 120


@unittest.skipUnless(HAVE_DEPS, "MuJoCo / Pinocchio not available")
class TestDynamicsCrossValidation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = R1Kinematics()
        cls.model = mujoco.MjModel.from_xml_path(str(SIM_MODEL_PATH))
        cls.data = mujoco.MjData(cls.model)
        cls.lo, cls.hi = cls.kin.configuration_limits()
        cls.configs = [
            cls.lo + (cls.hi - cls.lo) * np.random.default_rng(seed).random(7)
            for seed in range(NUM_CONFIGS)
        ]

    def test_gravity_matches_mujoco_over_120_configurations(self):
        worst = 0.0
        worst_q = None
        for q in self.configs:
            g_pin = pin.computeGeneralizedGravity(self.kin.model, self.kin.data, q)
            self.data.qpos[:] = q
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            delta = float(np.max(np.abs(g_pin - self.data.qfrc_bias)))
            if delta > worst:
                worst, worst_q = delta, q
        self.assertLess(
            worst, 1e-8, msg=f"worst gravity mismatch {worst} at q={worst_q}"
        )

    def test_gravity_sign_and_order_at_a_reference_pose(self):
        """The two engines must agree joint-for-joint, not just in norm."""
        q = np.array([0.0, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0])
        g_pin = pin.computeGeneralizedGravity(self.kin.model, self.kin.data, q)
        self.data.qpos[:] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.assertTrue(np.allclose(g_pin, self.data.qfrc_bias, atol=1e-9))
        # Sign sanity: the shoulder joints carry the arm's weight, so holding
        # them needs a non-trivial positive torque; the distal joints barely do.
        self.assertGreater(abs(g_pin[1]), 1.0)
        self.assertGreater(abs(g_pin[3]), 1.0)
        self.assertLess(abs(g_pin[6]), 0.5)

    def test_mass_matrix_matches_after_removing_armature(self):
        armature = self.model.dof_armature.copy()
        worst = 0.0
        for q in self.configs[:40]:
            m_pin = pin.crba(self.kin.model, self.kin.data, q)
            self.data.qpos[:] = q
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            m_mj = np.zeros((self.model.nv, self.model.nv))
            mujoco.mj_fullM(self.model, self.data, m_mj)
            m_mj = m_mj - np.diag(armature)
            worst = max(worst, float(np.max(np.abs(m_pin - m_mj))))
        self.assertLess(worst, 1e-9, msg=f"worst mass-matrix mismatch {worst}")

    def test_mass_matrix_is_symmetric_positive_definite(self):
        for q in self.configs[:30]:
            m_pin = pin.crba(self.kin.model, self.kin.data, q)
            self.assertTrue(np.allclose(m_pin, m_pin.T, atol=1e-12))
            self.assertTrue(np.all(np.linalg.eigvalsh(m_pin) > 0.0))

    def test_total_mass_matches_the_urdf(self):
        facts = rr.urdf_facts()
        # MuJoCo body 0 is the world; bodies 1..8 are base_link + Link1..Link7.
        mujoco_arm_mass = float(sum(self.model.body_mass[i] for i in range(1, 9)))
        self.assertAlmostEqual(
            mujoco_arm_mass, facts.total_mass + 1.7759, places=6
        )  # + base_link

    def test_gravity_torques_stay_inside_the_urdf_effort_limits(self):
        """If this failed, the arm could not hold itself at some pose."""
        effort = np.asarray(rr.limits.R1_MAX_TORQUE, dtype=np.float64)
        for q in self.configs:
            g = np.abs(pin.computeGeneralizedGravity(self.kin.model, self.kin.data, q))
            self.assertTrue(
                np.all(g <= effort),
                msg=f"gravity torque {g} exceeds effort limits {effort} at q={q}",
            )


@unittest.skipUnless(HAVE_DEPS, "MuJoCo / Pinocchio not available")
class TestCollisionAndLimits(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(SIM_MODEL_PATH))

    def test_collision_geoms_exist_but_are_disabled_by_default(self):
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(self.model.ngeom)
        ]
        collisions = [n for n in names if n and n.endswith("_collision")]
        visuals = [n for n in names if n and n.endswith("_visual")]
        self.assertEqual(len(collisions), 8)
        self.assertEqual(len(visuals), 8)
        # Collisions are off in the generated model: phase-2 control does not
        # rely on contact yet, and mesh-mesh contact is expensive.
        for i in range(self.model.ngeom):
            name = names[i]
            if name and name.endswith("_floor"):
                continue
            self.assertEqual(self.model.geom_contype[i], 0)
            self.assertEqual(self.model.geom_conaffinity[i], 0)

    def test_enabling_collision_sets_the_masks(self):
        arm = SimArm(SimArmConfig(enable_collision=True))
        self.assertTrue(np.all(arm.model.geom_contype != 0))
        self.assertTrue(np.all(arm.model.geom_conaffinity != 0))

    def test_no_self_collision_is_detected_in_folded_poses(self):
        """Recorded finding, NOT a safety guarantee.

        Deeply folded configurations of this URDF produce no self-contact in
        MuJoCo, so self-collision limits are NOT validated by it.  If a future
        model/mesh change introduces contact, this test fails on purpose and
        forces the collision story to be revisited.
        """
        model = mujoco.MjModel.from_xml_path(str(SIM_MODEL_PATH))
        model.geom_contype[:] = 1
        model.geom_conaffinity[:] = 1
        data = mujoco.MjData(model)
        folded = [
            np.array([0.0, 0.05, 3.40, 3.00, 0.05, 0.00, 0.00]),
            np.array([0.0, 0.05, 3.45, 3.09, 0.05, 0.45, 0.00]),
            np.array([0.0, 1.60, 3.40, 3.00, 0.00, 0.00, 0.00]),
            np.array([0.0, 0.06, 3.44, 3.09, 0.06, 0.47, 1.50]),
        ]
        for q in folded:
            data.qpos[:] = q
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            self.assertEqual(data.ncon, 0, msg=f"self-contact appeared at q={q}")

    def test_joint_limits_are_enforced_by_the_simulation(self):
        """Limits must resist a 1 rad overshoot command.

        MuJoCo joint limits are *compliant* constraints, so a sustained push
        penetrates slightly (measured ~0.04 rad at a 20 Nm demand).  The
        assertions are therefore: the joint stops at the limit within a small
        tolerance, and it never approaches the commanded target.
        """
        arm = SimArm(SimArmConfig(gravity=(0.0, 0.0, 0.0)))
        arm.connect()
        lo = arm.model.jnt_range[:, 0].copy()
        hi = arm.model.jnt_range[:, 1].copy()

        arm.write_command(rr.ArmCommand.position(hi + 1.0, 20.0, 5.0))
        arm.advance(1.0)
        q = arm.joint_positions
        self.assertTrue(np.all(q <= hi + 0.05), msg=f"q={q} hi={hi}")
        self.assertTrue(np.all(q >= hi - 0.05), msg=f"q={q} hi={hi}")

        arm.write_command(rr.ArmCommand.position(lo - 1.0, 20.0, 5.0))
        arm.advance(1.0)
        q = arm.joint_positions
        self.assertTrue(np.all(q >= lo - 0.05), msg=f"q={q} lo={lo}")
        self.assertTrue(np.all(q <= lo + 0.05), msg=f"q={q} lo={lo}")


@unittest.skipUnless(HAVE_DEPS, "MuJoCo / Pinocchio not available")
class TestControlRobustness(unittest.TestCase):
    """The controller must stay stable - and honest - when the model is wrong."""

    def _run(self, *, payload=0.0, friction=0.0, inertia_scale=1.0,
             frictionloss=0.0):
        arm = SimArm(SimArmConfig())
        # Perturb the plant only; the gravity feed-forward still uses the
        # NOMINAL Pinocchio model, exactly like a real controller would.
        if payload > 0.0:
            bid = arm.model.body("Link7").id
            arm.model.body_mass[bid] += payload
        if friction > 0.0:
            arm.model.dof_frictionloss[:] = friction
        if frictionloss > 0.0:
            arm.model.dof_frictionloss[:] = frictionloss
        if inertia_scale != 1.0:
            arm.model.dof_armature[:] *= inertia_scale
        arm.connect()
        robot = rr.R1Robot(backend=arm)
        arm.set_joint_positions(rr.SAFE_HOME)
        robot.refresh()
        robot.enable_motion(human_confirmed=True)
        kin = R1Kinematics()
        traj = rr.JointTrajectory.from_waypoints(
            [(0.0, rr.SAFE_HOME), (1.5, np.array([0.2, 0.5, 0.1, 0.4, -0.2, 0.1, 0.2]))]
        )
        executor = rr.TrajectoryExecutor(
            rr.ExecutorConfig(kp=60.0, kd=5.0, dt=0.002, settle_tol=0.02,
                              extra_time_s=1.5)
        )
        return executor.run(robot, traj, tau_ff=kin.gravity_torques)

    def test_nominal_case_reaches_the_target(self):
        result = self._run()
        self.assertIs(result.outcome, rr.Outcome.REACHED, msg=result.detail)

    def test_unknown_payload_stays_bounded(self):
        # 0.5 kg added to the tip, uncompensated: the arm sags but must not
        # diverge or oscillate. The outcome is expected to be NOT_REACHED on
        # the tight tolerance - that is the honest result, not a failure.
        result = self._run(payload=0.5)
        self.assertIn(
            result.outcome, (rr.Outcome.REACHED, rr.Outcome.NOT_REACHED)
        )
        self.assertLess(result.final_error.max(), 0.2, msg=result.detail)

    def test_joint_friction_stays_bounded(self):
        result = self._run(frictionloss=0.5)
        self.assertIn(
            result.outcome, (rr.Outcome.REACHED, rr.Outcome.NOT_REACHED)
        )
        self.assertLess(result.final_error.max(), 0.2, msg=result.detail)

    def test_inertia_estimate_error_stays_stable(self):
        result = self._run(inertia_scale=5.0)
        self.assertIn(
            result.outcome, (rr.Outcome.REACHED, rr.Outcome.NOT_REACHED)
        )
        self.assertLess(result.final_error.max(), 0.2, msg=result.detail)


class TestControlFrequencyAssumptions(unittest.TestCase):
    """The simulation timestep is NOT a claim about the real command rate."""

    def test_sdk_inter_write_delay_bounds_the_command_rate(self):
        # ArmMotorBus::step() sleeps 115 us after EVERY joint write (verified in
        # the SDK source), so one full 7-joint frame costs at least
        # 7 * 115 us = 805 us of enforced delay, before serial time and motor
        # turnaround.  A 500 Hz loop (2 ms period) therefore has less than
        # 1.2 ms for everything else and must be MEASURED on hardware, not
        # assumed.  This test documents the arithmetic floor.
        per_joint_delay_s = 115e-6
        joints = rr.NUM_JOINTS
        frame_floor_s = per_joint_delay_s * joints
        self.assertAlmostEqual(frame_floor_s, 805e-6, places=9)
        max_theoretical_rate_hz = 1.0 / frame_floor_s
        self.assertAlmostEqual(max_theoretical_rate_hz, 1242.2, places=0)
        # The simulation, by contrast, runs at 500 Hz (2 ms).  They are
        # different numbers and must not be conflated.
        self.assertEqual(int(round(1.0 / SimArmConfig().timestep)), 500)


if __name__ == "__main__":
    unittest.main()
