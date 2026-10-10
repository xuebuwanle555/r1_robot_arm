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

    def test_collision_geoms_exist_and_are_off_in_kinematics_mode(self):
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(self.model.ngeom)
        ]
        collisions = [n for n in names if n and n.endswith("_collision")]
        visuals = [n for n in names if n and n.endswith("_visual")]
        self.assertEqual(len(collisions), 8)
        self.assertEqual(len(visuals), 8)
        # The on-disk model is generated in the default "none" (pure-kinematics)
        # mode: every geom carries a zero contact mask.
        for i in range(self.model.ngeom):
            self.assertEqual(self.model.geom_contype[i], 0)
            self.assertEqual(self.model.geom_conaffinity[i], 0)

    def test_kinematics_mode_hides_a_real_collision(self):
        """"none" must NOT be used for contact / grasp experiments.

        The same pose that registers a tabletop collision in "environment" mode
        registers nothing here, because contact is compiled off.  This asserts the
        hazard explicitly so nobody re-enables the old blanket mask write.
        """
        arm = SimArm(SimArmConfig(collision_mode="none", add_table=True,
                                  gravity=(0, 0, 0), verify_model_on_connect=False))
        arm.connect()
        arm.set_joint_positions(rr.SAFE_HOME)
        self.assertEqual(arm.contact_count(), 0)

    def test_environment_mode_detects_a_designed_tabletop_collision(self):
        """Acceptance: MuJoCo detects a pre-designed collision scenario."""
        arm = SimArm(SimArmConfig(collision_mode="environment", add_table=True,
                                  gravity=(0, 0, 0), verify_model_on_connect=False))
        arm.connect()
        arm.set_joint_positions(rr.SAFE_HOME)  # links reach forward onto the table
        self.assertGreater(arm.contact_count(), 0)
        # Every contact involves the tabletop; no arm-vs-arm contact is possible.
        for a, b in arm.colliding_geoms():
            self.assertTrue("tabletop" in (a, b), msg=(a, b))
        self.assertEqual(arm.self_contact_pairs(), set())

    def test_environment_mode_nominal_pose_is_clear(self):
        """No fixture -> the stowed arm touches nothing but sits above the floor."""
        arm = SimArm(SimArmConfig(collision_mode="environment",
                                  gravity=(0, 0, 0), verify_model_on_connect=False))
        arm.connect()
        arm.set_joint_positions(rr.SAFE_HOME)
        self.assertEqual(arm.contact_count(), 0)

    def test_self_mode_nominal_is_clean_but_a_fold_is_detected(self):
        """Self-collision: adjacent/hull-artifact pairs are excluded so a nominal
        arm reads zero self contacts, yet a genuine fold (wrist across the upper
        arm) is detected."""
        arm = SimArm(SimArmConfig(collision_mode="self",
                                  gravity=(0, 0, 0), verify_model_on_connect=False))
        arm.connect()
        arm.set_joint_positions(rr.SAFE_HOME)
        self.assertEqual(arm.self_contact_pairs(), set())  # no hull false positives

        fold = np.array([0.72, 2.26, 0.1, 0.03, -1.87, 0.37, -1.47])
        arm.set_joint_positions(fold)
        self.assertGreater(len(arm.self_contact_pairs()), 0,
                           msg="a deeply folded pose must register self-contact")

    def test_runtime_mask_write_does_not_enable_contacts(self):
        """Regression for the silent no-op that hid this bug for a whole release.

        Writing geom_contype/conaffinity after the model is compiled does not
        start detecting contacts in MuJoCo 3.x; only the authored XML does.  This
        is why collision_mode is a *generation* parameter, not a runtime toggle.
        """
        arm = SimArm(SimArmConfig(collision_mode="none", add_table=True,
                                  gravity=(0, 0, 0), verify_model_on_connect=False))
        arm.connect()
        arm.set_joint_positions(rr.SAFE_HOME)
        # Blanket-enable every mask at runtime, exactly like the old buggy flag.
        arm.model.geom_contype[:] = 1
        arm.model.geom_conaffinity[:] = 1
        arm.set_joint_positions(rr.SAFE_HOME)  # re-run forward
        self.assertEqual(
            arm.contact_count(), 0,
            msg="runtime mask writes must not resurrect contact detection",
        )

    def test_controller_saturation_clamps_actuator_output(self):
        """A demand beyond the actuator ctrlrange is clamped (saturation)."""
        arm = SimArm(SimArmConfig(gravity=(0, 0, 0)))
        arm.connect()
        big = 1.0e9
        arm.data.ctrl[arm._act_ids] = big
        mujoco.mj_forward(arm.model, arm.data)
        # gaintype=fixed gain=1, so actuator_force == clamped ctrl.
        self.assertTrue(np.all(arm.data.actuator_force <= 1.0e5 + 1.0),
                        msg="actuator output did not saturate at ctrlrange")
        self.assertTrue(np.all(arm.data.actuator_force >= 0.0))

    def test_dynamics_provenance_is_fully_labelled(self):
        prov = rr.dynamics_provenance()
        for key in ("link_mass", "link_inertia", "joint_range", "joint_damping",
                    "joint_armature", "friction", "gear_ratio_and_stiffness",
                    "actuator_torque_limit", "collision_geometry"):
            self.assertIn(key, prov)
            self.assertTrue(prov[key])
        # Official vs assumption provenance is distinguishable.
        self.assertIn("official", prov["link_mass"])
        self.assertIn("hardware", prov["joint_damping"])
        self.assertIn("HARDWARE_UNVERIFIED", prov["gear_ratio_and_stiffness"])

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
        # assumed.  This test documents the arithmetic floor.  It is offline by
        # design: it must also run where MuJoCo is not installed.
        per_joint_delay_s = 115e-6
        joints = rr.NUM_JOINTS
        frame_floor_s = per_joint_delay_s * joints
        self.assertAlmostEqual(frame_floor_s, 805e-6, places=9)
        max_theoretical_rate_hz = 1.0 / frame_floor_s
        self.assertAlmostEqual(max_theoretical_rate_hz, 1242.2, places=0)

    @unittest.skipUnless(HAVE_DEPS, "MuJoCo / Pinocchio not available")
    def test_simulation_rate_is_not_the_hardware_rate(self):
        # The simulation runs at 500 Hz.  It is a different number from the
        # hardware floor above and the two must not be conflated.
        self.assertEqual(int(round(1.0 / SimArmConfig().timestep)), 500)


@unittest.skipUnless(HAVE_DEPS, "MuJoCo / Pinocchio not available")
class TestGravityLoadAtDriveRemoval(unittest.TestCase):
    """Task A, item 6: "the joints are still" is NOT evidence that the arm
    survives being de-energised.

    The controlled-stop logic used to conclude from `|dq| < tol` plus an elapsed
    hold window that it could drop to mode 0.  These tests pin down the physical
    consequence: at the instant the drive is removed the arm is perfectly still,
    and a fraction of a second later it is in free fall.  Only the torque keeps
    the pose up, so removing it is an operator decision, never a timeout.
    """

    HOLD_Q = np.array(rr.SAFE_HOME, dtype=float)

    def _sim(self) -> SimArm:
        sim = SimArm(SimArmConfig(verify_model_on_connect=False,
                                  collision_mode="none"))
        sim.connect()
        sim.set_joint_positions(self.HOLD_Q)
        return sim

    def _settle(self, sim: SimArm, seconds: float = 0.8) -> None:
        """Hold the pose under active FOC until it is genuinely still."""
        cmd = rr.ArmCommand.position(self.HOLD_Q, kp=80.0, kd=6.0)
        steps = int(seconds / sim.config.timestep)
        for _ in range(steps):
            sim.write_command(cmd)
            sim.advance(sim.config.timestep)

    def test_foc_hold_does_keep_the_pose_under_gravity(self):
        # Control case: with torque being produced, the pose is held.
        sim = self._sim()
        self._settle(sim)
        max_dq = float(np.max(np.abs(sim.joint_velocities)))
        drift = float(np.max(np.abs(sim.joint_positions - self.HOLD_Q)))
        self.assertLess(max_dq, 1e-3, "FOC hold should leave the arm still")
        self.assertLess(drift, 0.05, "FOC hold should keep the pose")

        # And it is holding against a real gravity load, not floating free.
        sim.write_command(rr.ArmCommand.stop())
        fall = self._fall_after_cut(sim, seconds=0.3)
        self.assertGreater(fall, 0.05,
                           "pose must not survive losing the holding torque")

    def test_zero_velocity_does_not_mean_the_pose_survives_the_cut(self):
        sim = self._sim()
        self._settle(sim)

        # The exact precondition the old stop logic treated as "settled".
        max_dq_at_cut = float(np.max(np.abs(sim.joint_velocities)))
        self.assertLess(max_dq_at_cut, 1e-3)

        sim.write_command(rr.ArmCommand.stop())  # mode 0: output removed
        self._advance(sim, 0.002, 1)
        self.assertGreater(float(np.max(np.abs(sim.joint_velocities))), 0.05,
                           "stillness at the cut did not predict the fall")
        self.assertGreater(self._fall_after_cut(sim, seconds=0.298), 0.05,
                           "a gravity-loaded arm drops when de-energised")

    def _advance(self, sim: SimArm, dt: float, steps: int) -> None:
        for _ in range(steps):
            sim.advance(dt)

    def _fall_after_cut(self, sim: SimArm, seconds: float) -> float:
        start = sim.joint_positions.copy()
        self._advance(sim, sim.config.timestep,
                      int(seconds / sim.config.timestep))
        return float(np.max(np.abs(sim.joint_positions - start)))


if __name__ == "__main__":
    unittest.main()
