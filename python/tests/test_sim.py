# ---------------------------------------------------------------------------
# Simulation tests: the generated MuJoCo model and the SimArm backend.
#
# Everything here is headless (no rendering, no GPU, no hardware).  The key
# cross-check is MuJoCo FK vs Pinocchio FK: two independent parsers of the same
# URDF must agree, which catches a model-generation regression.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402

try:
    import mujoco  # noqa: F401
    from r1_robot.kinematics import R1Kinematics
    from r1_robot.sim import (
        SIM_MODEL_PATH,
        SimArm,
        SimArmConfig,
        generate_mjcf,
        write_sim_model,
    )

    HAVE_SIM = True
except Exception as _exc:  # pragma: no cover - missing optional dependency
    HAVE_SIM = False
    _SIM_ERROR = _exc

# Poses are centralised in r1_robot.poses (safe home / sim initial pose).
HOME = rr.SAFE_HOME
GOAL = np.array([0.2, 0.5, 0.1, 0.4, -0.2, 0.1, 0.2])


@unittest.skipUnless(HAVE_SIM, "MuJoCo / Pinocchio not available")
class TestGeneratedModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model_path = write_sim_model()
        cls.model = mujoco.MjModel.from_xml_path(str(cls.model_path))
        cls.kin = R1Kinematics()

    def test_dimensions(self):
        self.assertEqual(self.model.nq, rr.NUM_JOINTS)
        self.assertEqual(self.model.nv, rr.NUM_JOINTS)
        self.assertEqual(self.model.nu, rr.NUM_JOINTS)  # one actuator per joint
        self.assertEqual(self.model.nbody, 9)  # world + base_link + Link1..7
        # 8 visual + 8 collision + 1 floor
        self.assertEqual(self.model.ngeom, 17)

    def test_actuators_are_named_after_the_joints(self):
        for name in rr.JOINT_NAMES:
            self.assertEqual(
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_ctrl"
                )
                >= 0,
                True,
            )

    def test_joint_ranges_match_the_urdf(self):
        for i, name in enumerate(rr.JOINT_NAMES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertGreaterEqual(jid, 0)
            lo, hi = self.model.jnt_range[jid]
            self.assertAlmostEqual(float(lo), float(rr.limits.R1_LOWER[i]), places=6)
            self.assertAlmostEqual(float(hi), float(rr.limits.R1_UPPER[i]), places=6)

    def test_body_masses_match_the_urdf(self):
        facts = rr.urdf_facts()
        for i, name in enumerate([f"Link{k}" for k in range(1, 8)]):
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(bid, 0)
            self.assertAlmostEqual(
                float(self.model.body_mass[bid]), float(facts.masses[i]), places=6
            )

    def test_fk_matches_pinocchio_at_several_poses(self):
        data = mujoco.MjData(self.model)
        rng = np.random.default_rng(1)
        lo, hi = self.kin.configuration_limits()
        for _ in range(5):
            q = lo + (hi - lo) * rng.random(rr.NUM_JOINTS)
            data.qpos[:] = q
            mujoco.mj_forward(self.model, data)
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Link7")
            p_mj = data.xpos[bid]
            quat_mj = data.xquat[bid]
            p_pin = self.kin.position(q)
            rot_pin = self.kin.forward_kinematics(q)[:3, :3]

            self.assertTrue(np.allclose(p_mj, p_pin, atol=1e-6), msg=f"q={q}")
            # Compare rotations via the trace (quaternion sign is ambiguous).
            w, x, y, z = quat_mj
            rot_mj = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ]
            )
            self.assertTrue(
                np.allclose(rot_mj, rot_pin, atol=1e-6), msg=f"rotation q={q}"
            )

    def test_generate_mjcf_is_deterministic(self):
        a = generate_mjcf()
        b = generate_mjcf()
        self.assertEqual(a, b)
        self.assertEqual(a.count("<general "), rr.NUM_JOINTS)


@unittest.skipUnless(HAVE_SIM, "MuJoCo / Pinocchio not available")
class TestSimArm(unittest.TestCase):
    def _arm(self, **kwargs):
        cfg = SimArmConfig(**kwargs)
        arm = SimArm(cfg)
        arm.connect()
        return arm

    def test_sim_arm_is_never_real_hardware(self):
        arm = self._arm()
        self.assertFalse(arm.is_real_hardware)
        self.assertEqual(arm.name, "sim")
        self.assertTrue(arm.has_valid_calibration())

    def test_calibration_flag_is_honoured(self):
        arm = self._arm(calibration_ok=False)
        self.assertFalse(arm.has_valid_calibration())

    def test_stop_leaves_the_joint_free_under_gravity(self):
        arm = self._arm()
        arm.set_joint_positions(HOME)
        arm.write_command(rr.ArmCommand.stop())
        start = arm.joint_positions.copy()
        arm.advance(0.5)
        # With no actuator force and gravity on, the arm must fall.
        self.assertGreater(float(np.max(np.abs(arm.joint_positions - start))), 0.01)

    def test_zero_gravity_and_stop_holds_position(self):
        arm = self._arm(gravity=(0.0, 0.0, 0.0))
        arm.set_joint_positions(HOME)
        arm.write_command(rr.ArmCommand.stop())
        arm.advance(0.5)
        self.assertTrue(np.allclose(arm.joint_positions, HOME, atol=1e-6))

    def test_position_control_reaches_the_target_smoothly(self):
        kin = R1Kinematics()
        arm = self._arm()
        robot = rr.R1Robot(backend=arm)
        arm.set_joint_positions(HOME)
        robot.refresh()
        robot.enable_motion(human_confirmed=True)

        traj = rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.5, GOAL)])
        executor = rr.TrajectoryExecutor(
            rr.ExecutorConfig(kp=60.0, kd=4.0, dt=0.002, settle_tol=0.01, extra_time_s=1.5)
        )
        # Gravity feed-forward (Pinocchio dynamics) removes the steady-state droop.
        result = executor.run(robot, traj, tau_ff=kin.gravity_torques)
        self.assertIs(result.outcome, rr.Outcome.REACHED, msg=result.detail)
        self.assertTrue(
            np.allclose(robot.state.q, GOAL, atol=0.01),
            msg=f"final q={robot.state.q}",
        )

    def test_position_control_is_smooth_no_jumps(self):
        kin = R1Kinematics()
        arm = self._arm()
        robot = rr.R1Robot(backend=arm)
        arm.set_joint_positions(HOME)
        robot.refresh()
        robot.enable_motion(human_confirmed=True)

        traj = rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.0, GOAL)])
        dt = 0.002
        prev = arm.joint_positions.copy()
        max_step = 0.0
        for t in np.arange(0.0, traj.duration, dt):
            q_des = traj.sample(t)
            cmd = rr.ArmCommand.position(q_des, 60.0, 4.0)
            cmd.tau[:] = kin.gravity_torques(q_des)  # track closely, no droop
            robot.send_action(cmd)
            robot.refresh()
            arm.advance(dt)
            cur = arm.joint_positions.copy()
            max_step = max(max_step, float(np.max(np.abs(cur - prev))))
            prev = cur
        # A smooth minimum-jerk move never jumps: well below the 0.05 rad
        # anti-step limit per control cycle.
        self.assertLess(max_step, 0.01, msg=f"max per-cycle step {max_step}")


if __name__ == "__main__":
    unittest.main()
