# ---------------------------------------------------------------------------
# Trajectory tests: the minimum-jerk profile and the executor's ability to tell
# *why* a motion did not finish (rejected / motor fault / timeout / not reached).
#
# All offline and deterministic: MockArm advances simulated time only when the
# executor asks it to, so no test sleeps.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402

# A pose inside the soft-limit margins (J2/J4 must stay above 0.05 rad).
HOME = np.array([0.0, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0])
GOAL = np.array([0.2, 0.5, 0.1, 0.4, -0.2, 0.1, 0.2])


def _robot(backend):
    backend.set_joint_positions(HOME)
    robot = rr.R1Robot(backend=backend)
    robot.connect()
    robot.enable_motion(human_confirmed=True)
    return robot


class TestJointTrajectory(unittest.TestCase):
    def test_endpoints_are_exact(self):
        traj = rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.0, GOAL)])
        self.assertAlmostEqual(traj.duration, 1.0)
        self.assertTrue(np.allclose(traj.sample(0.0), HOME))
        self.assertTrue(np.allclose(traj.sample(1.0), GOAL))
        self.assertTrue(np.allclose(traj.sample(-1.0), HOME))  # clamped

    def test_minimum_jerk_has_zero_velocity_at_the_ends(self):
        traj = rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.0, GOAL)])
        self.assertTrue(np.allclose(traj.sample_velocity(0.0), 0.0, atol=1e-9))
        self.assertTrue(np.allclose(traj.sample_velocity(1.0), 0.0, atol=1e-9))
        # Non-zero in the middle.
        self.assertGreater(
            float(np.max(np.abs(traj.sample_velocity(0.5)))), 0.0
        )

    def test_multiple_waypoints_are_monotone_in_time(self):
        traj = rr.JointTrajectory()
        traj.add(0.0, HOME).add(0.5, GOAL).add(1.0, HOME)
        self.assertAlmostEqual(traj.duration, 1.0)
        with self.assertRaises(ValueError):
            traj.add(0.5, HOME)  # non-increasing time


class TestExecutorOutcomes(unittest.TestCase):
    def test_reached_on_mock_arm(self):
        robot = _robot(rr.MockArm(rr.MockArmConfig()))
        executor = rr.TrajectoryExecutor(
            rr.ExecutorConfig(kp=60.0, kd=4.0, dt=0.002, settle_tol=0.02, extra_time_s=1.0)
        )
        result = executor.run(
            robot, rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.5, GOAL)])
        )
        self.assertIs(result.outcome, rr.Outcome.REACHED, msg=result.detail)
        self.assertTrue(result.ok)
        self.assertLess(result.final_error.max(), 0.02)

    def test_rejected_when_the_target_is_outside_the_soft_limits(self):
        robot = _robot(rr.MockArm(rr.MockArmConfig()))
        bad = HOME.copy()
        bad[5] = -0.5  # J6 soft lower bound is -0.524 + 0.05 = -0.474
        executor = rr.TrajectoryExecutor(rr.ExecutorConfig(dt=0.002))
        result = executor.run(
            robot, rr.JointTrajectory.from_waypoints([(0.0, HOME), (0.5, bad)])
        )
        self.assertIs(result.outcome, rr.Outcome.REJECTED)
        self.assertIn("safety", result.detail)

    def test_motor_fault_is_detected(self):
        backend = rr.MockArm(rr.MockArmConfig())
        robot = _robot(backend)  # connect + enable while the arm is healthy
        # The fault appears *after* motion is enabled, i.e. during a move.
        backend.inject_error(3, 0x8)  # transient undervoltage on J4
        executor = rr.TrajectoryExecutor(rr.ExecutorConfig(dt=0.002, kp=60.0, kd=4.0))
        result = executor.run(
            robot, rr.JointTrajectory.from_waypoints([(0.0, HOME), (1.0, GOAL)])
        )
        self.assertIs(result.outcome, rr.Outcome.MOTOR_FAULT, msg=result.detail)
        self.assertEqual(result.worst_joint, 3)
        self.assertIn("motor error", result.detail)

    def test_motor_fault_blocks_motion_enable(self):
        # A motor fault present *before* enabling must stop motion at the gate.
        backend = rr.MockArm(rr.MockArmConfig())
        backend.set_joint_positions(HOME)
        robot = rr.R1Robot(backend=backend)
        robot.connect()
        backend.inject_error(0, 0x8)  # connect() clears injected errors
        with self.assertRaises(PermissionError) as ctx:
            robot.enable_motion(human_confirmed=True)
        self.assertIn("motor_error", str(ctx.exception))

    def test_not_reached_with_a_steady_state_load(self):
        # Start at the drooped equilibrium of a pure PD controller under load:
        # droop = 2.0 Nm / 60 (Nm/rad) = 0.0333 rad.  The arm stays there, so
        # it never settles inside a tight tolerance, and the 0.0333 rad offset
        # is below the 0.05 rad anti-step limit (so it is not rejected).
        droop = 2.0 / 60.0
        backend = rr.MockArm(rr.MockArmConfig(gravity_torque=2.0))
        backend.set_joint_positions(HOME - droop)
        robot = rr.R1Robot(backend=backend)
        robot.connect()
        robot.enable_motion(human_confirmed=True)
        executor = rr.TrajectoryExecutor(
            rr.ExecutorConfig(kp=60.0, kd=5.0, dt=0.002, settle_tol=0.01,
                              extra_time_s=0.3)
        )
        result = executor.run(
            robot, rr.JointTrajectory.from_waypoints([(0.0, HOME)])
        )
        self.assertIs(result.outcome, rr.Outcome.NOT_REACHED, msg=result.detail)
        self.assertAlmostEqual(result.final_error.max(), droop, places=3)
        self.assertLess(result.final_error.max(), 0.05)  # never tripped anti-step

    def test_timeout_when_the_budget_is_too_small(self):
        robot = _robot(rr.MockArm(rr.MockArmConfig(gravity_torque=1.5)))
        executor = rr.TrajectoryExecutor(
            rr.ExecutorConfig(kp=60.0, kd=5.0, dt=0.002, timeout_s=0.3,
                              settle_tol=0.02)
        )
        # Budget (0.3 s) is far shorter than the trajectory (3.0 s).
        result = executor.run(
            robot, rr.JointTrajectory.from_waypoints([(0.0, HOME), (3.0, GOAL)])
        )
        self.assertIs(result.outcome, rr.Outcome.TIMEOUT, msg=result.detail)

    def test_feedforward_must_have_seven_values(self):
        robot = _robot(rr.MockArm(rr.MockArmConfig()))
        executor = rr.TrajectoryExecutor(rr.ExecutorConfig(dt=0.002))
        with self.assertRaises(ValueError):
            executor.run(
                robot,
                rr.JointTrajectory.from_waypoints([(0.0, HOME), (0.5, GOAL)]),
                tau_ff=lambda q: np.zeros(3),
            )


if __name__ == "__main__":
    unittest.main()
