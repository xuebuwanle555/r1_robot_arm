# ---------------------------------------------------------------------------
# Python unit tests.  Offline only: MockArm / MockGripper, no hardware, no
# sleeping, no policy imports.
#
#   cd python && python -m pytest -q         (or)
#   cd python && python -m unittest discover -s tests -v
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import r1_robot as rr  # noqa: E402

LIMITS = rr.r1_default_limits()


class TestTypes(unittest.TestCase):
    def test_stop_is_inert(self):
        cmd = rr.ArmCommand.stop()
        self.assertTrue(cmd.is_stop())
        self.assertFalse(cmd.has_actuation())
        self.assertTrue(cmd.all_finite())

    def test_zero_torque_counts_as_actuation(self):
        cmd = rr.ArmCommand.zero_torque()
        self.assertFalse(cmd.is_stop())
        self.assertTrue(cmd.has_actuation())
        self.assertTrue(np.all(cmd.mode == rr.ControlMode.FOC))

    def test_position_command(self):
        cmd = rr.ArmCommand.position(np.full(7, 0.2), 30.0, 2.0)
        self.assertTrue(np.allclose(cmd.q, 0.2))
        self.assertTrue(np.allclose(cmd.kp, 30.0))
        self.assertTrue(np.allclose(cmd.kd, 2.0))

    def test_non_finite_detection(self):
        cmd = rr.ArmCommand.position(np.full(7, 0.2), 30.0, 2.0)
        cmd.q[2] = np.nan
        self.assertFalse(cmd.all_finite())

    def test_state_helpers(self):
        state = rr.RobotState()
        self.assertFalse(state.all_valid())
        self.assertEqual(state.invalid_joints(), list(range(7)))
        state.valid[:] = True
        self.assertTrue(state.all_valid())
        self.assertEqual(state.min_supply_voltage(), 0.0)  # unknown, not 0 V
        state.voltage[:] = 48.0
        state.voltage[4] = 41.0
        self.assertAlmostEqual(state.min_supply_voltage(), 41.0)


class TestLimits(unittest.TestCase):
    def test_limits_match_the_official_urdf(self):
        self.assertAlmostEqual(LIMITS[0].lower, -2.723)
        self.assertAlmostEqual(LIMITS[1].lower, 0.0)
        self.assertAlmostEqual(LIMITS[5].upper, 0.524)
        self.assertAlmostEqual(LIMITS[6].max_torque, 10.0)
        self.assertIn("UNVERIFIED", rr.R1_LIMITS_SOURCE)

    def test_check_limits_names_the_joint(self):
        q = np.full(7, 0.1)
        self.assertEqual(rr.check_limits(q, LIMITS), [])
        q[2] = 9.0
        problems = rr.check_limits(q, LIMITS)
        self.assertEqual(len(problems), 1)
        self.assertIn("joint 2", problems[0])

    def test_clamp_to_limits(self):
        q = np.full(7, 0.1)
        clamped, modified = rr.clamp_to_limits(q, LIMITS)
        self.assertFalse(modified)
        q[2] = 9.0
        clamped, modified = rr.clamp_to_limits(q, LIMITS)
        self.assertTrue(modified)
        self.assertAlmostEqual(clamped[2], LIMITS[2].upper)
        clamped, _ = rr.clamp_to_limits(q, LIMITS, margin=0.1)
        self.assertAlmostEqual(clamped[2], LIMITS[2].upper - 0.1)


class TestDefaultSafety(unittest.TestCase):
    def test_defaults_are_the_safe_values(self):
        """Phase-1 guarantee: the defaults never permit real hardware."""
        self.assertFalse(rr.SafetyConfig().allow_real_hardware)
        self.assertTrue(rr.SafetyConfig().require_calibration)
        self.assertTrue(rr.SafetyConfig().require_all_joints_valid)

    def test_default_backend_is_mock(self):
        robot = rr.R1Robot()
        self.assertIsInstance(robot.backend, rr.MockArm)
        self.assertFalse(robot.backend.is_real_hardware)


class TestR1RobotGating(unittest.TestCase):
    def _robot(self, **kwargs):
        robot = rr.R1Robot(**kwargs)
        robot.connect()
        return robot

    def test_actuation_is_refused_before_enable(self):
        robot = self._robot()
        with self.assertRaises(PermissionError) as ctx:
            robot.send_action(rr.ArmCommand.position(np.full(7, 0.2), 20.0, 3.0))
        self.assertIn("motion is not enabled", str(ctx.exception))
        self.assertFalse(robot.motion_enabled)

    def test_enable_requires_human_confirmation(self):
        robot = self._robot()
        with self.assertRaises(PermissionError) as ctx:
            robot.enable_motion()
        self.assertIn("human_confirmed", str(ctx.exception))
        robot.enable_motion(human_confirmed=True)
        self.assertTrue(robot.motion_enabled)

    def test_valid_command_is_accepted_once_enabled(self):
        backend = rr.MockArm()
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        backend.set_joint_positions(np.full(7, 0.18))
        robot.refresh()
        robot.enable_motion(human_confirmed=True)

        robot.send_action(rr.ArmCommand.position(np.full(7, 0.2), 20.0, 3.0))
        self.assertEqual(backend.command_count, 1)

    def test_commands_are_still_validated_after_enabling(self):
        backend = rr.MockArm()
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        backend.set_joint_positions(np.full(7, 0.18))
        robot.refresh()
        robot.enable_motion(human_confirmed=True)

        with self.assertRaises(ValueError) as ctx:
            robot.send_action(rr.ArmCommand.position(np.full(7, 0.3), 20.0, 3.0))
        self.assertIn("cmd.step_too_large", str(ctx.exception))

        with self.assertRaises(ValueError) as ctx:
            robot.send_action(rr.ArmCommand.position(np.full(7, 0.2), 5000.0, 3.0))
        self.assertIn("cmd.gain_out_of_range", str(ctx.exception))

        self.assertEqual(backend.command_count, 0)

    def test_stop_is_always_accepted(self):
        backend = rr.MockArm()
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        backend.set_feedback_valid_all(False)
        robot.refresh()
        robot.send_action(rr.ArmCommand.stop())
        self.assertEqual(backend.command_count, 1)

    def test_emergency_stop_disarms(self):
        backend = rr.MockArm()
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        backend.set_joint_positions(np.full(7, 0.18))
        robot.refresh()
        robot.enable_motion(human_confirmed=True)

        robot.emergency_stop()
        self.assertFalse(robot.motion_enabled)
        self.assertTrue(backend.last_command.is_stop())
        with self.assertRaises(PermissionError):
            robot.send_action(rr.ArmCommand.position(np.full(7, 0.2), 20.0, 3.0))

    def test_homing_is_blocked_in_phase_1(self):
        robot = self._robot()
        with self.assertRaises(NotImplementedError):
            robot.home()

    def test_real_backend_requires_the_master_switch(self):
        class FakeReal(rr.MockArm):
            @property
            def is_real_hardware(self) -> bool:
                return True

            @property
            def name(self) -> str:
                return "fake-real"

        robot = rr.R1Robot(backend=FakeReal())
        with self.assertRaises(PermissionError) as ctx:
            robot.connect()
        self.assertIn("allow_real_hardware", str(ctx.exception))

        allowed = rr.R1Robot(
            backend=FakeReal(), safety=rr.SafetyConfig(allow_real_hardware=True)
        )
        allowed.connect()
        self.assertFalse(allowed.motion_enabled)  # connecting enables nothing
        self.assertIn("motion_enabled=False", repr(allowed))

    def test_enable_refused_without_calibration(self):
        backend = rr.MockArm(rr.MockArmConfig(calibration_ok=False))
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        with self.assertRaises(PermissionError) as ctx:
            robot.enable_motion(human_confirmed=True)
        self.assertIn("enable.no_calibration", str(ctx.exception))

    def test_enable_refused_without_full_feedback(self):
        backend = rr.MockArm()
        robot = rr.R1Robot(backend=backend, limits=LIMITS)
        robot.connect()
        backend.set_feedback_valid(3, False)
        robot.refresh()
        with self.assertRaises(PermissionError) as ctx:
            robot.enable_motion(human_confirmed=True)
        self.assertIn("state.joint_invalid", str(ctx.exception))

    def test_state_checks(self):
        robot = rr.R1Robot()
        report = robot.check_state()
        self.assertFalse(report.ok())
        self.assertIn("state.not_connected", report.fault_codes())

        delayed = rr.R1Robot(backend=rr.MockArm(rr.MockArmConfig(feedback_delay_s=1.0)))
        delayed.connect()
        self.assertIn("state.stale_feedback", delayed.check_state().fault_codes())

        undervoltage = rr.R1Robot(
            backend=rr.MockArm(rr.MockArmConfig(supply_voltage=20.0))
        )
        undervoltage.connect()
        self.assertIn("state.undervoltage", undervoltage.check_state().fault_codes())

        faulty = rr.R1Robot(backend=rr.MockArm())
        faulty.connect()
        faulty.backend.inject_error(1, 0x8)
        faulty.refresh()
        self.assertIn("state.motor_error", faulty.check_state().fault_codes())


class TestMockArmDynamics(unittest.TestCase):
    def test_servos_toward_the_target(self):
        arm = rr.MockArm()
        arm.connect()
        arm.set_joint_positions(np.zeros(7))
        arm.write_command(rr.ArmCommand.position(np.full(7, 0.2), 20.0, 3.0))
        arm.advance(3.0)
        self.assertTrue(np.allclose(arm.read_state().q, 0.2, atol=0.02))

    def test_stop_bleeds_off_velocity(self):
        arm = rr.MockArm()
        arm.connect()
        arm.set_joint_velocities(np.full(7, 1.0))
        arm.write_command(rr.ArmCommand.stop())
        arm.advance(1.0)
        self.assertLess(abs(arm.read_state().dq[0]), 0.6)

    def test_clamps_to_the_urdf_limits(self):
        arm = rr.MockArm()
        arm.connect()
        arm.write_command(rr.ArmCommand.position(np.full(7, 9.0), 20.0, 3.0))
        arm.advance(3.0)
        q = arm.read_state().q
        for i, lim in enumerate(LIMITS):
            self.assertLessEqual(q[i], lim.upper + 1e-9)
            self.assertGreaterEqual(q[i], lim.lower - 1e-9)

    def test_refuses_commands_while_disconnected(self):
        arm = rr.MockArm()
        with self.assertRaises(RuntimeError):
            arm.write_command(rr.ArmCommand.stop())
        arm.advance(1.0)  # no-op, must not raise
        self.assertEqual(arm.simulated_time_ns, 0)


class TestGripper(unittest.TestCase):
    def test_dm4310_fails_closed(self):
        gripper = rr.Dm4310Gripper()
        with self.assertRaises(NotImplementedError):
            gripper.connect()
        with self.assertRaises(NotImplementedError):
            gripper.write_command(rr.GripperCommand())
        self.assertFalse(gripper.is_connected)
        self.assertFalse(gripper.state().valid)
        self.assertIn("UNVERIFIED", gripper.limits.source)

    def test_mock_gripper_round_trip(self):
        gripper = rr.MockGripper()
        gripper.connect()
        gripper.write_command(
            rr.GripperCommand(mode=rr.GripperCommandMode.POSITION, position=0.7)
        )
        self.assertAlmostEqual(gripper.state().position, 0.7)
        self.assertEqual(gripper.command_count, 1)
        gripper.stop()
        self.assertIs(gripper.last_command.mode, rr.GripperCommandMode.STOP)

    def test_mock_gripper_validates_its_envelope(self):
        gripper = rr.MockGripper()
        gripper.connect()
        with self.assertRaises(ValueError):
            gripper.write_command(
                rr.GripperCommand(mode=rr.GripperCommandMode.POSITION, position=1.5)
            )
        with self.assertRaises(ValueError):
            gripper.write_command(
                rr.GripperCommand(mode=rr.GripperCommandMode.CURRENT, current=99.0)
            )
        self.assertEqual(gripper.command_count, 0)


class TestConfig(unittest.TestCase):
    SAMPLE = """
# sample
backend = unitree          # inline comment
safety.allow_real_hardware = TRUE
safety.max_kp = 42.5
safety.max_feedback_age_s = not-a-number
hardware.port = /dev/ttyUSB0
hardware.baudrate = 6000000
"""

    def test_parses_keys_and_strips_comments(self):
        values = rr.parse_config(self.SAMPLE)
        self.assertEqual(values["backend"], "unitree")
        self.assertEqual(values["hardware.port"], "/dev/ttyUSB0")

    def test_rejects_a_line_without_an_equals_sign(self):
        with self.assertRaises(rr.ConfigError):
            rr.parse_config("backend unitree\n")

    def test_safety_config_from_values(self):
        values = rr.parse_config(self.SAMPLE)
        cfg = rr.safety_config_from(values)
        self.assertTrue(cfg.allow_real_hardware)
        self.assertAlmostEqual(cfg.max_kp, 42.5)
        # Falls back to the conservative default, never to 0.
        self.assertAlmostEqual(cfg.max_feedback_age_s, rr.SafetyConfig().max_feedback_age_s)

    def test_backend_selection(self):
        values = rr.parse_config(self.SAMPLE)
        sel = rr.backend_selection_from(values)
        self.assertEqual(sel.kind, "unitree")
        self.assertEqual(sel.baudrate, 6000000)

        default = rr.backend_selection_from({})
        self.assertEqual(default.kind, "mock")
        self.assertEqual(default.port, "/dev/ttyUSB3")

    def test_repository_config_file_is_loadable(self):
        path = Path(__file__).resolve().parents[2] / "config" / "r1_robot.conf"
        if not path.exists():
            self.skipTest("config/r1_robot.conf not found")
        values = rr.load_config(path)
        cfg = rr.safety_config_from(values)
        self.assertFalse(cfg.allow_real_hardware)  # shipped config is safe
        self.assertEqual(rr.backend_selection_from(values).kind, "mock")


if __name__ == "__main__":
    unittest.main(verbosity=2)
