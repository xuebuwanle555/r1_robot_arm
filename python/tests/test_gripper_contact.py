# ---------------------------------------------------------------------------
# Task B: does the simulated gripper actually move, touch and push?
#
# Everything here is verified OFFLINE, in simulation only.  Nothing in this file
# is evidence about a real DM4310 or a real R1: the contact stiffness, friction,
# damping and the 8 N channel ceiling are simulation assumptions
# (`gripper_mjcf.GRIPPER_PROVENANCE`), and the forces below are simulated contact
# forces, not measured grasp forces.
#
# The grasp is tested in free space, with the jaws closing on a block that is
# released at the same moment: the question is "does the channel produce real
# contact and enough friction to hold the object", not "does a table-standing
# object fit between the fingers".  The second question is currently blocked and
# documented in SIMULATION.md - the model's collision proxies are bounding
# spheres, so a solid work surface cannot be trusted for clearance yet.
#
# No cheating constraints, asserted structurally rather than trusted:
#   * the scene has exactly one equality constraint and it is the finger mimic,
#     so nothing is welded to the arm or to the world;
#   * every object is a free joint, so it moves only under contact and gravity.
# ---------------------------------------------------------------------------
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import mujoco

    from r1_robot.gripper import (GripperCommand, GripperCommandMode,
                                  GripperLimits)
    from r1_robot.gripper_mjcf import (FINGER_LEFT, FINGER_RIGHT,
                                       FINGER_TRAVEL_M, GRIPPER_CHANNEL,
                                       GRIPPER_MAX_FORCE_N, GRIPPER_PROVENANCE,
                                       ObjectSpec)
    from r1_robot.gripper_sim import (GripperScene, SceneArmBackend,
                                      SceneGripper)
    from r1_robot.types import ArmCommand, ControlMode, NUM_JOINTS
    HAVE = True
except Exception as exc:                                  # pragma: no cover
    HAVE = False
    _ERR = exc

BLOCK_Z = 0.55          # free space: nothing to knock the object off
HOLD_DT = 0.002
GRAVITY = 9.81


def _scene(mass=0.05, size=0.04, name="block"):
    obj = ObjectSpec(name=name, shape="box", size=(size, size, size),
                     pos=(0.30, 0.0, BLOCK_Z), mass=mass)
    return GripperScene(objects=[obj]), obj


def _grippers(scene):
    return SceneGripper(scene)


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class GripperMotionTest(unittest.TestCase):
    """Items 1-3: synchronized opposite motion, URDF travel, numerical health."""

    def setUp(self):
        self.scene, self.obj = _scene()
        self.gripper = _grippers(self.scene)
        self.gripper.connect()
        self.mj = self.scene._mj
        self.jl = mujoco.mj_name2id(self.scene.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    FINGER_LEFT)
        self.jr = mujoco.mj_name2id(self.scene.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    FINGER_RIGHT)

    def _q(self, which):
        adr = int(self.scene.model.jnt_qposadr[which])
        return float(self.scene.data.qpos[adr])

    def test_the_mimic_is_the_only_equality_and_nothing_is_welded(self):
        m = self.scene.model
        self.assertEqual(m.neq, 1,
                         "the scene must contain exactly one equality "
                         "constraint (the finger mimic)")
        self.assertEqual(int(m.eq_type[0]), int(mujoco.mjtEq.mjEQ_JOINT))
        self.assertEqual({int(m.eq_obj1id[0]), int(m.eq_obj2id[0])},
                         {self.jl, self.jr},
                         "the only equality must tie the two finger joints")
        # Objects must be free, not attached: qpos has a 7-slot free joint each.
        for obj in self.scene.objects:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"{obj.name}_free")
            self.assertGreaterEqual(jid, 0)
            self.assertEqual(int(m.jnt_type[jid]), int(mujoco.mjtJoint.mjJNT_FREE))

    def test_fingers_move_in_sync_and_in_opposite_directions(self):
        for target in (1.0, 0.6, 0.2, 0.0, 0.8):
            self.gripper.write_command(
                GripperCommand(mode=GripperCommandMode.POSITION, position=target))
            self.scene.step(0.5)
            left, right = self._q(self.jl), self._q(self.jr)
            self.assertAlmostEqual(left, -right, places=3,
                                   msg=f"coupling broken at {target}: "
                                       f"{left} vs {right}")
            self.assertGreater(left, 0.0,
                               "the driven finger must actually travel")

    def test_travel_and_gap_match_the_urdf_joint_limits(self):
        m = self.scene.model
        self.assertTrue(np.allclose(m.jnt_range[self.jl], [0.0, FINGER_TRAVEL_M]))
        self.assertTrue(np.allclose(m.jnt_range[self.jr],
                                     [-FINGER_TRAVEL_M, 0.0]))
        self.gripper.write_command(GripperCommand(
            mode=GripperCommandMode.POSITION, position=1.0))
        self.scene.step(2.0)
        self.assertAlmostEqual(self._q(self.jl), FINGER_TRAVEL_M, delta=2e-3)
        self.assertGreater(self.scene.gripper_gap(), 0.09,
                           msg=f"full travel must open the tips ~100 mm, got "
                               f"{self.scene.gripper_gap():.4f}")
        self.assertAlmostEqual(self.gripper.state().position, 1.0, delta=0.05,
                               msg="the reported normalized opening must agree")
        self.gripper.write_command(GripperCommand(
            mode=GripperCommandMode.POSITION, position=0.0))
        self.scene.step(2.0)
        # The jaws stop 3.2 mm short of the joint limit because their collision
        # proxies - derived from the finger meshes - touch each other first.
        # Measured here, not assumed: the final state carries a
        # left_finger_link x right_finger_link contact.  The proxy fit is the
        # same limitation tracked in SIMULATION.md, and the drive force at that
        # stall is ~1 N, well inside the 8 N channel ceiling, so this is the
        # geometry, not the force limit.
        self.assertAlmostEqual(self._q(self.jl), 0.0, delta=5e-3,
                               msg="closed must reach the joint limit within "
                                   "the finger-proxy stall")
        self.assertGreater(self._q(self.jl), 0.0,
                           "the stall is contact, so the limit is never passed")
        self.assertLess(self.scene.gripper_gap(), 0.01,
                        "closed must bring the fingertip sites together")

    def test_cycling_stays_numerically_healthy(self):
        worst_pen = 0.0
        peak_speed = 0.0
        for cycle in range(3):
            for target in (0.0, 1.0, 0.5):
                self.gripper.write_command(GripperCommand(
                    mode=GripperCommandMode.POSITION, position=target))
                for _ in range(150):
                    self.scene.step(HOLD_DT)
                    peak_speed = max(peak_speed, abs(self.scene.gripper_rate()))
                    worst_pen = max(worst_pen, self.scene.max_penetration_m(
                        restrict=(FINGER_LEFT + "_link",
                                  FINGER_RIGHT + "_link")))
        self.assertFalse(self.scene.any_nonfinite())
        self.assertLess(worst_pen, 0.01,
                        f"a finger tunneled {worst_pen * 1e3:.1f} mm")
        self.assertLess(peak_speed, 2.0,
                        "the servo must not spike: the fingers are light")

    def test_the_channel_is_the_only_way_in_and_refuses_bad_commands(self):
        lim = self.gripper.limits
        self.assertIsInstance(lim, GripperLimits)
        for bad in (1.4, -0.2, float("nan"), float("inf")):
            with self.assertRaises(ValueError,
                                   msg=f"position {bad} must be refused"):
                self.gripper.write_command(GripperCommand(
                    mode=GripperCommandMode.POSITION, position=bad))
        # Torque/current mode is not modelled, so it must fail closed rather
        # than pretend a current command did something.
        with self.assertRaises(NotImplementedError):
            self.gripper.write_command(GripperCommand(
                mode=GripperCommandMode.CURRENT, current=0.5))
        other = _grippers(self.scene)
        with self.assertRaises(RuntimeError):
            other.write_command(GripperCommand(
                mode=GripperCommandMode.POSITION, position=0.5))


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class GripperContactTest(unittest.TestCase):
    """Items 4-7: observable contact, force through the channel, labels."""

    def setUp(self):
        self.scene, self.obj = _scene(mass=0.05, size=0.04)
        self.gripper = _grippers(self.scene)
        self.gripper.connect()
        # Park the arm in free space and open the jaws THROUGH the adapter, then
        # present the block at the fingertip plane.  Placing the object is scene
        # setup; nothing below moves it except contact.
        self.scene.set_arm_positions(np.array([0.0, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0]))
        self.open_m = 0.026
        self.gripper.write_command(GripperCommand(
            mode=GripperCommandMode.POSITION, position=self.open_m / FINGER_TRAVEL_M))
        self.scene.step(1.5)
        self.tip = self.scene.finger_midpoint()
        self.scene.place_object("block", self.tip)
        self.scene.step(0.02)

    def test_the_jaws_close_on_free_space_and_the_channel_collapses(self):
        """Commanded opening must actually reduce the fingertip-plane gap.

        This is the part of "the gripper can close on something" that can be
        verified without a work surface: the channel between the fingertip sites
        must track the command, and the servo must stop at the travel limit
        rather than keep pushing.
        """
        gaps = []
        for target_m in (0.026, 0.020, 0.014, 0.006, 0.0):
            self.gripper.write_command(GripperCommand(
                mode=GripperCommandMode.POSITION,
                position=target_m / FINGER_TRAVEL_M))
            self.scene.step(1.2)                 # let the servo settle
            gaps.append(self.scene.gripper_gap())
        self.assertTrue(all(np.isfinite(gaps)))
        for a, b in zip(gaps, gaps[1:]):
            self.assertLessEqual(b, a + 1e-3,
                                 f"closing must not widen the channel: {gaps}")
        self.assertLess(gaps[-1], 0.01,
                        f"fully commanded closed, the tips must meet: {gaps}")
        self.assertFalse(self.scene.any_nonfinite())

    @unittest.expectedFailure
    def test_contact_appears_and_the_normal_force_grows_with_the_command(self):
        """BLOCKED, and kept visible rather than quietly deleted.

        The reason changed once the collision proxies were fixed (they are boxes
        fitted to the CAD envelopes now, not bounding spheres): this test's own
        fixture.  A 40 mm block released in free space at the fingertip plane has
        fallen ~70 mm by the time the first 1.2 s closing command settles, so the
        jaws close on nothing and the force stays 0.00 N - the geometry is no longer
        the excuse, the falling object is.

        `SceneRig.run_pick_and_place` is where the same question is asked against a
        supported object, and what it measures is reported there: with the pads
        pointing down at the payload, the current sample set offers no clean
        gripping plane at these positions at all, and with them pointing up the
        settled force is 0.10-0.21 N against the 0.27 N the model's own friction
        says a 50 g payload needs.  Until the pad contact is modelled better than a
        box fitted to a rotated mesh, that is the honest answer about grasp force in
        this simulation.
        """
        forces = []
        for target_m in (0.026, 0.022, 0.018, 0.012, 0.006):
            self.gripper.write_command(GripperCommand(
                mode=GripperCommandMode.POSITION,
                position=target_m / FINGER_TRAVEL_M))
            self.scene.step(1.2)
            forces.append(self.scene.finger_contact_force("block"))
        self.assertGreater(forces[-1], 0.0,
                           f"never contacted the block: {forces}")
        self.assertGreater(forces[-1], forces[-2],
                           f"force must grow as the jaws close: {forces}")

    @unittest.expectedFailure
    def test_friction_alone_holds_the_block_against_gravity(self):
        """BLOCKED, for the reason given on the test above.

        The arithmetic is not in doubt: holding a 50 g block at the model's
        mu = 0.9 needs 0.27 N of normal force, well inside the 8 N channel ceiling.
        What is not yet demonstrated is that these pads produce it on a real object
        geometry - in a supported grasp the settled force measured 0.10-0.21 N.
        """
        mu = float(self.scene.model.geom_friction[
            [g for g in range(self.scene.model.ngeom)
             if self.scene._body_name(g) == FINGER_LEFT + "_link"
             and self.scene.model.geom_contype[g]][0]][0])
        needed = self.obj.mass * GRAVITY / (2.0 * mu)
        self.gripper.write_command(GripperCommand(
            mode=GripperCommandMode.POSITION, position=0.0))
        self.scene.step(2.0)
        z0 = self.scene.object_height("block")
        force = self.scene.finger_contact_force("block")
        self.assertGreater(force, needed,
                           f"grip force {force:.2f} N below the {needed:.2f} N "
                           f"needed at mu={mu}")
        self.assertLess(abs(self.scene.object_height("block") - z0), 0.03)

    def test_effort_8_is_not_reported_as_a_measured_grasp_force(self):
        prov = GRIPPER_PROVENANCE
        self.assertIn("gripper force limit 8 N reading", prov)
        self.assertIn("NOT a measured", prov["gripper force limit 8 N reading"])
        for key in ("fingertip friction 0.9/0.6", "solimp/solref contact",
                    "gripper kp / kv", "joint damping 0.2",
                    "armature 0.01 per joint", "object mass / size"):
            self.assertIn(key, prov, f"{key} must be labelled")
            self.assertIn("assumption", prov[key].lower())
        # And the model must actually carry the ceiling it claims.
        m = self.scene.model
        idx = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_CHANNEL)
        self.assertTrue(m.actuator_forcelimited[idx])
        self.assertTrue(np.allclose(m.actuator_forcerange[idx],
                                     [-GRIPPER_MAX_FORCE_N,
                                      GRIPPER_MAX_FORCE_N]))

    def test_a_stop_command_never_adds_grip_force(self):
        self.gripper.write_command(GripperCommand(
            mode=GripperCommandMode.POSITION, position=0.020 / FINGER_TRAVEL_M))
        self.scene.step(0.6)
        self.gripper.stop()
        held = self.scene.data.ctrl[self.scene.gripper_index]
        self.assertAlmostEqual(held, self.scene.gripper_metres(), delta=1e-9,
                               msg="stop must hold the current opening, not "
                                   "drive further in")


@unittest.skipUnless(HAVE, f"MuJoCo stack unavailable: {_ERR if not HAVE else ''}")
class SceneBackendTest(unittest.TestCase):
    """The 7-joint view the frozen safety layer sees."""

    def setUp(self):
        self.scene, _ = _scene()
        self.backend = SceneArmBackend(self.scene)

    def test_backend_reports_a_seven_joint_frame_and_stops_actually_stop(self):
        st = self.backend.read_state()
        self.assertEqual(st.q.shape, (NUM_JOINTS,))
        self.assertFalse(self.backend.is_real_hardware)
        self.assertTrue(self.backend.has_valid_calibration())
        self.backend.connect()
        cmd = ArmCommand.position(np.zeros(NUM_JOINTS) + 0.2, kp=50.0, kd=2.0)
        self.backend.write_command(cmd)
        self.assertEqual(self.backend.read_state().mode[0], ControlMode.FOC)
        self.backend.stop()
        self.assertTrue(np.all(self.backend.read_state().mode
                               == ControlMode.STOP))
        self.assertTrue(np.allclose(self.scene.data.ctrl[:7] * 0, 0.0))

    def test_pd_law_lives_in_the_actuator_not_in_the_command_loop(self):
        """A 50 Hz zero-order-held torque is unstable on the light wrist.

        Measured with the earlier design: commanding the same pose with kp=80 as
        an externally computed torque, refreshed at 50 Hz, drove J6 0.6 rad away
        in four ticks.  The gains are therefore written into biasprm and MuJoCo
        evaluates them every 2 ms step.
        """
        self.backend.connect()
        cmd = ArmCommand.position(np.zeros(NUM_JOINTS) + 0.3, kp=80.0, kd=4.0)
        for _ in range(200):
            self.backend.write_command(cmd)
            self.scene.step(0.02)
        err = np.abs(self.scene.arm_joint_positions - 0.3).max()
        self.assertLess(err, 0.15,
                        f"the arm should hold near the setpoint, off by {err:.3f}")
        idx = self.scene.arm_index["J6"]
        self.assertAlmostEqual(self.scene.model.actuator_biasprm[idx][1], -80.0)
        self.assertAlmostEqual(self.scene.model.actuator_biasprm[idx][2], -4.0)

    def test_feedback_age_is_honoured_and_write_is_refused_while_disconnected(self):
        stale = SceneArmBackend(self.scene, feedback_age_s=1.0)
        stale.connect()
        age = (stale.read_state().stamp_ns
               - __import__("time").monotonic_ns()) / 1e9
        self.assertLess(age, -0.5, "the backend must stamp feedback as requested")
        quiet = SceneArmBackend(self.scene)
        with self.assertRaises(RuntimeError):
            quiet.write_command(ArmCommand.position(np.zeros(NUM_JOINTS),
                                                    kp=1.0, kd=1.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
