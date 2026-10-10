# ---------------------------------------------------------------------------
# Integration test for the optional C++↔simulation bridge.
#
# Proves that the PRODUCTION C++ ControlService (the same class tested against
# MockJointBus in the unit suite) can drive the real MuJoCo SimArm physics,
# enforce its command-timeout watchdog against a REAL monotonic clock, latch a
# fault on upper-layer silence, refuse to move until an operator clears and
# re-enables, and then resume.
#
# Skips cleanly when the binding was not built
# (cmake -DR1_ROBOT_BUILD_PYTHON_BINDING=ON), when MuJoCo/Pinocchio are absent,
# or on any import error — so it never breaks the default / CI build.
#
# It advances a small amount of real wall time (the bridge shares one real
# clock for physics and the watchdog), which is the point: no virtual-clock
# stubbing of the safety path.
# ---------------------------------------------------------------------------
from __future__ import annotations

import time
import unittest

import numpy as np

try:
    import r1_robot as rr
    from r1_robot import _control_binding as cb

    # The bridge drives a real SimArm, which needs both optional sim engines at
    # construction time. Importing the binding alone is not enough: without this
    # the test fails instead of skipping on a machine that built the .so but has
    # no MuJoCo / Pinocchio.
    import mujoco  # noqa: F401
    import pinocchio  # noqa: F401

    HAVE_BRIDGE = True
except Exception:  # pragma: no cover - binding not built, or sim deps missing
    HAVE_BRIDGE = False

GOAL = np.array([0.2, 0.5, 0.1, 0.4, -0.2, 0.1, 0.2])


def _make_sim():
    return rr.SimArm(
        rr.SimArmConfig(
            gravity=(0.0, 0.0, 0.0),
            collision_mode="none",
            verify_model_on_connect=False,
        )
    )


def _config(timeout_s: float) -> "cb.ControlServiceConfig":
    cfg = cb.ControlServiceConfig()
    cfg.period_ns = 10_000_000          # 10 ms
    cfg.command_timeout_ns = int(timeout_s * 1e9)
    cfg.stop_settle_ns = 50_000_000
    cfg.stop_vel_tol = 0.2
    cfg.hold_kp = 60.0
    cfg.hold_kd = 5.0
    # A real (lagging) plant legitimately overshoots the conservative default
    # anti-step bound during tracking; loosen it for this integration demo.
    cfg.max_joint_step_rad = 0.5
    return cfg


@unittest.skipUnless(HAVE_BRIDGE, "C++/sim bridge not built")
class TestControlServiceSimBridge(unittest.TestCase):
    def test_cpp_service_drives_mujoco_then_survives_comm_loss(self):
        sim = _make_sim()
        service = cb.make_control_service(sim, _config(timeout_s=0.5))

        # connect + enable go through the C++ gates exactly as on hardware.
        ok, err = service.connect()
        self.assertTrue(ok, msg=err)
        sim.set_joint_positions(rr.SAFE_HOME)
        service.heartbeat()
        enable = service.enable_motion(human_confirmed=True)
        self.assertTrue(enable.ok, msg=enable.error)

        # Stream a 0.3 s trajectory; each cycle: heartbeat, C++ control cycle,
        # then step the MuJoCo plant.
        dur_ns = int(0.3e9)
        service.submit_trajectory(
            [(0, rr.SAFE_HOME.astype(np.float64)), (dur_ns, GOAL.astype(np.float64))]
        )
        dt = 0.002
        deadline = time.monotonic() + 1.5
        steps = 0
        while time.monotonic() < deadline and steps < 400:
            service.heartbeat()
            service.run_cycle()
            sim.advance(dt)
            time.sleep(dt)
            steps += 1
            if service.fault_latched():
                break

        self.assertFalse(service.fault_latched(),
                         msg=f"unexpected fault: {service.last_fault_code()}")
        q = sim.joint_positions
        self.assertLess(float(np.max(np.abs(q - GOAL))), 0.1,
                        msg=f"C++ loop did not drive physics to goal: {q}")

        # -- communication loss: the upper layer stops heartbeating ----------
        # command_timeout = 0.5 s; sleep past it with NO heartbeat, then cycle.
        time.sleep(0.7)
        service.run_cycle()

        self.assertTrue(service.fault_latched())
        self.assertEqual(service.last_fault_code(), "comm.watchdog_timeout")
        # It moves into the controlled stop and refuses to keep moving.
        self.assertEqual(service.state(), cb.ServiceState.CONTROLLED_STOP)
        self.assertEqual(service.stop_reason(), cb.StopReason.UPPER_LAYER_TIMEOUT)
        # Keep cycling well past the old 500 ms hold window: the stop must hold
        # under FOC and never drop the drive by itself, even with real physics.
        for _ in range(80):
            service.run_cycle()
            sim.advance(dt)
        self.assertEqual(service.state(), cb.ServiceState.CONTROLLED_STOP)
        # Re-enabling while latched is refused (a timeout never self-recovers).
        self.assertFalse(service.enable_motion(human_confirmed=True).ok)

        # An operator clears the fault; motion is only restored by a fresh,
        # human-confirmed enable.
        cleared, _ = service.clear_fault()
        self.assertTrue(cleared)
        self.assertEqual(service.state(), cb.ServiceState.STANDBY)
        self.assertFalse(service.fault_latched())
        self.assertTrue(service.enable_motion(human_confirmed=True).ok)

    def test_enable_requires_human_confirmation_through_the_bridge(self):
        sim = _make_sim()
        service = cb.make_control_service(sim, _config(timeout_s=0.5))
        ok, err = service.connect()
        self.assertTrue(ok, msg=err)
        self.assertFalse(service.enable_motion(human_confirmed=False).ok)
        self.assertFalse(service.fault_latched())


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(HAVE_BRIDGE, "C++/sim bridge not built")
class TestBridgeInputValidation(unittest.TestCase):
    """Task D: nothing is read out of a NumPy array before its shape, dtype,
    contiguity and element count have been checked, and every malformed upload
    turns into an error a Python caller can actually see."""

    def _bringup(self, timeout_s=0.5):
        """A connected, enabled service parked on a pose inside the soft limits.

        Order matters: connect() reloads SimArm at zero, and the zero pose is
        already outside J2/J4's soft margin, so the pose has to be set after it.
        """
        sim = _make_sim()
        service = cb.make_control_service(sim, _config(timeout_s=timeout_s))
        ok, err = service.connect()
        self.assertTrue(ok, msg=err)
        sim.set_joint_positions(rr.SAFE_HOME)
        service.heartbeat()
        res = service.enable_motion(human_confirmed=True)
        self.assertTrue(res.ok, msg=res.error)
        service.heartbeat()
        service.run_cycle()
        return service, sim

    def _submit(self, waypoints):
        service, _ = self._bringup()
        return service.submit_trajectory(waypoints)

    def test_valid_upload_reports_acceptance(self):
        r = self._submit([(0, rr.SAFE_HOME.astype(np.float64)),
                          (int(1e9), GOAL.astype(np.float64))])
        self.assertTrue(bool(r), msg=r.error)
        self.assertEqual(r.status, cb.SubmitStatus.ACCEPTED)

    def test_rejects_wrong_element_count(self):
        short = np.zeros(6, dtype=np.float64)
        with self.assertRaisesRegex((ValueError, TypeError), "7 elements"):
            self._submit([(0, short), (int(1e8), short)])

    def test_rejects_wrong_dtype(self):
        ints = np.zeros(7, dtype=np.int32)
        with self.assertRaisesRegex(TypeError, "dtype"):
            self._submit([(0, ints.astype(np.int32)), (int(1e8), ints)])

    def test_rejects_non_contiguous_and_multidimensional(self):
        wide = np.zeros(14, dtype=np.float64)
        strided = wide[::2]            # 7 elements, but not contiguous
        self.assertEqual(strided.shape, (7,))
        self.assertFalse(strided.flags["C_CONTIGUOUS"])
        with self.assertRaisesRegex(ValueError, "contiguous"):
            self._submit([(0, strided), (int(1e8), strided)])
        nested = np.zeros((7, 1), dtype=np.float64)
        with self.assertRaisesRegex(ValueError, "1-dimensional"):
            self._submit([(0, nested), (int(1e8), nested)])

    def test_rejects_malformed_waypoint_pairs(self):
        for bad in ([(0,)], [((0, GOAL, GOAL), GOAL)], ["not a pair"]):
            with self.assertRaises((ValueError, TypeError)):
                self._submit(bad)

    def test_rejects_negative_timestamps_and_non_arrays(self):
        with self.assertRaises(ValueError):
            self._submit([(-1, GOAL.astype(np.float64)),
                          (int(1e8), GOAL.astype(np.float64))])
        with self.assertRaises((TypeError, ValueError)):
            self._submit([(0, {"q": 1}), (int(1e8), GOAL.astype(np.float64))])

    def test_empty_and_too_short_plans_are_refused_not_raised(self):
        # A structurally fine upload that the C++ admission check rejects comes
        # back as a SubmitResult, because nothing was executed.
        r = self._submit([])
        self.assertFalse(bool(r))
        self.assertIn("waypoint", r.error)

    def test_non_finite_waypoint_is_refused_by_the_service(self):
        bad = GOAL.astype(np.float64).copy()
        bad[2] = np.nan
        r = self._submit([(0, rr.SAFE_HOME.astype(np.float64)), (int(1e9), bad)])
        self.assertFalse(bool(r))
        self.assertIn("finite", r.error)

    def test_a_plain_list_of_seven_floats_is_accepted(self):
        r = self._submit([(0, [0.0, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0]),
                          (int(1e9), list(GOAL))])
        self.assertTrue(bool(r), msg=r.error)


class _RaisingArm:
    """A backend whose actuating call explodes, standing in for a SimArm that
    dies mid-step.  The C++ side must surface it as a fault, not swallow it."""

    name = "raising-sim"

    def __init__(self):
        self._inner = _make_sim()

    def __getattr__(self, item):
        return getattr(self._inner, item)

    def write_command(self, cmd):
        raise RuntimeError("sim lost the plot")


@unittest.skipUnless(HAVE_BRIDGE, "C++/sim bridge not built")
class TestBridgeFailurePaths(unittest.TestCase):
    def test_python_exception_becomes_an_explicit_send_failure(self):
        arm = _RaisingArm()
        cfg = _config(timeout_s=0.5)
        service = cb.make_control_service(arm, cfg)
        ok, err = service.connect()
        self.assertTrue(ok, msg=err)
        # Park AFTER connect(): connect() reloads the model at zero, and the
        # zero pose is already outside J2/J4's soft margin, which would fault
        # for the wrong reason.
        arm.set_joint_positions(rr.SAFE_HOME)
        res = service.enable_motion(human_confirmed=True)
        self.assertTrue(res.ok, msg=res.error)
        service.heartbeat()
        service.run_cycle()

        self.assertTrue(service.fault_latched())
        self.assertEqual(service.last_fault_code(), "bus.send_failed")
        self.assertEqual(service.state(), cb.ServiceState.CONTROLLED_STOP)
        self.assertEqual(service.stop_reason(), cb.StopReason.UNSAFE_STATE)
        self.assertGreaterEqual(service.metrics().send_failures, 1)

    def test_bridge_backend_is_never_treated_as_real_hardware(self):
        # allow_real_hardware stays False, yet enabling works: the service sees a
        # simulation backend, so the hardware gates are simply not in play.
        sim = _make_sim()
        self.assertFalse(sim.is_real_hardware)
        cfg = _config(timeout_s=0.5)
        cfg.allow_real_hardware = False
        service = cb.make_control_service(sim, cfg)
        self.assertTrue(service.connect()[0])
        self.assertTrue(service.enable_motion(human_confirmed=True).ok)


@unittest.skipUnless(HAVE_BRIDGE, "C++/sim bridge not built")
class TestIndependentWatchdog(unittest.TestCase):
    """Task D, item 4: the watchdog has to be proven without Python cooperating.

    The synchronous `run_cycle()` tests schedule the C++ cycle from Python, so
    they can only ever show that the service reacts when it is *told* to look.
    Here the C++ service owns its own periodic thread and Python calls nothing at
    all after startup; the timeout must latch on its own.
    """

    def test_timeout_latches_with_no_run_cycle_call_from_python(self):
        sim = _make_sim()
        cfg = _config(timeout_s=0.3)
        cfg.period_ns = 5_000_000          # 5 ms
        service = cb.make_control_service(sim, cfg)

        ok, err = service.connect()
        self.assertTrue(ok, msg=err)
        sim.set_joint_positions(rr.SAFE_HOME)
        service.heartbeat()
        self.assertTrue(service.enable_motion(human_confirmed=True).ok)

        service.start()
        try:
            # Python goes completely silent: no heartbeat, no run_cycle.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not service.fault_latched():
                time.sleep(0.02)

            self.assertTrue(service.fault_latched(),
                            "the C++ thread never noticed the silence")
            self.assertEqual(service.last_fault_code(), "comm.watchdog_timeout")
            self.assertEqual(service.stop_reason(),
                             cb.StopReason.UPPER_LAYER_TIMEOUT)
            self.assertEqual(service.state(), cb.ServiceState.CONTROLLED_STOP)
            # A timeout never restores motion by itself.
            self.assertFalse(service.enable_motion(human_confirmed=True).ok)
            self.assertGreater(service.metrics().cycles, 0)
        finally:
            service.stop()
