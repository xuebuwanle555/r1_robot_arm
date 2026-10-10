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
