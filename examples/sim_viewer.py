#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# examples/sim_viewer.py
#
# Open a MuJoCo viewer on the R1-7A and run a SAFE, pre-defined simulation
# trajectory.  This never touches real hardware: it only ever creates a SimArm
# (a MuJoCo backend that reports is_real_hardware == False).
#
# The trajectory is chosen so every target stays inside the safety soft limits
# (J2 and J4 have URDF lower bound 0, so their commandable lower bound is the
# 0.05 rad soft margin, not 0 - see the note in SIMULATION.md).
#
# Usage
# -----
#   # with a display (interactive MuJoCo viewer):
#   python examples/sim_viewer.py
#
#   # headless (no display / no GPU): runs the same trajectory, prints results
#   python examples/sim_viewer.py --headless
#
#   # options
#   python examples/sim_viewer.py --loop            # repeat until closed
#   python examples/sim_viewer.py --no-hold         # exit right after the move
#   python examples/sim_viewer.py --duration 1.0 --kp 80 --kd 5 --no-gravity
#
# Behaviour
# ---------
#   * interactive: the window opens, the trajectory runs once, and then the
#     window STAYS OPEN (--hold, the default) so the model can be inspected;
#     close the window (or Ctrl-C) to quit.  Use --no-hold to exit immediately
#     after the move, or --loop to repeat it.
#   * headless: runs the same trajectory, prints the result and exits.
#
# Known issue
# -----------
#   On some GL drivers (e.g. software rendering under Xvfb) MuJoCo's GLFW
#   passive viewer segfaults in its *global destructor* at interpreter
#   shutdown, after this script has already finished.  The entry point
#   therefore flushes and calls os._exit() to skip that teardown.  It does not
#   affect the run itself.
# ---------------------------------------------------------------------------
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

# Make the repository's python/ package importable when run from anywhere.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

import r1_robot as rr  # noqa: E402

# Safe poses: every joint inside [lower + 0.05, upper - 0.05].
HOME = np.array([0.0, 0.30, 0.30, 0.30, 0.0, 0.0, 0.0])
WAYPOINTS = [
    HOME,
    np.array([0.30, 0.60, 0.20, 0.50, -0.30, 0.20, 0.30]),
    np.array([-0.30, 0.40, 0.50, 0.20, 0.30, -0.20, -0.30]),
    HOME,
]


def check_waypoints(waypoints) -> None:
    """Fail fast if a waypoint is not commandable under the soft limits."""
    cfg = rr.SafetyConfig()
    for i, q in enumerate(waypoints):
        problems = rr.check_limits(q, margin=cfg.soft_limit_margin)
        if problems:
            raise SystemExit(
                f"waypoint {i} is not commandable under the safety limits:\n  "
                + "\n  ".join(problems)
            )


def build_trajectory(duration_per_segment: float) -> rr.JointTrajectory:
    traj = rr.JointTrajectory()
    for i, q in enumerate(WAYPOINTS):
        traj.add(i * duration_per_segment, q)
    return traj


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true",
                        help="do not open a viewer (no display / no GPU)")
    parser.add_argument("--duration", type=float, default=1.5,
                        help="seconds per trajectory segment (default 1.5)")
    parser.add_argument("--kp", type=float, default=80.0,
                        help="position gain [Nm/rad] (safety max 100)")
    parser.add_argument("--kd", type=float, default=5.0,
                        help="damping gain [Nm/(rad/s)] (safety max 5)")
    parser.add_argument("--dt", type=float, default=0.002,
                        help="control/physics step [s]")
    parser.add_argument("--no-gravity", action="store_true",
                        help="disable gravity in the simulation")
    parser.add_argument("--loop", action="store_true",
                        help="repeat the trajectory until the viewer is closed")
    parser.add_argument("--hold", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="keep the viewer open after the move (default: on "
                             "when a viewer is opened, off when headless)")
    args = parser.parse_args(argv)

    if not (0.0 < args.kp <= 100.0) or not (0.0 <= args.kd <= 5.0):
        parser.error("kp/kd outside the safety envelope (kp<=100, kd<=5)")

    check_waypoints(WAYPOINTS)

    kin = rr.R1Kinematics()
    sim_cfg = rr.SimArmConfig(
        gravity=(0.0, 0.0, 0.0) if args.no_gravity else (0.0, 0.0, -9.81)
    )
    arm = rr.SimArm(sim_cfg)
    arm.connect()

    robot = rr.R1Robot(backend=arm)
    robot.connect()
    arm.set_joint_positions(HOME)
    robot.refresh()
    robot.enable_motion(human_confirmed=True)

    print(f"backend        : {robot.backend.name} "
          f"(real_hardware={robot.backend.is_real_hardware})")
    print(f"joints         : {rr.JOINT_NAMES}")
    print(f"trajectory     : {len(WAYPOINTS)} waypoints, "
          f"{args.duration:.2f}s per segment, kp={args.kp}, kd={args.kd}")
    print("feed-forward   : Pinocchio gravity compensation")
    print("no real hardware is involved.\n")

    traj = build_trajectory(args.duration)
    executor = rr.TrajectoryExecutor(
        rr.ExecutorConfig(kp=args.kp, kd=args.kd, dt=args.dt,
                          settle_tol=0.01, extra_time_s=1.0)
    )

    viewer = None
    if not args.headless:
        try:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(arm.model, arm.data)
            print("viewer: opened. Close the window (or Ctrl-C) to quit.")
        except Exception as exc:  # no display, no GL, ...
            print(f"viewer: unavailable ({exc}); running headless.")

    hold = bool(args.hold) if args.hold is not None else (viewer is not None)

    def on_step(t, state):
        if viewer is not None:
            if not viewer.is_running():
                raise KeyboardInterrupt
            viewer.sync()
            time.sleep(args.dt)

    def keep_open(seconds_per_frame: float = 1.0 / 60.0) -> None:
        """Idle until the user closes the window (no commands are sent)."""
        while viewer is not None and viewer.is_running():
            viewer.sync()
            time.sleep(seconds_per_frame)

    try:
        while True:
            result = executor.run(
                robot, traj, tau_ff=kin.gravity_torques, on_step=on_step
            )
            p = kin.position(robot.state.q)
            print(f"result         : {result.outcome.value} "
                  f"(elapsed {result.elapsed_s:.2f}s, "
                  f"final error {result.final_error.max():.4f} rad)")
            print(f"end-effector   : [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m")

            if viewer is not None and hold:
                print("viewer: holding the final pose; close the window to exit.")
                keep_open()
                return 0 if result.ok else 1

            if not args.loop or result.outcome is not rr.Outcome.REACHED:
                return 0 if result.ok else 1
            # reset to HOME for the next loop
            arm.set_joint_positions(HOME)
            robot.refresh()
    except KeyboardInterrupt:
        print("\ninterrupted; stopping.")
        return 0
    finally:
        robot.disconnect()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # Skip interpreter teardown: MuJoCo's GLFW passive viewer can segfault in its
    # global destructor on some GL drivers, *after* this script has finished.
    os._exit(code)
