# ---------------------------------------------------------------------------
# Joint limits (URDF frame, SI units).
#
# PROVENANCE
#   Read from the official URDF shipped with the SDK:
#     urdf/R1_Robotic_Arm_V1.1-0915/.../urdf/R1_Robotic_Arm.urdf
#   => URDF-verified, hardware-UNVERIFIED.
#
# TODO(bringup) 待验证:
#   * `max_torque` is the URDF `effort` attribute - confirm against the real
#     actuators and the power supply.
#   * `max_velocity` is the URDF `velocity` attribute.
#   * The gravity-load direction of each joint is not verified.
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import NUM_JOINTS

R1_LIMITS_SOURCE = (
    "urdf/R1_Robotic_Arm_V1.1-0915/.../R1_Robotic_Arm.urdf "
    "(URDF-verified, hardware-UNVERIFIED)"
)

R1_LOWER = np.array([-2.723, 0.0, -0.349, 0.0, -2.269, -0.524, -1.571])
R1_UPPER = np.array([2.723, 3.1416, 3.491, 3.1415, 2.269, 0.524, 1.571])
R1_MAX_VELOCITY = np.array([18.7, 18.7, 33.4, 33.4, 33.4, 37.7, 37.7])
R1_MAX_TORQUE = np.array([60.0, 60.0, 33.0, 33.0, 33.0, 10.0, 10.0])

assert len(R1_LOWER) == NUM_JOINTS


@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float
    max_velocity: float
    max_torque: float


def r1_default_limits() -> tuple[JointLimits, ...]:
    """The limits used whenever no other table is supplied."""
    return tuple(
        JointLimits(
            float(R1_LOWER[i]),
            float(R1_UPPER[i]),
            float(R1_MAX_VELOCITY[i]),
            float(R1_MAX_TORQUE[i]),
        )
        for i in range(NUM_JOINTS)
    )


def _soft_bounds(lim: JointLimits, margin: float) -> tuple[float, float]:
    """Soft limits; collapse to the joint centre if the margin is too wide."""
    centre = 0.5 * (lim.lower + lim.upper)
    return min(lim.lower + margin, centre), max(lim.upper - margin, centre)


def check_limits(
    q, limits: tuple[JointLimits, ...] | None = None, margin: float = 0.0
) -> list[str]:
    """Return human-readable violations (empty list when q is inside)."""
    limits = limits or r1_default_limits()
    q = np.asarray(q, dtype=np.float64)
    problems: list[str] = []
    for i in range(NUM_JOINTS):
        if not np.isfinite(q[i]):
            problems.append(f"joint {i} value is not finite")
            continue
        lo, hi = _soft_bounds(limits[i], margin)
        if q[i] < lo or q[i] > hi:
            problems.append(
                f"joint {i} q={q[i]:.4f} rad outside "
                f"[{limits[i].lower}, {limits[i].upper}] (margin {margin})"
            )
    return problems


def clamp_to_limits(
    q, limits: tuple[JointLimits, ...] | None = None, margin: float = 0.0
) -> tuple[np.ndarray, bool]:
    """Clamp a copy of q into the limits; returns (q, was_modified)."""
    limits = limits or r1_default_limits()
    out = np.asarray(q, dtype=np.float64).copy()
    modified = False
    for i in range(NUM_JOINTS):
        lo, hi = _soft_bounds(limits[i], margin)
        if not np.isfinite(out[i]):
            out[i] = 0.5 * (limits[i].lower + limits[i].upper)
            modified = True
            continue
        clamped = float(np.clip(out[i], lo, hi))
        if clamped != out[i]:
            out[i] = clamped
            modified = True
    return out, modified
