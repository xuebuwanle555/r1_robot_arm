# ---------------------------------------------------------------------------
# Configuration reader.
#
# Reads the SAME "key = value" file as the C++ side (config/r1_robot.conf) so
# there is a single source of truth for the safety thresholds.  The default
# values below must stay in sync with include/r1_robot/safety.hpp.
# ---------------------------------------------------------------------------
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .safety import SafetyConfig

_TRUE = {"true", "1", "yes", "on"}
_FALSE = {"false", "0", "no", "off"}


@dataclass
class BackendSelection:
    kind: str = "mock"
    port: str = "/dev/ttyUSB3"
    baudrate: int = 6_000_000
    calib_path: str = ""


class ConfigError(ValueError):
    pass


def parse_config(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ConfigError(f"line {lineno}: expected 'key = value', got {line!r}")
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.split("#", 1)[0].strip()
        if not key:
            raise ConfigError(f"line {lineno}: empty key")
        values[key] = value
    return values


def load_config(path: str | Path) -> dict[str, str]:
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot open config file: {path}") from exc
    return parse_config(text)


def _as_float(values: dict[str, str], key: str, fallback: float) -> float:
    if key not in values:
        return fallback
    try:
        return float(values[key])
    except ValueError:
        return fallback


def _as_bool(values: dict[str, str], key: str, fallback: bool) -> bool:
    if key not in values:
        return fallback
    token = values[key].strip().lower()
    if token in _TRUE:
        return True
    if token in _FALSE:
        return False
    return fallback


def safety_config_from(values: dict[str, str]) -> SafetyConfig:
    defaults = SafetyConfig()
    return SafetyConfig(
        allow_real_hardware=_as_bool(
            values, "safety.allow_real_hardware", defaults.allow_real_hardware
        ),
        require_all_joints_valid=_as_bool(
            values, "safety.require_all_joints_valid", defaults.require_all_joints_valid
        ),
        require_calibration=_as_bool(
            values, "safety.require_calibration", defaults.require_calibration
        ),
        max_feedback_age_s=_as_float(
            values, "safety.max_feedback_age_s", defaults.max_feedback_age_s
        ),
        soft_limit_margin=_as_float(
            values, "safety.soft_limit_margin", defaults.soft_limit_margin
        ),
        max_joint_step_rad=_as_float(
            values, "safety.max_joint_step_rad", defaults.max_joint_step_rad
        ),
        max_kp=_as_float(values, "safety.max_kp", defaults.max_kp),
        max_kd=_as_float(values, "safety.max_kd", defaults.max_kd),
        max_tau=_as_float(values, "safety.max_tau", defaults.max_tau),
        max_abs_velocity=_as_float(
            values, "safety.max_abs_velocity", defaults.max_abs_velocity
        ),
        min_supply_voltage=_as_float(
            values, "safety.min_supply_voltage", defaults.min_supply_voltage
        ),
    )


def backend_selection_from(values: dict[str, str]) -> BackendSelection:
    return BackendSelection(
        kind=values.get("backend", "mock"),
        port=values.get("hardware.port", "/dev/ttyUSB3"),
        baudrate=int(_as_float(values, "hardware.baudrate", 6_000_000)),
        calib_path=values.get("hardware.calib_path", ""),
    )
