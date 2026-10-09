"""r1_robot - Python API for the Unitree R1-7A research stack (phase 1).

Scope of the Python layer
-------------------------
Planning, datasets, policy inference and analysis.  Python does NOT own the
real-time control loop: the hardware backends live in C++ where the safety
gates and the 6 Mbaud serial transport are implemented.  The only backend
available here is MockArm, which is offline and deterministic.

Safety model (mirrors C++ R1Arm)
--------------------------------
* MockArm is the default; real hardware is never selected implicitly.
* `SafetyConfig.allow_real_hardware` defaults to False.
* `R1Robot.enable_motion(human_confirmed=True)` must be called explicitly.
* Every actuating command is re-validated; a stop command is always accepted.
* No homing, no automatic zero-torque, no automatic enabling.
"""

from .backends import ArmBackend, MockArm, MockArmConfig, NullBackend
from .config import (
    BackendSelection,
    ConfigError,
    backend_selection_from,
    load_config,
    parse_config,
    safety_config_from,
)
from .gripper import (
    Dm4310Gripper,
    Gripper,
    GripperCommand,
    GripperCommandMode,
    GripperLimits,
    GripperState,
    MockGripper,
)
from .limits import (
    R1_LIMITS_SOURCE,
    JointLimits,
    check_limits,
    clamp_to_limits,
    r1_default_limits,
)
from .kinematics import (
    END_EFFECTOR_FRAME,
    LINK_NAMES,
    OFFICIAL_URDF,
    R1Kinematics,
    UrdfFacts,
    joint_limits_are_consistent_with_constants,
    official_urdf_path,
    urdf_facts,
)
from .robot import R1Robot, R1Status
from .safety import (
    SafetyConfig,
    SafetyIssue,
    SafetyLevel,
    SafetyMonitor,
    SafetyReport,
)
from .sim import SimArm, SimArmConfig, generate_mjcf, write_sim_model
from .trajectory import (
    ExecutionResult,
    ExecutorConfig,
    JointTrajectory,
    Outcome,
    TrajectoryExecutor,
    waypoints_from_positions,
)
from .types import (
    JOINT_NAMES,
    NUM_JOINTS,
    ArmCommand,
    ControlMode,
    RobotState,
)

__version__ = "0.1.0"
PHASE = 1

__all__ = [
    "PHASE",
    "__version__",
    # types
    "NUM_JOINTS",
    "JOINT_NAMES",
    "ControlMode",
    "ArmCommand",
    "RobotState",
    # limits
    "JointLimits",
    "R1_LIMITS_SOURCE",
    "r1_default_limits",
    "check_limits",
    "clamp_to_limits",
    # safety
    "SafetyConfig",
    "SafetyLevel",
    "SafetyIssue",
    "SafetyReport",
    "SafetyMonitor",
    # backends / robot
    "ArmBackend",
    "MockArm",
    "MockArmConfig",
    "NullBackend",
    "R1Robot",
    "R1Status",
    # gripper
    "Gripper",
    "GripperCommand",
    "GripperCommandMode",
    "GripperLimits",
    "GripperState",
    "MockGripper",
    "Dm4310Gripper",
    # config
    "ConfigError",
    "BackendSelection",
    "parse_config",
    "load_config",
    "safety_config_from",
    "backend_selection_from",
    # kinematics (Pinocchio, optional dependency)
    "R1Kinematics",
    "UrdfFacts",
    "urdf_facts",
    "official_urdf_path",
    "joint_limits_are_consistent_with_constants",
    "LINK_NAMES",
    "END_EFFECTOR_FRAME",
    "OFFICIAL_URDF",
    # simulation (MuJoCo, optional dependency)
    "SimArm",
    "SimArmConfig",
    "generate_mjcf",
    "write_sim_model",
    # trajectory / execution
    "JointTrajectory",
    "TrajectoryExecutor",
    "ExecutorConfig",
    "ExecutionResult",
    "Outcome",
    "waypoints_from_positions",
]
