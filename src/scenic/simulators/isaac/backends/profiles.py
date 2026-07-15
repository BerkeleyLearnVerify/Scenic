"""Typed manipulator profiles for the Isaac Sim backends.

Each profile captures the per-robot values that cannot be reliably inferred from
the USD asset: arm/gripper joint roles, control link, home pose, TCP offset,
grasp orientation, gripper open/closed setpoints, and grasp-tuning physics.
They are valid only for the exact robot/gripper USD asset (and variant) named by
the profile.

Both ``core_51`` and ``experimental_60`` import these profiles instead of keeping
their own module-level constants, so the two backends can no longer drift.  See
``README.md`` (Known Issues) for which of these values are genuine design choices
versus values that could, in principle, be read back from the initialized Isaac
articulation.
"""

from dataclasses import dataclass

import numpy as np


def _arr(values):
    """Build a read-only float array for use as a profile constant."""
    return np.array(values, dtype=float)


@dataclass(frozen=True, kw_only=True)
class ManipulatorProfile:
    """Values common to every manipulator profile.

    ``control_link_name`` is the articulation link used for Jacobian / link-index
    lookups.  ``downward_orientation`` and ``tcp_offset`` define the tool frame,
    and ``ik_damping`` / ``ik_step_scale`` parameterize the damped-least-squares
    IK step.
    """

    control_link_name: str
    downward_orientation: np.ndarray
    tcp_offset: np.ndarray
    ik_damping: float = 0.05
    ik_step_scale: float = 1.0


@dataclass(frozen=True, kw_only=True)
class FrankaProfile(ManipulatorProfile):
    """Franka Panda + built-in parallel gripper."""

    usd_path: str
    usd_variants: tuple
    end_effector_prim: str
    default_dof_positions: np.ndarray
    gripper_dof_names: tuple
    open_gripper_positions: np.ndarray
    closed_gripper_positions: np.ndarray


@dataclass(frozen=True, kw_only=True)
class UR5eProfile(ManipulatorProfile):
    """UR5e arm + Robotiq 2F-85 gripper (manual closed-loop USD authoring)."""

    arm_dof_names: tuple
    default_arm_pose: np.ndarray
    control_prim: str
    gripper_variant: str
    gripper_prim: str
    gripper_open_position: float
    gripper_closed_position: float
    gripper_fully_closed_position: float
    gripper_close_velocity: float
    gripper_open_velocity: float
    gripper_contact_material_path: str
    object_contact_material_path: str
    gripper_static_friction: float
    gripper_dynamic_friction: float
    object_static_friction: float
    object_dynamic_friction: float
    contact_offset: float
    rest_offset: float
    pick_object_mass_kg: float
    gripper_max_force: float
    gripper_stiffness: float
    gripper_damping: float
    gripper_max_joint_velocity_deg_per_sec: float
    mimic_natural_frequency: float
    mimic_damping_ratio: float
    outer_finger_parallel_stiffness: float


FRANKA_PROFILE = FrankaProfile(
    control_link_name="panda_hand",
    # Isaac quaternion convention: [w, x, y, z].
    downward_orientation=_arr([0.0, 1.0, 0.0, 0.0]),
    tcp_offset=_arr([0.0, 0.0, 0.0877]),
    ik_damping=0.05,
    ik_step_scale=1.0,
    usd_path="Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    usd_variants=(("Gripper", "AlternateFinger"), ("Mesh", "Performance")),
    end_effector_prim="panda_hand",
    default_dof_positions=_arr(
        [0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.05, 0.05]
    ),
    gripper_dof_names=("panda_finger_joint1", "panda_finger_joint2"),
    open_gripper_positions=_arr([0.05, 0.05]),
    closed_gripper_positions=_arr([0.01, 0.01]),
)


UR5E_PROFILE = UR5eProfile(
    control_link_name="wrist_3_link",
    downward_orientation=_arr([0.0, 0.70710678, 0.70710678, 0.0]),
    tcp_offset=_arr([0.0, 0.0, 0.135]),
    ik_damping=0.08,
    ik_step_scale=0.25,
    arm_dof_names=(
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
    default_arm_pose=_arr(
        [-np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0, 0.0]
    ),
    control_prim="wrist_3_link",
    gripper_variant="Robotiq_2f_85",
    gripper_prim="Gripper/Robotiq_2F_85",
    gripper_open_position=0.0,
    gripper_closed_position=float(np.deg2rad(40.0)),
    gripper_fully_closed_position=47.0,
    gripper_close_velocity=float(np.deg2rad(90.0)),
    gripper_open_velocity=float(-np.deg2rad(45.0)),
    gripper_contact_material_path="/World/UR5eRobotiqGripMaterial",
    object_contact_material_path="/World/UR5ePickObjectGripMaterial",
    gripper_static_friction=2.0,
    gripper_dynamic_friction=2.0,
    object_static_friction=2.0,
    object_dynamic_friction=2.0,
    contact_offset=0.002,
    rest_offset=0.0,
    pick_object_mass_kg=0.475,
    gripper_max_force=5.0,
    gripper_stiffness=0.0,
    gripper_damping=5000.0,
    gripper_max_joint_velocity_deg_per_sec=130.0,
    mimic_natural_frequency=0.0,
    mimic_damping_ratio=0.0,
    outer_finger_parallel_stiffness=0.05,
)
