"""Base manipulator profile consumed by the generic backend manipulator code."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


def _arr(values):
    return np.array(values, dtype=float)


#: Fields the backends read on the ``gripper_style == "robotiq_2f85"`` path
#: (declared by ``UR5eProfile``, the reference implementation).
ROBOTIQ_2F85_REQUIRED_FIELDS = (
    "gripper_prim",
    "gripper_fully_closed_position",
    "gripper_contact_material_path",
    "object_contact_material_path",
    "gripper_static_friction",
    "gripper_dynamic_friction",
    "object_static_friction",
    "object_dynamic_friction",
    "contact_offset",
    "rest_offset",
    "pick_object_mass_kg",
    "gripper_max_force",
    "gripper_stiffness",
    "gripper_damping",
    "gripper_max_joint_velocity_deg_per_sec",
    "mimic_natural_frequency",
    "mimic_damping_ratio",
    "outer_finger_parallel_stiffness",
)


@dataclass(frozen=True, kw_only=True)
class ManipulatorProfile:
    """Immutable, typed description of a manipulator robot (a frozen dataclass).

    Field groups: asset (``usd_path``/``usd_variants``), kinematics
    (``*_dof_names``, ``*_prim``, ``default_arm_pose``), gripper
    (``*gripper*``), motion tuning (``ik_*``, ``rmpflow_*``), capability flags
    (``supports_pick_place``).

    ``gripper_style`` selects the backend gripper path: ``"robotiq_2f85"``
    uses the closed-loop Robotiq authoring and requires the fields in
    ``ROBOTIQ_2F85_REQUIRED_FIELDS``; any other value is treated as a plain
    parallel gripper (``gripper_action_deltas`` optional, e.g. Franka).
    ``supports_pick_place`` currently assumes a Franka-style position-mode
    parallel gripper.

    New robots need no Scenic-internal edits: define a profile instance in
    your own module and attach it to a ``ManipulatorRobot`` subclass in your
    scenario (see "Adding a Custom Robot" in the README).
    """

    usd_path: str
    usd_variants: tuple
    arm_dof_names: tuple
    gripper_dof_names: tuple
    control_link_name: str
    end_effector_prim: str
    gripper_frame_prim: str
    default_arm_pose: np.ndarray
    downward_orientation: np.ndarray
    tcp_offset: np.ndarray
    open_gripper_positions: np.ndarray
    closed_gripper_positions: np.ndarray
    gripper_open_velocity: Optional[float] = None
    gripper_close_velocity: Optional[float] = None
    gripper_style: str
    gripper_control_mode: str = "position"
    supports_pick_place: bool = False
    ik_damping: float = 0.05
    ik_step_scale: float = 1.0
    rmpflow_policy_name: Optional[str] = None
    rmpflow_uses_tcp_offset: bool = False

    def __post_init__(self):
        if self.gripper_control_mode == "velocity" and (
            self.gripper_open_velocity is None or self.gripper_close_velocity is None
        ):
            raise ValueError(
                "gripper_control_mode='velocity' requires gripper_open_velocity"
                " and gripper_close_velocity"
            )
        if self.gripper_style == "robotiq_2f85":
            missing = [
                name for name in ROBOTIQ_2F85_REQUIRED_FIELDS if not hasattr(self, name)
            ]
            if missing:
                raise ValueError(
                    "gripper_style='robotiq_2f85' requires fields"
                    f" {missing} (see UR5eProfile)"
                )
