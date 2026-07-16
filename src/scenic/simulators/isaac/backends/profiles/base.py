"""Base manipulator profile consumed by the generic backend manipulator code."""

from dataclasses import dataclass

import numpy as np


def _arr(values):
    return np.array(values, dtype=float)


@dataclass(frozen=True, kw_only=True)
class ManipulatorProfile:
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
    gripper_open_velocity: float = None
    gripper_close_velocity: float = None
    gripper_style: str
    gripper_control_mode: str = "position"
    supports_pick_place: bool = False
    ik_damping: float = 0.05
    ik_step_scale: float = 1.0
    rmpflow_policy_name: str = None
    rmpflow_uses_tcp_offset: bool = False
