"""UR5e arm + Robotiq 2F-85 gripper (manual closed-loop USD authoring)."""

from dataclasses import dataclass

import numpy as np

from scenic.simulators.isaac.backends.profiles.base import ManipulatorProfile, _arr


@dataclass(frozen=True, kw_only=True)
class UR5eProfile(ManipulatorProfile):
    gripper_prim: str
    gripper_fully_closed_position: float
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


UR5E_PROFILE = UR5eProfile(
    usd_path="Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    usd_variants=(("Gripper", "Robotiq_2f_85"),),
    arm_dof_names=(
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
    gripper_dof_names=("finger_joint",),
    control_link_name="wrist_3_link",
    end_effector_prim="wrist_3_link",
    gripper_frame_prim="Gripper/Robotiq_2F_85/base_link",
    default_arm_pose=_arr(
        [-np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0, 0.0]
    ),
    downward_orientation=_arr([0.0, 0.70710678, 0.70710678, 0.0]),
    tcp_offset=_arr([0.0, 0.0, 0.135]),
    open_gripper_positions=_arr([0.0]),
    closed_gripper_positions=_arr([float(np.deg2rad(40.0))]),
    gripper_open_velocity=float(-np.deg2rad(45.0)),
    gripper_close_velocity=float(np.deg2rad(90.0)),
    gripper_style="robotiq_2f85",
    gripper_control_mode="velocity",
    supports_pick_place=False,
    ik_damping=0.08,
    ik_step_scale=0.25,
    rmpflow_policy_name="UR5e",
    rmpflow_uses_tcp_offset=True,
    gripper_prim="Gripper/Robotiq_2F_85",
    gripper_fully_closed_position=47.0,
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
