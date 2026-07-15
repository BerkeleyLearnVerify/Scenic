"""Franka Panda + built-in parallel gripper."""

from dataclasses import dataclass

from scenic.simulators.isaac.backends.profiles.base import ManipulatorProfile, _arr


@dataclass(frozen=True, kw_only=True)
class FrankaProfile(ManipulatorProfile):
    gripper_action_deltas: tuple


FRANKA_PROFILE = FrankaProfile(
    usd_path="Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    usd_variants=(("Gripper", "AlternateFinger"), ("Mesh", "Performance")),
    arm_dof_names=(
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ),
    gripper_dof_names=("panda_finger_joint1", "panda_finger_joint2"),
    control_link_name="panda_hand",
    end_effector_prim="panda_hand",
    gripper_frame_prim="panda_rightfinger",
    default_arm_pose=_arr([0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741]),
    downward_orientation=_arr([0.0, 1.0, 0.0, 0.0]),
    tcp_offset=_arr([0.0, 0.0, 0.0877]),
    open_gripper_positions=_arr([0.05, 0.05]),
    closed_gripper_positions=_arr([0.01, 0.01]),
    gripper_style="franka_hand",
    gripper_control_mode="position",
    supports_pick_place=True,
    ik_damping=0.05,
    ik_step_scale=1.0,
    rmpflow_policy_name="Franka",
    rmpflow_uses_tcp_offset=False,
    gripper_action_deltas=(0.01, 0.01),
)
