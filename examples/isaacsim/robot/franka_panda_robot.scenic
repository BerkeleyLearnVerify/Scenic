model scenic.simulators.isaac.model

import numpy as np

from scenic.simulators.isaac.backends import get_backend_version
import scenic.simulators.isaac.utils as scenic_utils

IK_METHODS = {
    "singular-value-decomposition",
    "pseudoinverse",
    "transpose",
    "damped-least-squares",
}

DEFAULT_DOF_POSITIONS = np.array(
    [0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.0, 0.0],
    dtype=float,
)
OPEN_GRIPPER_POSITIONS = [0.05, 0.05]
CLOSED_GRIPPER_POSITIONS = [0.005, 0.005]
DOWNWARD_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
ARM_DOF_INDICES = list(range(7))
GRIPPER_DOF_INDICES = [7, 8]
PICK_PLACE_STATES = {}


def _quat_mul(a, b):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    return np.stack(
        [
            qq - ww + (z1 - y1) * (y2 - z2),
            qq - xx + (x1 + w1) * (x2 + w2),
            qq - yy + (w1 - x1) * (y2 + z2),
            qq - zz + (z1 + y1) * (w2 - x2),
        ],
        axis=-1,
    )


def _quat_conjugate(q):
    return np.concatenate((q[:, :1], -q[:, 1:]), axis=-1)


def differential_inverse_kinematics(
    jacobian,
    current_position,
    current_orientation,
    goal_position,
    goal_orientation=None,
    method="damped-least-squares",
    scale=1.0,
    damping=0.05,
    min_singular_value=1e-5,
):
    jacobian = np.asarray(jacobian, dtype=float)
    if jacobian.shape != (6, 7):
        raise ValueError(f"expected a 6x7 end-effector Jacobian, got {jacobian.shape}")

    current_position = np.asarray(current_position, dtype=float).reshape(1, 3)
    current_orientation = np.asarray(current_orientation, dtype=float).reshape(1, 4)
    goal_position = np.asarray(goal_position, dtype=float).reshape(1, 3)
    goal_orientation = (
        current_orientation if goal_orientation is None else goal_orientation
    )
    goal_orientation = np.asarray(goal_orientation, dtype=float).reshape(1, 4)

    q = _quat_mul(goal_orientation, _quat_conjugate(current_orientation))
    error = np.concatenate(
        [goal_position - current_position, q[:, 1:] * np.sign(q[:, [0]])],
        axis=-1,
    ).reshape(6)

    scale = float(scale)
    damping = float(damping)
    min_singular_value = float(min_singular_value)

    if method == "singular-value-decomposition":
        U, S, Vh = np.linalg.svd(jacobian, full_matrices=False)
        inv_s = np.where(S > min_singular_value, 1.0 / S, 0.0)
        jacobian_inverse = np.matmul(np.matmul(Vh.T, np.diag(inv_s)), U.T)
    elif method == "pseudoinverse":
        jacobian_inverse = np.linalg.pinv(jacobian)
    elif method == "transpose":
        jacobian_inverse = jacobian.T
    elif method == "damped-least-squares":
        lhs = np.matmul(jacobian, jacobian.T) + np.eye(6, dtype=float) * (damping**2)
        jacobian_inverse = np.matmul(
            jacobian.T,
            np.linalg.solve(lhs, np.eye(6, dtype=float)),
        )
    else:
        raise ValueError(f"invalid IK method {method!r}")

    return scale * np.matmul(jacobian_inverse, error)


def _new_pick_place_state():
    return {
        "initialized": False,
        "stage": 0,
        "stage_steps": 0,
        "done": False,
        "pick_position": None,
        "place_position": None,
        "end_effector": None,
        "end_effector_link_index": None,
    }


def _initialize_franka(sim, obj, state):
    from isaacsim.core.experimental.prims import RigidPrim

    handle = sim.world.get_object(obj.name)
    robot = handle.wrapper
    state["end_effector"] = RigidPrim(f"{handle.prim_path}/panda_hand")
    state["end_effector_link_index"] = robot.get_link_indices("panda_hand").list()[0]
    robot.set_default_state(dof_positions=DEFAULT_DOF_POSITIONS.tolist())
    robot.reset_to_default_state()
    state["initialized"] = True
    return handle


def _move_arm(robot, state, target, method, scale, damping, min_singular_value):
    current_position, current_orientation = state["end_effector"].get_world_poses()
    current_position = current_position.numpy()
    current_orientation = current_orientation.numpy()

    jacobians = robot.get_jacobian_matrices().numpy()
    jacobian = jacobians[0, state["end_effector_link_index"] - 1, :, :7]
    delta = differential_inverse_kinematics(
        jacobian,
        current_position,
        current_orientation,
        np.asarray(target, dtype=float).reshape(1, 3),
        goal_orientation=np.array([DOWNWARD_ORIENTATION], dtype=float),
        method=method,
        scale=scale,
        damping=damping,
        min_singular_value=min_singular_value,
    )

    current_dof_positions = robot.get_dof_positions().numpy()
    arm_targets = (
        current_dof_positions[:, ARM_DOF_INDICES] + delta
    ).reshape(-1).tolist()
    robot.set_dof_position_targets(arm_targets, dof_indices=ARM_DOF_INDICES)


def franka_pick_place_control(sim, obj, command):
    target_object, goal_position, method, scale, damping, min_singular_value = command
    scale = float(scale)
    damping = float(damping)
    min_singular_value = float(min_singular_value)

    state_key = (id(sim), obj.name)
    state = PICK_PLACE_STATES.get(state_key)
    if state is None:
        state = PICK_PLACE_STATES[state_key] = _new_pick_place_state()
    if state["done"]:
        return

    handle = (
        sim.world.get_object(obj.name)
        if state["initialized"]
        else _initialize_franka(sim, obj, state)
    )
    robot = handle.wrapper
    if state["pick_position"] is None:
        target_handle = sim.world.get_object(target_object.name)
        state["pick_position"] = (
            target_handle.wrapper.get_world_poses()[0].numpy()[0].copy()
        )
    if state["place_position"] is None:
        state["place_position"] = scenic_utils.vectorToArray(goal_position).copy()

    phases = [
        (
            state["pick_position"] + np.array([0.0, 0.0, 0.20]),
            OPEN_GRIPPER_POSITIONS,
            120,
        ),
        (
            state["pick_position"] + np.array([0.0, 0.0, 0.10]),
            OPEN_GRIPPER_POSITIONS,
            80,
        ),
        (None, CLOSED_GRIPPER_POSITIONS, 100),
        (
            state["pick_position"] + np.array([0.0, 0.0, 0.50]),
            CLOSED_GRIPPER_POSITIONS,
            150,
        ),
        (
            state["place_position"] + np.array([0.0, 0.0, 0.50]),
            CLOSED_GRIPPER_POSITIONS,
            180,
        ),
        (
            state["place_position"] + np.array([0.0, 0.0, 0.20]),
            CLOSED_GRIPPER_POSITIONS,
            90,
        ),
        (None, OPEN_GRIPPER_POSITIONS, 20),
    ]
    target, gripper_positions, steps = phases[state["stage"]]
    if target is not None:
        _move_arm(robot, state, target, method, scale, damping, min_singular_value)
    robot.set_dof_position_targets(
        list(gripper_positions), dof_indices=GRIPPER_DOF_INDICES
    )
    state["stage_steps"] += 1
    if state["stage_steps"] > steps:
        print(f"CustomFranka stage={state['stage']}, method={method}, target={target}")
        state["stage"] += 1
        state["stage_steps"] = 0
        state["done"] = state["stage"] >= len(phases)


behavior FrankaPickPlaceWithIK(
    target_object,
    goal_position,
    ik_method,
    ik_scale,
    ik_damping,
    ik_min_singular_value,
):
    while True:
        take applyController(
            [
                target_object,
                goal_position,
                ik_method,
                ik_scale,
                ik_damping,
                ik_min_singular_value,
            ]
        )


class CustomFrankaPanda(IsaacSimRobot):
    width: 0.3
    length: 0.3
    height: 0.01
    isaac_asset_path: "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

    def move(self, sim, command):
        franka_pick_place_control(sim, self, command)
