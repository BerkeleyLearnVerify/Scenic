from pathlib import Path
import genesis as gs
import numpy as np

import scenic
from scenic.simulators.genesis.simulator import GenesisSimulator
from scenic.gym.envs.scenic_gym import ScenicGymEnv

def get_obs(simulation):
    target_obj = next(o for o in simulation.objects if getattr(o, "name", None) == "Target")
    qpos = target_obj.genesis_entity.get_qpos().cpu().numpy()
    return (simulation.objects[0].genesis_entity, qpos[:, :3])

def get_info(simulation):
    return None

def get_reward(simulation):
    pass

def make_actions(obs, horizon):
    franka, target_pos = obs
    batch_size = target_pos.shape[0]
    target_quat = np.tile(np.array([0, 1, 0, 0]), [batch_size, 1])
    q = franka.inverse_kinematics(
        link=franka.get_link("hand"),
        pos=target_pos,  # shape (n_envs, 3)
        quat=target_quat,  # shape (n_envs, 4)
        rot_mask=[False, False, True],  # for demo purpose: only restrict direction of z-axis
    )
    path = franka.plan_path(
        qpos_goal=q,
        num_waypoints=horizon,
    )
    return path.cpu().numpy()

if __name__ == '__main__':
    scenario = scenic.scenarioFromFile(Path(__file__).parent/"panda.scenic")
    simulator = GenesisSimulator(
        timestep=0.01,
        genesis_options = {
            "backend": gs.gpu,
            "show_viewer": True,
            "precision": str(32),
            "substeps": 5,
        }
    )

    batch_size = 4
    horizon=1000

    gym_env = ScenicGymEnv(
        scenario=scenario,
        simulator=simulator,
        get_obs=get_obs,
        get_info=get_info,
        get_reward=get_reward,
        batch_size=batch_size,
        max_steps=10000,
    )

    obs, info = gym_env.reset()
    actions = make_actions(obs, horizon)

    for i in range(horizon):
        obs, reward, terminated, truncated, info = gym_env.step(np.expand_dims(actions[i], 0))
