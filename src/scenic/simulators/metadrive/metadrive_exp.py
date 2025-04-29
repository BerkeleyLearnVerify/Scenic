from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.policy.expert_policy import ExpertPolicy

env=MetaDriveEnv(dict(map="C",
                      agent_policy=ExpertPolicy,
                      log_level=50,
                      traffic_density=0))
print(env.observation_space)
try:
    # run several episodes
    env.reset()
    for step in range(300):
        # simulation
        obs,_,_,_,info = env.step([0, 3])
        env.render(mode="topdown", 
                   window=False,
                   screen_record=True,
                   screen_size=(700, 870),
                   camera_position=(60,-63)
                  )
        print(f"observation: {obs}")
        if info["arrive_dest"]:
            break
    env.top_down_renderer.generate_gif()
finally:
    env.close()


