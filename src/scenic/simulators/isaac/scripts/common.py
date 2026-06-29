import scenic

from scenic.simulators.isaac.lab_env import (
    ScenicIsaacLabEnvWrapper,
    configure_env_cfg_for_scenic_terrain,
)
from scenic.simulators.isaac.terrain_utils import (
    build_scenic_terrain_data,
    terrain_objects_from_scene,
)


def load_scenic_scenario(path, model):
    return scenic.scenarioFromFile(path, model=model, params={"isaacLab": True})


def configure_initial_scenic_terrain(env_cfg, scenario, terrain_border_width):
    scene, _ = scenario.generate()
    terrain_data = build_scenic_terrain_data(
        terrain_objects_from_scene(scene), border_width=terrain_border_width
    )
    configure_env_cfg_for_scenic_terrain(env_cfg, terrain_data)
    return scene, terrain_data


def wrap_with_scenic(env, scenario, terrain_border_width):
    return ScenicIsaacLabEnvWrapper(
        env, scenario, terrain_border_width=terrain_border_width
    )
