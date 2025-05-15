import habitat_sim
import pdb
import magnum as mn
import warnings
from habitat.tasks.rearrange.rearrange_sim import RearrangeSim
warnings.filterwarnings('ignore')
from habitat_sim.utils.settings import make_cfg
from matplotlib import pyplot as plt
from habitat_sim.utils import viz_utils as vut
from omegaconf import DictConfig
import numpy as np
from habitat.articulated_agents.robots import FetchRobot
from habitat.config.default import get_agent_config
from habitat.config.default_structured_configs import ThirdRGBSensorConfig, HeadRGBSensorConfig, HeadPanopticSensorConfig
from habitat.config.default_structured_configs import SimulatorConfig, HabitatSimV0Config, AgentConfig
from habitat.config.default import get_agent_config
import habitat
from habitat_sim.physics import JointMotorSettings, MotionType
from habitat.config.default_structured_configs import TaskConfig, EnvironmentConfig, DatasetConfig, HabitatConfig
from habitat.config.default_structured_configs import ArmActionConfig, BaseVelocityActionConfig, OracleNavActionConfig, ActionConfig
from habitat.core.env import Env
from omegaconf import OmegaConf
import os

def make_sim_cfg(agent_dict):
    # Start the scene config
    sim_cfg = SimulatorConfig(type="RearrangeSim-v0") # TODO change this for general sim in the future
    
    data_path = '/home/kxu/habitat-lab/data/'
    # This is for better graphics
    sim_cfg.habitat_sim_v0.enable_hbao = True
    sim_cfg.habitat_sim_v0.enable_physics = True

    
    # Set up an example scene
    sim_cfg.scene = os.path.join(data_path, "hab3_bench_assets/hab3-hssd/scenes/103997919_171031233.scene_instance.json")
    sim_cfg.scene_dataset = os.path.join(data_path, "hab3_bench_assets/hab3-hssd/hab3-hssd.scene_dataset_config.json")
    sim_cfg.additional_object_paths = [os.path.join(data_path, 'objects/ycb/configs/')]
    
    print(sim_cfg.scene)
    print(sim_cfg.scene_dataset)
    
    cfg = OmegaConf.create(sim_cfg)
    print('finished init configs!!!')
    # Set the scene agents
    cfg.agents = agent_dict
    cfg.agents_order = list(cfg.agents.keys())
    return cfg

def make_hab_cfg(agent_dict, action_dict, lab_sensor_dict, timestep=1):
    """
    Make the configurations for habitat env
    """
    sim_cfg = make_sim_cfg(agent_dict)
    task_cfg = TaskConfig(type="RearrangeEmptyTask-v0")
    # task_cfg = TaskConfig()
    task_cfg.actions = action_dict
    # task_cfg.lab_sensors = lab_sensor_dict
    env_cfg = EnvironmentConfig()
    env_cfg.max_episode_steps = int(1e20)
    env_cfg.max_episode_seconds = int(1e20)
    # FIXME line below has hardcoded directory
    dataset_cfg = DatasetConfig(type="RearrangeDataset-v0", 
                                data_path="/home/kxu/habitat-lab/data/hab3_bench_assets/episode_datasets/small_large.json.gz",
                                scenes_dir="/home/kxu/habitat-lab/data/scene_datasets") 

    task_cfg.physics_target_sps = 1/timestep # This communicates the Scenic timestep to habitat

    hab_cfg = HabitatConfig()
    hab_cfg.environment = env_cfg
    hab_cfg.task = task_cfg
    hab_cfg.dataset = dataset_cfg
    hab_cfg.simulator = sim_cfg
    hab_cfg.simulator.seed = hab_cfg.seed

    # print(f"env_config: {env_cfg}")
    # print(f"task_config: {task_cfg}")
    # print(f"dataset_config: {dataset_cfg}")
    # print(f"sim_config: {sim_cfg}")
    # print(f"hab_config {hab_cfg}")
    return hab_cfg

def create_agent_config(name, agent_type, urdf_path, motion_data_path=None, sim_sensors=None, ik_arm_urdf=""):
    # TODO add cases for humanoids!!!
    main_agent_config = AgentConfig()
    main_agent_config.articulated_agent_urdf = urdf_path
    main_agent_config.articulated_agent_type = agent_type
    main_agent_config.sim_sensors = sim_sensors
    if motion_data_path:
        main_agent_config.motion_data_path = motion_data_path
    if ik_arm_urdf != "":
        main_agent_config.ik_arm_urdf = ik_arm_urdf
    return main_agent_config

def init_rearrange_sim(agent_dict):
    # Start the scene config
    sim_cfg = make_sim_cfg(agent_dict)    
    print('FINISHED MAKING SIM CFG') 
    cfg = OmegaConf.create(sim_cfg)
    
    # Create the scene
    sim = RearrangeSim(cfg)

    # This is needed to initialize the agents
    sim.agents_mgr.on_new_scene()

    # For this tutorial, we will also add an extra camera that will be used for third person recording.
    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.uuid = "scene_camera_rgb"

    # TODO: this is a bit dirty but I think its nice as it shows how to modify a camera sensor...
    sim.add_sensor(camera_sensor_spec, 0)

    return sim

def init_rearrange_env(agent_dict, action_dict, lab_sensor_dict, timestep=1):
    """
    Initializes the rearrangement environment
    """
    hab_cfg = make_hab_cfg(agent_dict, action_dict, lab_sensor_dict, timestep=timestep)
    res_cfg = OmegaConf.create(hab_cfg)
    # breakpoint()
    # print(f"res_cfg {res_cfg}")
    # print(f"scenes_dir = {res_cfg['scenes_dir']} ")
    return Env(res_cfg)

def add_scene_camera(env, name='scene_camera_rgb', camera_pos: mn.Vector3 = mn.Vector3(0, 4, 7),
                     orientation: mn.Vector3 = mn.Vector3(-1.57, 0, 0),
                     resolution: mn.Vector2i = (mn.Vector2i(1024, 1024)), agent_id=None):

    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.uuid = name
    camera_sensor_spec.position = camera_pos
    camera_sensor_spec.orientation = orientation
    camera_sensor_spec.resolution = resolution
    env.sim.add_sensor(camera_sensor_spec, agent_id)


def set_agent_state(agent, position, orientation):
    agent_state = habitat_sim.AgentState()
    agent_state.position = position
    agent_state.orientation = orientation
    agent.set_state(agent_state)


def get_agent_state(agent):
    return agent.get_state()

def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


def scenic_to_habitat_map(pose, obj=None):
    """
    Converts from the Scenic map coordinate to the habitat map frame coordinate
    Args:
    """
    g = np.array([[0, 1, 0, 0], 
                  [0, 0, 1, 0], 
                  [1, 0, 0, 0], 
                  [0, 0, 0, 1]])
    x, y, z, roll, pitch, yaw = pose
    x, y, z, _ = g @ np.array([x, y, z, 1])

    new_roll = pitch
    new_pitch = yaw
    new_yaw = roll
    return (x, y, z, new_roll, new_pitch, new_yaw)



def habitat_to_scenic_map(pose, obj=None):
    """
    Converts from the habitat map frame coordinate to the Scenic map coordinate
    Args:
    pose: (x, y, z, roll, pitch, yaw)
    """
    # assert len(pose) == 4
    # return self.RobotToScenicMap(self.HabitatToRobotMap(pose), obj=obj)
    g = np.array([[0, 1, 0, 0], 
                  [0, 0, 1, 0], 
                  [1, 0, 0, 0], 
                  [0, 0, 0, 1]])
    g = np.linalg.inv(g)
    x, y, z, roll, pitch, yaw = pose
    x, y, z, _ = g @ np.array([x, y, z, 1])
    new_roll = yaw
    new_pitch = roll
    new_yaw = pitch
    return (x, y, z, new_roll, new_pitch, new_yaw)

