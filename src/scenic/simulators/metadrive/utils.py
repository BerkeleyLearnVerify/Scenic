from metadrive.envs import MetaDriveEnv
from metadrive.manager import BaseManager
from metadrive.envs import BaseEnv
from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.policy.idm_policy import IDMPolicy
from metadrive.component.traffic_participants.pedestrian import Pedestrian
from metadrive.engine.asset_loader import AssetLoader
from metadrive.manager.sumo_map_manager import SumoMapManager
from metadrive.obs.observation_base import DummyObservation

from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Orientation, Vector

def metadriveToScenicPosition(loc):
    return Vector(loc[0], -loc[1], 0)

def scenicToMetaDrivePosition(vec):
    return (vec[0], -vec[-1])


class DriveManager(BaseManager):
    def __init__(self):
        super(DriveManager, self).__init__()
        self.generated_v = None
        self.ped = None
        self.generate_ts = 0

    def reset(self):
        super().reset()  # Call super method after cleanup
        
    def before_step(self):
        pass
        
    def after_step(self):
        pass
        

class DriveEnv(BaseEnv):
    def reward_function(self, agent):
        """Dummy reward function."""
        return 0, {}

    def cost_function(self, agent):
        """Dummy cost function."""
        return 0, {}

    def done_function(self, agent):
        """Dummy done function."""
        return False, {}
    
    def get_single_observation(self):
        """Dummy observation function."""
        return DummyObservation()
    
    def setup_engine(self):
        super().setup_engine()
        map_path = AssetLoader.file_path("carla", "CARLA_town01.net.xml", unix_style=False)
        self.engine.register_manager("map_manager", SumoMapManager(map_path))
        # self.engine.register_manager("drive_mgr", DriveManager())