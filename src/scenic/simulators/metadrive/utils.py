try:
    from metadrive.envs import BaseEnv
    from metadrive.manager.sumo_map_manager import SumoMapManager
    from metadrive.obs.observation_base import DummyObservation
except ImportError as e:
    raise ModuleNotFoundError('Metadrive scenarios require the "metadrive" package') from e

from scenic.core.vectors import Vector
import sys

def metadriveToScenicPosition(loc):
    return Vector(loc[0], -loc[1], 0)

def scenicToMetaDrivePosition(vec, center_x, center_y):
    return (vec[0] + -center_x, vec[-1] + -center_y)   
        
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
        posix_path = self.config["sumo_map"]
        final_path = str(posix_path) if not sys.platform.startswith("win") else posix_path.as_posix()
        self.engine.register_manager("map_manager", SumoMapManager(final_path))