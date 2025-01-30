# NOTE: MetaDrive currently has their own coordinate
# system where (0,0) is centered around the middle of
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172

try:
    from metadrive.envs import BaseEnv
    from metadrive.manager.sumo_map_manager import SumoMapManager
    from metadrive.obs.observation_base import DummyObservation
except ImportError as e:
    raise ModuleNotFoundError(
        'Metadrive scenarios require the "metadrive" package'
    ) from e

import math
import xml.etree.ElementTree as ET

from scenic.core.vectors import Vector


def calculateFilmSize(sumo_map_boundary, scaling=5, margin_factor=1.1):
    """Calculates the film size for rendering based on the map's boundary."""
    # Calculate the width and height based on the sumo_map_boundary
    xmin, ymin, xmax, ymax = sumo_map_boundary
    width = xmax - xmin
    height = ymax - ymin

    # Apply margin and convert to pixels
    adjusted_width = width * margin_factor
    adjusted_height = height * margin_factor
    return int(adjusted_width * scaling), int(adjusted_height * scaling)


def extractNetOffsetAndBoundary(sumo_map_path):
    """Extracts the net offset and boundary from the given SUMO map file."""
    tree = ET.parse(sumo_map_path)
    root = tree.getroot()
    location_tag = root.find("location")
    net_offset = tuple(map(float, location_tag.attrib["netOffset"].split(",")))
    sumo_map_boundary = tuple(map(float, location_tag.attrib["convBoundary"].split(",")))
    return net_offset, sumo_map_boundary


def getMapParameters(sumo_map_path):
    """Retrieve the map parameters."""
    net_offset, sumo_map_boundary = extractNetOffsetAndBoundary(sumo_map_path)
    if net_offset and sumo_map_boundary:
        xmin, ymin, xmax, ymax = sumo_map_boundary
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        offset_x = net_offset[0]
        offset_y = net_offset[1]
        return center_x, center_y, offset_x, offset_y, sumo_map_boundary
    else:
        raise RuntimeError("Failed to extract netOffset or convBoundary from SUMO map.")


def metadriveToScenicPosition(loc, center_x, center_y, offset_x, offset_y):
    """Converts MetaDrive position to Scenic position using map parameters."""
    x_scenic = loc[0] + center_x - offset_x
    y_scenic = loc[1] + center_y - offset_y
    return Vector(x_scenic, y_scenic, 0)


def scenicToMetaDrivePosition(vec, center_x, center_y, offset_x, offset_y):
    """Converts Scenic position to MetaDrive position using map parameters."""
    adjusted_x = vec[0] - center_x + offset_x
    adjusted_y = vec[1] - center_y + offset_y
    return adjusted_x, adjusted_y


def scenicToMetaDriveHeading(scenicHeading):
    """
    Converts Scenic heading to MetaDrive heading by adding π/2 (90 degrees).

    Scenic's coordinate system has 0 radians pointing North, while MetaDrive uses
    0 radians pointing East. This function shifts the heading to align with MetaDrive's system.
    """
    metadriveHeading = scenicHeading + (math.pi / 2)
    # Normalize to [-π, π]
    return (metadriveHeading + math.pi) % (2 * math.pi) - math.pi


def metaDriveToScenicHeading(metaDriveHeading):
    """Converts MetaDrive heading to Scenic heading by subtracting π/2 (90 degrees)."""
    scenicHeading = metaDriveHeading - (math.pi / 2)
    # Normalize to [-π, π]
    return (scenicHeading + math.pi) % (2 * math.pi) - math.pi


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
        """Setup the engine for MetaDrive."""
        super().setup_engine()
        self.engine.register_manager(
            "map_manager", SumoMapManager(self.config["sumo_map"])
        )

    def step(self):
        """
        Custom step function to bypass MetaDrive's before_step and after_step.

        In MetaDrive, the typical step includes before_step, step, and after_step
        processes. However, in this class, we have already applied the necessary actions
        to the vehicles in the `MetaDriveSimulation` class's `executeActions`. Therefore,
        here we skip MetaDrive's `before_step` and directly call the engine's step and after_step.
        """
        self.engine.step(self.config["decision_repeat"])
        self.engine.after_step()
        while self.in_stop:
            self.engine.taskMgr.step()
