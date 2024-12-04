try:
    from metadrive.envs import BaseEnv
    from metadrive.manager.sumo_map_manager import SumoMapManager
    from metadrive.obs.observation_base import DummyObservation
except ImportError as e:
    raise ModuleNotFoundError(
        'Metadrive scenarios require the "metadrive" package'
    ) from e

import math
import sys
import xml.etree.ElementTree as ET

from scenic.core.vectors import Vector


def calculateFilmSize(sumo_map_boundary, scaling=5, margin_factor=1.1):
    xmin, ymin, xmax, ymax = sumo_map_boundary
    width = xmax - xmin
    height = ymax - ymin

    # Apply margin
    adjusted_width = width * margin_factor
    adjusted_height = height * margin_factor

    # Convert to pixels for film_size
    film_size_x = int(adjusted_width * scaling)
    film_size_y = int(adjusted_height * scaling)

    return (film_size_x, film_size_y)


def extractNetOffsetAndBoundary(net_file_path):
    tree = ET.parse(net_file_path)
    root = tree.getroot()
    location_tag = root.find("location")
    net_offset = tuple(map(float, location_tag.attrib["netOffset"].split(",")))
    sumo_map_boundary = tuple(map(float, location_tag.attrib["convBoundary"].split(",")))
    return net_offset, sumo_map_boundary


def metadriveToScenicPosition(loc, center_x, center_y, offset_x, offset_y):
    # print(f"Input MetaDrive Position: {loc}")
    x_scenic = loc[0] + center_x - offset_x
    y_scenic = loc[1] + center_y - offset_y
    result = Vector(x_scenic, y_scenic, 0)
    # print(f"Converted to Scenic Position: {result}")
    # breakpoint()
    return result


def scenicToMetaDrivePosition(vec, center_x, center_y, offset_x, offset_y):
    print("offset x: ", offset_x)
    print("offset y: ", offset_y)
    # print(f"Input Scenic Position: {vec}")
    adjusted_x = vec[0] - center_x + offset_x
    adjusted_y = vec[1] - center_y + offset_y
    result = (adjusted_x, adjusted_y)
    # print(f"Converted to MetaDrive Position: {result}")
    # # Validate reverse conversion
    # reverse = metadriveToScenicPosition(result, center_x, center_y, offset_x, offset_y)
    # print(f"Reverse Converted Scenic Position: {reverse}")
    # breakpoint()
    # print(f"Scenic Position: {vec}, MetaDrive Position: ({adjusted_x}, {adjusted_y})")
    return result


def scenicToMetaDriveHeading(scenicHeading):
    # Add π/2 to shift from North-based (Scenic) to East-based (MetaDrive)
    metadriveHeading = scenicHeading + (math.pi / 2)
    # Normalize to [-π, π]
    return (metadriveHeading + math.pi) % (2 * math.pi) - math.pi


def metaDriveToScenicHeading(metaDriveHeading):
    # Subtract π/2 to shift from East-based (MetaDrive) to North-based (Scenic)
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
        super().setup_engine()
        posix_path = self.config["sumo_map"]
        final_path = (
            str(posix_path)
            if not sys.platform.startswith("win")
            else posix_path.as_posix()
        )
        self.engine.register_manager("map_manager", SumoMapManager(final_path))
