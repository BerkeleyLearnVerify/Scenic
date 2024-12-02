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

from scenic.core.vectors import Vector


def metadriveToScenicPosition(loc, center_x, center_y):
    # # print("loc: ", loc)
    # output = Vector(loc[0] + center_x, loc[1] + center_y, 0)
    # # print("output: ", output)
    # return output
    x_scenic = loc[0] + center_x
    y_scenic = loc[1] + center_y
    # print(f"MetaDrive Position: {loc}, Scenic Position: ({x_scenic}, {y_scenic})")
    return Vector(x_scenic, y_scenic, 0)


def scenicToMetaDrivePosition(vec, center_x, center_y):
    # print("vec: ", vec)
    # breakpoint()
    """
    WORKING ON ROAD EXAMPLE
    vec:  Vector(87.46780276591852, -101.87887262172462, 0)
    (Pdb) center_x
    197.17500004534386
    (Pdb) center_y
    164.28006063115316
    (Pdb) vec[0], vec[1]
    (362.504519480603, -0.9735181061390716)

    adjusted for metadrive x: 165.3295194353
    adjust for metadrive y: 165.2535787373
    """

    """
    THIS ONE IS NOT WORKING BUT IT APPEARS THE REASON MAY BE RELATED TO THE SIGNS
    vec[0], vec[1]
    (392.5373157291839, -307.43332657898054)
    (Pdb) center_x
    197.17500004534386
    (Pdb) center_y
    164.28006063115316

    adjusted for metadrive x: 195.3623156838
    adjust for metadrive y: -471.7133872101

    Actual adjustments that are needed: -143.1532659478
    """

    # if vec[0] > 0:
    #     adjusted_x = vec[0] - center_x
    # else:
    #     adjusted_x = center_x + vec[0]

    # if vec[1] > 0:
    #     adjusted_y = vec[1] - center_y
    # else:
    #     adjusted_y = center_y + vec[1]

    # return (adjusted_x, adjusted_y)

    adjusted_x = vec[0] - center_x
    adjusted_y = vec[1] - center_y
    # print(f"Scenic Position: {vec}, MetaDrive Position: ({adjusted_x}, {adjusted_y})")
    return (adjusted_x, adjusted_y)


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
