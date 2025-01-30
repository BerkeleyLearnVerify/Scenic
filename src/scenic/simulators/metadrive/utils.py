# NOTE: MetaDrive currently has their own coordinate
# system where (0,0) is centered around the middle of
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172

try:
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
    from metadrive.envs import BaseEnv
    from metadrive.manager.sumo_map_manager import SumoMapManager
    from metadrive.obs.observation_base import DummyObservation
    from metadrive.utils import merge_dicts
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


def full_stop_workaround_step_simi(self, actions, ego_obj, throttle_break):
    # prepare for stepping the simulation
    scene_manager_before_step_infos = self.client.engine.before_step(actions)
    if throttle_break < 0:
        ego_obj.apply_throttle_brake(throttle_break)
    # step all entities and the simulator
    self.client.engine.step(self.client.config["decision_repeat"])
    # update states, if restore from episode data, position and heading will be force set in update_state() function
    scene_manager_after_step_infos = self.client.engine.after_step()

    # Note that we use shallow update for info dict in this function! This will accelerate system.
    return merge_dicts(
        scene_manager_after_step_infos,
        scene_manager_before_step_infos,
        allow_new_keys=True,
        without_copy=True,
    )


def full_stop_workaround_step(self):
    ego_obj = self.scene.objects[0]
    if ego_obj.isCar:
        action = ego_obj.collectAction()
        actions = self.client._preprocess_actions(action)
        throttle_break = action[1]
        engine_info = full_stop_workaround_step_simi(
            self, actions, ego_obj, throttle_break
        )
        while self.client.in_stop:
            self.client.engine.taskMgr.step()
        return self.client._get_step_return(actions, engine_info=engine_info)


# class MetadriveVehicle(DefaultVehicle):
#     """
#     This class is a custom subclass of DefaultVehicle. It overrides the `_apply_throttle_brake`
#     method to implement a fix for the car stopping fully, which is not included in the latest
#     version of MetaDrive on PyPI (as of now).

#     In the latest version of MetaDrive, the car doesn't come to a full stop reliably,
#     so this fix has been added here.
#     """

#     def _apply_throttle_brake(self, throttle_brake):
#         max_engine_force = self.config["max_engine_force"]
#         max_brake_force = self.config["max_brake_force"]
#         for wheel_index in range(4):
#             if throttle_brake >= 0:
#                 self.system.setBrake(2.0, wheel_index)
#                 if self.speed_km_h > self.max_speed_km_h:
#                     self.system.applyEngineForce(0.0, wheel_index)
#                 else:
#                     self.system.applyEngineForce(max_engine_force * throttle_brake, wheel_index)
#             else:
#                 if self.enable_reverse:
#                     self.system.applyEngineForce(max_engine_force * throttle_brake, wheel_index)
#                     self.system.setBrake(0, wheel_index)
#                 else:
#                     DEADZONE = 0.01

#                     # Speed m/s in car's heading:
#                     heading = self.heading
#                     velocity = self.velocity
#                     speed_in_heading = velocity[0] * heading[0] + velocity[1] * heading[1]

#                     if speed_in_heading < DEADZONE:
#                         self.system.applyEngineForce(0.0, wheel_index)
#                         self.system.setBrake(2, wheel_index)
#                     else:
#                         self.system.applyEngineForce(0.0, wheel_index)
#                         self.system.setBrake(abs(throttle_brake) * max_brake_force, wheel_index)
