# NOTE: MetaDrive uses a coordinate system where (0,0) is centered
# around the middle of the SUMO map. To ensure alignment, we shift
# positions using both the computed SUMO map center (center_x, center_y)
# and adjust for SUMO’s netOffset (offset_x, offset_y).

import math
import xml.etree.ElementTree as ET

from metadrive.envs import BaseEnv, MetaDriveEnv
from metadrive.manager.sumo_map_manager import SumoMapManager
from metadrive.obs.observation_base import DummyObservation
from metadrive.policy.expert_policy import ExpertPolicy
from metadrive.obs.state_obs import LidarStateObservation
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
    xmin, ymin, xmax, ymax = sumo_map_boundary
    center_x = (xmin + xmax) / 2
    center_y = (ymin + ymax) / 2
    scenic_offset = (center_x - net_offset[0], center_y - net_offset[1])
    return scenic_offset, sumo_map_boundary


def metadriveToScenicPosition(loc, scenic_offset):
    """Converts MetaDrive position to Scenic position using map parameters."""
    x_scenic = loc[0] + scenic_offset[0]
    y_scenic = loc[1] + scenic_offset[1]
    return Vector(x_scenic, y_scenic, 0)


def scenicToMetaDrivePosition(vec, scenic_offset):
    """Converts Scenic position to MetaDrive position using map parameters."""
    adjusted_x = vec[0] - scenic_offset[0]
    adjusted_y = vec[1] - scenic_offset[1]
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
        # return LidarStateObservation(self.config)
        # print(f"Lidar Obs {LidarStateObservation(self.config)}")
        # o = DummyObservation()
        o = LidarStateObservation(self.config)
        print(f"obs: {o}")
        return o

    def setup_engine(self):
        """Setup the engine for MetaDrive."""
        super().setup_engine()
        self.engine.register_manager(
            "map_manager", SumoMapManager(self.config["sumo_map"])
        )

class PolicyDriveEnv(MetaDriveEnv):
    def done_function(self, agent):
        """Dummy done function."""
        return False, {}

class IntersectionEnv(BaseEnv):

    def reward_function(self, vehicle_id):
        """Dummy reward function."""
        vehicle = self.agents[vehicle_id]
        step_info = dict()

        # Reward for moving forward in current lane
        if vehicle.lane in vehicle.navigation.current_ref_lanes:
            current_lane = vehicle.lane
            positive_road = 1
        else:
            current_lane = vehicle.navigation.current_ref_lanes[0]
            current_road = vehicle.navigation.current_road
            positive_road = 1 if not current_road.is_negative_road() else -1
        long_last, _ = current_lane.local_coordinates(vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(vehicle.position)

        # reward for lane keeping, without it vehicle can learn to overtake but fail to keep in lane
        if self.config["use_lateral_reward"]:
            lateral_factor = clip(1 - 2 * abs(lateral_now) / vehicle.navigation.get_current_lane_width(), 0.0, 1.0)
        else:
            lateral_factor = 1.0

        reward = 0.0
        reward += self.config["driving_reward"] * (long_now - long_last) * lateral_factor * positive_road
        reward += self.config["speed_reward"] * (vehicle.speed_km_h / vehicle.max_speed_km_h) * positive_road

        step_info["step_reward"] = reward

        if self._is_arrive_destination(vehicle):
            reward = +self.config["success_reward"]
        elif self._is_out_of_road(vehicle):
            reward = -self.config["out_of_road_penalty"]
        elif vehicle.crash_vehicle:
            reward = -self.config["crash_vehicle_penalty"]
        elif vehicle.crash_object:
            reward = -self.config["crash_object_penalty"]
        elif vehicle.crash_sidewalk:
            reward = -self.config["crash_sidewalk_penalty"]
        step_info["route_completion"] = vehicle.navigation.route_completion
        return reward, step_info

    def cost_function(self, vehicle_id):
        """Dummy cost function."""
        vehicle = self.agents[vehicle_id]
        step_info = dict()
        step_info["cost"] = 0
        if self._is_out_of_road(vehicle):
            step_info["cost"] = self.config["out_of_road_cost"]
        elif vehicle.crash_vehicle:
            step_info["cost"] = self.config["crash_vehicle_cost"]
        elif vehicle.crash_object:
            step_info["cost"] = self.config["crash_object_cost"]
        return step_info['cost'], step_info

    def done_function(self, vehicle_id):
        """Dummy done function."""
        return False, {}

    # def get_single_observation(self, vehicle_id):
        # """Dummy observation function."""
        # return DummyObservation()

    def setup_engine(self):
        """Setup the engine for MetaDrive."""
        super().setup_engine()
        self.engine.register_manager(
            "map_manager", SumoMapManager(self.config["sumo_map"])
        )

    @staticmethod
    def _is_arrive_destination(vehicle):
        """
        Args:
            vehicle: The BaseVehicle instance.

        Returns:
            flag: Whether this vehicle arrives its destination.
        """
        long, lat = vehicle.navigation.final_lane.local_coordinates(vehicle.position)
        flag = (vehicle.navigation.final_lane.length - 5 < long < vehicle.navigation.final_lane.length + 5) and (
            vehicle.navigation.get_current_lane_width() / 2 >= lat >=
            (0.5 - vehicle.navigation.get_current_lane_num()) * vehicle.navigation.get_current_lane_width()
        )
        return flag

    def _is_out_of_road(self, vehicle):
        # A specified function to determine whether this vehicle should be done.
        # return vehicle.on_yellow_continuous_line or (not vehicle.on_lane) or vehicle.crash_sidewalk
        ret = not vehicle.on_lane
        if self.config["out_of_route_done"]:
            ret = ret or vehicle.out_of_route
        elif self.config["on_continuous_line_done"]:
            ret = ret or vehicle.on_yellow_continuous_line or vehicle.on_white_continuous_line or vehicle.crash_sidewalk
        if self.config["on_broken_line_done"]:
            ret = ret or vehicle.on_broken_line
        return ret
