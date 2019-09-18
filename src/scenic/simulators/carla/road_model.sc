import math
import time

from scenic.simulators.carla.interface import CarlaWorkspace
from scenic.simulators.carla.map import mapPath

workspace = CarlaWorkspace(mapPath)

roadDirection = workspace.road_direction
road = workspace.drivable_region
sidewalk = workspace.sidewalk_region
# laneSectionDict is a dict from road id to a list of dicts from
# lane id to polygon, one dict per lane section.
laneSectionDict = workspace.lane_sec_dict

# TODO: Get vehicle models, dimensions from Carla
constructor Car:
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2.5
    height: 5

constructor Pedestrian:
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5