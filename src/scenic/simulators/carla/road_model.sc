import math
import time

from scenic.simulators.carla.interface import CarlaWorkspace
from scenic.simulators.carla.map import mapPath

# TODO: Write carla_workspace
workspace = CarlaWorkspace(mapPath)

roadDirection = workspace.road_direction
road = workspace.drivable_region
sidewalk = workspace.sidewalk_region

# TODO: Get vehicle models, dimensions from Carla
constructor Car:
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg

constructor Pedestrian:
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5