import math
import time

import lgsvl

from scenic.simulators.carla.interface import CarlaWorkspace
from scenic.simulators.lgsvl.map import mapPath

workspace = CarlaWorkspace(mapPath)     # TODO improve

roadDirection = workspace.road_direction
road = workspace.drivable_region
sidewalk = workspace.sidewalk_region

# TODO: Get vehicle models, dimensions from LGSVL
constructor Car:
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2.5
    height: 5
    lgsvlName: 'Sedan'
    lgsvlAgentType: lgsvl.AgentType.NPC

constructor EgoCar(Car):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: lgsvl.AgentType.EGO

constructor Pedestrian:
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5
    lgsvlName: 'Bob'
    lgsvlAgentType: lgsvl.AgentType.PEDESTRIAN
