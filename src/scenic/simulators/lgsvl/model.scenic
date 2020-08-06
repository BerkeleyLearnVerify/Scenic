import math
import time

import lgsvl

import scenic.simulators.domains.driving.model as baseModel
from scenic.simulators.domains.driving.model import DrivingObject

from scenic.simulators.lgsvl.simulator import LGSVLSimulator

# Load map and set up various useful regions, etc.

workspace = baseModel.workspace

network = baseModel.network
road = baseModel.road
roadDirection = baseModel.roadDirection
curb = baseModel.curb
sidewalk = baseModel.sidewalk
intersection = baseModel.intersection

## LGSVL objects

class LGSVLObject(DrivingObject):
    lgsvlObject: None
    elevation: None
    speed: 0

# TODO: Get vehicle models, dimensions from LGSVL
class Car(LGSVLObject):
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2.5
    height: 5
    requireVisible: False
    lgsvlName: 'Sedan'
    lgsvlAgentType: lgsvl.AgentType.NPC

class EgoCar(Car):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: lgsvl.AgentType.EGO

class ApolloCar(EgoCar):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: lgsvl.AgentType.EGO
    apolloVehicle: 'Lincoln2017MKZ'
    apolloModules: ['Localization', 'Perception', 'Transform', 'Routing',
                    'Prediction', 'Planning', 'Camera']
    dreamview: None     # connection to Dreamview (set at runtime)
    bridgeHost: 'localhost'
    bridgePort: 9090

class Pedestrian(LGSVLObject):
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5
    lgsvlName: 'Bob'
    lgsvlAgentType: lgsvl.AgentType.PEDESTRIAN

## Utility classes

class Waypoint(OrientedPoint):
    heading: roadDirection at self.position
    speed: 10
