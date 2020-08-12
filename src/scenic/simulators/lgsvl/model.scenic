import math
import time

import lgsvl

import scenic.domains.driving.model as baseModel
from scenic.domains.driving.model import DrivingObject, Vehicle, Pedestrian, Steers, Walks
from scenic.simulators.lgsvl.simulator import LGSVLSimulator
import scenic.simulators.lgsvl.utils as utils
from scenic.simulators.lgsvl.actions import *

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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._state = None      # used internally to accumulate state updates
        self._stateUpdated = False

    def setPosition(self, pos, elevation):
        self._state.position = utils.scenicToLGSVLPosition(pos, elevation)
        self._stateUpdated = True

    def setVelocity(self, vel):
        self._state.velocity = utils.scenicToLGSVLPosition(vel)
        self._stateUpdated = True

# TODO: Get vehicle models, dimensions from LGSVL
class Car(Vehicle, LGSVLObject):
    lgsvlName: 'Sedan'
    lgsvlAgentType: lgsvl.AgentType.NPC

class EgoCar(Car, Steers):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: lgsvl.AgentType.EGO

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    @property
    def control(self):
        if self._control is None:
            self._control = lgsvl.VehicleControl()
            self._stickyControl = True
        return self._control

    def setThrottle(self, throttle):
        self.control.throttle = throttle

    def setSteering(self, steering):
        self.control.steering = steering

    def setBraking(self, braking):
        self.control.braking = braking

    def setHandbrake(self, handbrake):
        self.control.handbrake = handbrake

    def setReverse(self, reverse):
        self.control.reverse = reverse

class ApolloCar(EgoCar):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: lgsvl.AgentType.EGO
    apolloVehicle: 'Lincoln2017MKZ'
    apolloModules: ['Localization', 'Perception', 'Transform', 'Routing',
                    'Prediction', 'Planning', 'Camera']
    dreamview: None     # connection to Dreamview (set at runtime)
    bridgeHost: 'localhost'
    bridgePort: 9090

class Pedestrian(Pedestrian, LGSVLObject, Walks):
    lgsvlName: 'Bob'
    lgsvlAgentType: lgsvl.AgentType.PEDESTRIAN

    def setWalkingDirection(self, heading):
        super().setWalkingDirection(heading)    # TODO use better implementation?

    def setWalkingSpeed(self, speed):
        super().setWalkingSpeed(speed)

## Utility classes

class Waypoint(OrientedPoint):
    heading: roadDirection at self.position
    speed: 10
