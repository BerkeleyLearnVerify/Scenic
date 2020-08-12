import math
import time

import lgsvl

from scenic.domains.driving.model import *
from scenic.simulators.lgsvl.simulator import LGSVLSimulator
import scenic.simulators.lgsvl.utils as utils
from scenic.simulators.lgsvl.actions import *

## LGSVL objects

class LGSVLObject(DrivingObject):
    lgsvlObject: None   # corresponding lgsvl.Agent object
    state: None     # LGSVL state, used internally to accumulate state updates

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stateUpdated = False

    def setPosition(self, pos, elevation):
        self.state.position = utils.scenicToLGSVLPosition(pos, elevation)
        self._stateUpdated = True

    def setVelocity(self, vel):
        self.state.velocity = utils.scenicToLGSVLPosition(vel)
        self._stateUpdated = True

# TODO: Get vehicle models, dimensions from LGSVL
class Vehicle(Vehicle, LGSVLObject):
    pass

class Car(Vehicle):
    def __new__(cls, *args, **kwargs):
        return super().__new__(EgoCar, *args, **kwargs)

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

class NPCCar(NPCCar, Vehicle):
    lgsvlName: 'Sedan'
    lgsvlAgentType: lgsvl.AgentType.NPC

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
