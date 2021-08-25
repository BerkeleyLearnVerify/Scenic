"""Scenic world model for the LGSVL Simulator."""

from scenic.domains.driving.model import *
from scenic.simulators.lgsvl.behaviors import *

try:
    import lgsvl
    EGO_TYPE = lgsvl.AgentType.EGO
    NPC_TYPE = lgsvl.AgentType.NPC
    PEDESTRIAN_TYPE = lgsvl.AgentType.PEDESTRIAN
    from scenic.simulators.lgsvl.simulator import LGSVLSimulator
    from scenic.simulators.lgsvl.actions import *
    import scenic.simulators.lgsvl.utils as utils
except ModuleNotFoundError:
    # to allow generating static scenes without having the lgsvl package installed
    EGO_TYPE = 'EGO'
    NPC_TYPE = 'NPC'
    PEDESTRIAN_TYPE = 'PEDESTRIAN'

    import warnings
    warnings.warn('the "lgsvl" package is not installed; '
                  'will not be able to run dynamic simulations')

    def LGSVLSimulator(*args, **kwargs):
        raise RuntimeError('the "lgsvl" package is required to run simulations '
                           'from this scenario')

if 'lgsvl_map' not in globalParameters:
    raise RuntimeError('need to specify map before importing LGSVL model '
                       '(set the global parameter "lgsvl_map")')
simulator LGSVLSimulator(globalParameters.lgsvl_map)

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

class EgoCar(Vehicle, Steers):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    lgsvlAgentType: EGO_TYPE

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

(Car) = (EgoCar)

class ApolloCar(EgoCar):
    lgsvlName: 'Lincoln2017MKZ (Apollo 5.0)'
    apolloVehicle: 'Lincoln2017MKZ'
    apolloModules: ['Localization', 'Perception', 'Transform', 'Routing',
                    'Prediction', 'Planning', 'Camera']
    bridgeHost: 'localhost'
    bridgePort: 9090

    dreamview: None     # connection to Dreamview (set at runtime)

class NPCCar(NPCCar, Vehicle):
    lgsvlName: 'Sedan'
    lgsvlAgentType: NPC_TYPE

class Pedestrian(Pedestrian, LGSVLObject, Walks):
    lgsvlName: 'Bob'
    lgsvlAgentType: PEDESTRIAN_TYPE

    def setWalkingDirection(self, heading):
        super().setWalkingDirection(heading)    # TODO use better implementation?

    def setWalkingSpeed(self, speed):
        super().setWalkingSpeed(speed)

## Utility classes

class Waypoint(OrientedPoint):
    heading: roadDirection at self.position
    speed: 10
