from scenic.simulators.metadrive.simulator import MetaDriveSimulator 
from scenic.domains.driving.model import *
from scenic.simulators.metadrive.actions import *
import pathlib

try:
    map_town = pathlib.Path(globalParameters.map).stem
except Exception as e:
    raise RuntimeError("'map' is not defined") from e
try:
    sumo_map = pathlib.Path(globalParameters.sumo_map).stem
except Exception as e:
    raise RuntimeError("'sumo_map' is not defined") from e

param map = map_town
param sumo_map = sumo_map
param timestep = 0.1
param render = 1

(xmin, ymin), (xmax, ymax), _ = road.AABB

# NOTE: MetaDrive currently has their own coordinate 
# system where (0,0) is centered around the middle of 
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172
center_x = abs(xmin + xmax)/2
center_y = abs(ymin + ymax)/2
param center_x = center_x
param center_y = center_y

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    sumo_map=globalParameters.sumo_map,
    center_x = globalParameters.center_x,
    center_y = globalParameters.center_y,
)

class MetaDriveActor(DrivingObject):
    """Abstract class for MetaDrive objects.

    Properties:
        metaDriveActor (dynamic): Set during simulations to the ``metaDrive.Actor`` representing this
            object. Serves as entrypoint to MetaDrive Object class and relevant APIs
    """
    metaDriveActor: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setPosition(self, pos):
        self.metaDriveActor.last_position = pos

    def setVelocity(self, vel):
        self.metaDriveActor.before_step([0, vel])


class Vehicle(Vehicle, Steers, MetaDriveActor):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.metaDriveActor.before_step([0, throttle])

    def setSteering(self, steering):
        self.metaDriveActor.before_step([steering, 0])

    def setBraking(self, braking):
        self.metaDriveActor.before_step([0, -braking])


class Car(Vehicle):
    @property
    def isCar(self):
        return True