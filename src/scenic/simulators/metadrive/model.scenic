from scenic.simulators.metadrive.simulator import MetaDriveSimulator
from scenic.domains.driving.model import *
from scenic.simulators.metadrive.actions import *
from scenic.simulators.metadrive.behaviors import *
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
param render3D = 0  # Default to 0 (2D view)

# (xmin, ymin), (xmax, ymax), _ = road.AABB

# Handle both 2D and 3D AABB cases from road.AABB
road_aabb = road.AABB
# print("ROAD AABB: ", road.AABB)
if len(road_aabb[0]) == 2:  # 2D case
    (xmin, ymin), (xmax, ymax) = road_aabb
else:  # 3D case
    (xmin, ymin, _), (xmax, ymax, _) = road_aabb

# NOTE: MetaDrive currently has their own coordinate
# system where (0,0) is centered around the middle of
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172

center_x = (xmin + xmax) / 2
center_y = (ymin + ymax) / 2
param center_x = center_x
param center_y = center_y

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    render3D=bool(globalParameters.render3D),
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
        # Initialize the control dictionary
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    @property
    def control(self):
        """Returns the current accumulated control inputs."""
        return self._control

    def resetControl(self):
        """Reset the control inputs after they've been applied."""
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    def applyControl(self):
        """Applies the accumulated control inputs using `before_step`."""
        action = [
            self._control["steering"],
            self._control["throttle"] - self._control["brake"],
        ]
        self.metaDriveActor.before_step(action)

    def setPosition(self, pos):
        self.metaDriveActor.last_position = scenicToMetaDrivePosition(pos, center_x, center_y)

    def setVelocity(self, vel):
        # need to change
        self.metaDriveActor.before_step([0, vel])


class Vehicle(Vehicle, Steers, MetaDriveActor):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.control["throttle"] = throttle

    def setSteering(self, steering):
        self.control["steering"] = steering

    def setBraking(self, braking):
        self.control["brake"] = braking


class Car(Vehicle):
    @property
    def isCar(self):
        return True
