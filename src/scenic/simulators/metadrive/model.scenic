from scenic.simulators.metadrive.simulator import MetaDriveSimulator
from scenic.domains.driving.model import *
from scenic.simulators.metadrive.actions import *
from scenic.simulators.metadrive.behaviors import *
from scenic.simulators.metadrive.utils import extractNetOffsetAndBoundary
import pathlib

try:
    map_town = pathlib.Path(globalParameters.map).stem
except Exception as e:
    raise RuntimeError("'map' is not defined") from e
try:
    sumo_map_path = pathlib.Path(globalParameters.sumo_map)
except Exception as e:
    raise RuntimeError("'sumo_map' is not defined") from e

param map = map_town
param sumo_map = sumo_map_path.stem
param timestep = 0.1
param render = 1
param render3D = 0

# NOTE: MetaDrive currently has their own coordinate
# system where (0,0) is centered around the middle of
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172

# Extract network offset and boundary for SUMO map
net_offset, sumo_map_boundary = extractNetOffsetAndBoundary(sumo_map_path)
if net_offset and sumo_map_boundary:
    xmin, ymin, xmax, ymax = sumo_map_boundary
    center_x = (xmin + xmax) / 2
    center_y = (ymin + ymax) / 2
    offset_x = net_offset[0]
    offset_y = net_offset[1]
else:
    raise RuntimeError("Failed to extract netOffset or convBoundary from SUMO map.")

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    render3D=bool(globalParameters.render3D),
    sumo_map=globalParameters.sumo_map,
    center_x = center_x,
    center_y = center_y,
    offset_x = offset_x,
    offset_y = offset_y,
    sumo_map_boundary = sumo_map_boundary,
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
        steering = -self._control["steering"]  # Invert the steering to match MetaDrive's convention
        action = [
            steering,
            self._control["throttle"] - self._control["brake"],
        ]
        # print(f"Scenic Controls for {self.metaDriveActor} - Steering: {self._control['steering']}, "
        #     f"Throttle: {self._control['throttle']}, Brake: {self._control['brake']}")
        self.metaDriveActor.before_step(action)

    def setPosition(self, pos):
        converted_position = scenicToMetaDrivePosition(pos, center_x, center_y, offset_x, offset_y)
        self.metaDriveActor.set_position(converted_position)
        # self.metaDriveActor.last_position = scenicToMetaDrivePosition(pos, center_x, center_y, offset_x, offset_y)

    def setVelocity(self, vel):
        # TODO
        # xVel, yVel, _ = vel  # Extract 2D components; ignore zVel

        # vel_vector = np.array([xVel, yVel])  # Create a 2D velocity vector
        # direction = vel_vector / (np.linalg.norm(vel_vector) + 1e-6)  # Normalize
        # speed = np.linalg.norm(vel_vector)  # Calculate speed

        # # Use MetaDrive's `set_velocity` API
        # self.metaDriveActor.set_velocity(direction, speed)

        # need to change
        # self.metaDriveActor.before_step([0, vel])
        raise NotImplementedError


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

class Pedestrian(Pedestrian, MetaDriveActor, Walks):

    def setWalkingDirection(self, heading):
        # TODO
        direction = Vector(0, 1, 0).rotatedBy(heading)
        self.control.direction = _utils.scenicToCarlaVector3D(*direction)

    def setWalkingSpeed(self, speed):
        # TODO
        self.control.speed = speed
