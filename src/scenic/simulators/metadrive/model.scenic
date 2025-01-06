from scenic.simulators.metadrive.simulator import MetaDriveSimulator
from scenic.domains.driving.model import *
from scenic.simulators.metadrive.actions import *
from scenic.simulators.metadrive.behaviors import *
from scenic.simulators.metadrive.utils import scenicToMetaDriveHeading
from metadrive.utils.math import norm


param sumo_map = globalParameters.sumo_map
param timestep = 0.1
param render = 1
param render3D = 0

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    render3D=bool(globalParameters.render3D),
    sumo_map=globalParameters.sumo_map,
)

class MetaDriveActor(DrivingObject):
    """Abstract class for MetaDrive objects."""
    metaDriveActor: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    @property
    def control(self):
        """Returns the current accumulated control inputs."""
        return self._control

    def resetControl(self):
        """Reset the control inputs after they've been applied."""
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    # def applyControl(self):
    #     """Applies the accumulated control inputs using `before_step`."""
    #     print("SELF: ", self)
    #     if isinstance(self, Car):  # Apply to cars only
    #         steering = -self._control["steering"]  # Invert the steering to match MetaDrive's convention
    #         action = [
    #             steering,
    #             self._control["throttle"] - self._control["brake"],
    #         ]
    #         self.metaDriveActor.before_step(action)

    def setPosition(self, pos):
        converted_position = scenicToMetaDrivePosition(pos, self.sumo_map)
        self.metaDriveActor.set_position(converted_position)

    def setVelocity(self, vel):
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
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def isPedestrian(self):
        return True

    def setWalkingDirection(self, heading):
        converted_heading = scenicToMetaDriveHeading(heading)
        # self.metaDriveActor.set_heading_theta(converted_heading)
        direction = Vector(math.cos(converted_heading), math.sin(converted_heading))
        self.metaDriveActor.set_velocity([direction.x, direction.y])

    def setWalkingSpeed(self, speed):
        # Get the current heading
        current_heading = self.metaDriveActor.heading_theta  # This is in radians
        # Convert heading to direction vector
        direction = Vector(math.cos(current_heading), math.sin(current_heading))
        # Set the velocity with the desired speed
        self.metaDriveActor.set_velocity([direction.x, direction.y], speed)
