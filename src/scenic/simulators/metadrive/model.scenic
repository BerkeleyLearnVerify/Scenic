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

# NOTE: MetaDrive currently has their own coordinate
# system where (0,0) is centered around the middle of
# the SUMO Map. To preserve the original SUMO map coordinates
# we will offset by the computed center x and y coordinates
# https://github.com/metadriverse/metadrive/blob/aaed1f7f2512061ddd8349d1d411e374dab87a43/metadrive/utils/sumo/map_utils.py#L165-L172

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    render3D=bool(globalParameters.render3D),
    sumo_map=globalParameters.sumo_map,
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
        converted_position = scenicToMetaDrivePosition(pos, self.sumo_map)
        self.metaDriveActor.set_position(converted_position)

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
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Initialize the current speed model, default to the first option in SPEED_LIST
        self.current_speed_model = 0.4  # Default speed, can be adjusted

    def setWalkingDirection(self, heading):
        print("HERE IN SET_WALKING_DIRECTION")

        metadrive_heading = scenicToMetaDriveHeading(heading)

        # Calculate the direction vector in MetaDrive using the heading
        direction = Vector(0, self.current_speed_model).rotatedBy(metadrive_heading)

        # Use MetaDrive's set_velocity to update the pedestrian's velocity
        self.set_velocity([direction.x, direction.y])

    def setWalkingSpeed(self, speed):
        print("HERE IN SET_WALKING_SPEED")

        # Adjust speed in MetaDrive using the set_velocity method
        if hasattr(self, 'velocity') and self.velocity is not None:
            direction = self.velocity.normalized()  # Keep current walking direction
        else:
            # If no velocity has been set yet, default to the current direction
            direction = Vector(0, self.current_speed_model).rotatedBy(0)  # Default direction

        # Apply new speed
        self.set_velocity([direction.x, direction.y], speed)

    def set_velocity(self, velocity, value=None):
        """
        Update the pedestrian's velocity.
        This method mimics the `set_velocity` from MetaDrive's Pedestrian class.
        """
        # If a specific speed is provided, adjust the velocity accordingly
        if value is not None:
            velocity_magnitude = value  # Use the specified speed
        else:
            velocity_magnitude = norm(velocity[0], velocity[1])  # Calculate magnitude of velocity vector

        # Create a new velocity vector, normalize and apply the magnitude
        direction = Vector(*velocity)
        direction = direction.normalized() * velocity_magnitude
        self.velocity = direction  # Update velocity

        print(f"Setting velocity to: {self.velocity}")
        # Further logic for moving the pedestrian can be added here
