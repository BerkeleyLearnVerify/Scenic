"""Scenic world model for traffic scenarios in MetaDrive.

The model currently supports vehicles and pedestrians. It implements the
basic `Vehicle`, `Car` and `Pedestrian` classes from the :obj:`scenic.domains.driving` domain.
Vehicles and pedestrians support the basic actions and behaviors from the driving domain.

The model defines several global parameters, whose default values can be overridden
in scenarios using the ``param`` statement or on the command line using the
:option:`--param` option:

Global Parameters:
    sumo_map (str or Path): Path to the SUMO map (``.net.xml`` file) to use in the simulation.
        This map should correspond to the **map** file used in the scenario, which must be in the OpenDrive
        format (``.xodr`` file).
    timestep (float): The interval (in seconds) between each simulation step. This determines how often Scenic
        interrupts MetaDrive to run behaviors, check requirements, and update the simulation state.
        The default value is 0.1 seconds.
    render (bool): Whether to render the simulation screen. If True (default), it will open a screen and render
        the simulation. If False, rendering is disabled.
    render3D (bool): Whether to render the simulation in 3D. If True, it will render the simulation in 3D.
        If False (default), it will render in 2D.
    real_time (bool): If True (default), the simulation will run in real time, ensuring each step takes at least
        as long as the specified timestep. If False, the simulation may run faster, based on the time it takes
        to process each step.
"""
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
param real_time = 1

simulator MetaDriveSimulator(
    sumo_map=globalParameters.sumo_map,
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    render3D=bool(globalParameters.render3D),
    real_time=bool(globalParameters.real_time),
)

class MetaDriveActor(DrivingObject):
    """Abstract class for MetaDrive objects.

    This class serves as a base for objects in the MetaDrive simulator. It provides essential
    functionality for associating Scenic objects with their corresponding MetaDrive simulation objects.

    Properties:
        metaDriveActor: A reference to the MetaDrive actor (e.g., vehicle or pedestrian) associated
                        with this Scenic object. This is set when the object is created in the simulator.
                        It allows interaction with MetaDrive's simulation environment, such as applying actions
                        or retrieving simulation data (position, velocity, etc.).
    """
    metaDriveActor: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Vehicle(Vehicle, Steers, MetaDriveActor):
    """Abstract class for steerable vehicles."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    @property
    def control(self):
        """Returns the current accumulated control inputs (throttle, brake, and steering)."""
        return self._control

    def resetControl(self):
        """Reset the control inputs after they've been applied."""
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    def setThrottle(self, throttle):
        self.control["throttle"] = throttle

    def setSteering(self, steering):
        self.control["steering"] = steering

    def setBraking(self, braking):
        self.control["brake"] = braking

    def collectAction(self):
        """For vehicles, accumulate the throttle, brake, and steering, and return the action."""
        steering = -self.control["steering"]  # Invert the steering to match MetaDrive's convention
        action = [
            steering,
            self.control["throttle"] - self.control["brake"],
        ]
        return action

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
        converted_heading = scenicToMetaDriveHeading(self.heading)
        direction = Vector(math.cos(converted_heading), math.sin(converted_heading))
        self.metaDriveActor.set_velocity([direction.x, direction.y], self.speed)

    def setWalkingSpeed(self, speed):
        current_heading = scenicToMetaDriveHeading(self.heading)
        direction = Vector(math.cos(current_heading), math.sin(current_heading))
        self.metaDriveActor.set_velocity([direction.x, direction.y], speed)
