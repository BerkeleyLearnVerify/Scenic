"""Scenic world model for traffic scenarios in MetaDrive.

The model currently supports vehicles and pedestrians. It implements the
basic :obj:`~scenic.domains.driving.model.Car` and `Pedestrian` classes from the :obj:`scenic.domains.driving` domain.
Vehicles and pedestrians support the basic actions and behaviors from the driving domain.

The model defines several global parameters, whose default values can be overridden
in scenarios using the ``param`` statement or on the command line using the
:option:`--param` option:

Global Parameters:
    sumo_map (str or Path): Path to the SUMO map (``.net.xml`` file) to use in the simulation.
        This map should correspond to the **map** file used in the scenario. See the documentation in
        :doc:`scenic.domains.driving.model` for details.
    timestep (float): The interval (in seconds) between each simulation step. This determines how often Scenic
        interrupts MetaDrive to run behaviors, check requirements, and update the simulation state.
        The default value is 0.1 seconds.
    render (bool): Whether to render the simulation screen. If True (default), it will open a screen and render
        the simulation. If False, rendering is disabled.
    render3D (bool): Whether to render the simulation in 3D. If True, it will render the simulation in 3D.
        If False (default), it will render in 2D.
    real_time (bool): If True (default), the simulation will run in real time, ensuring each step takes at least
        as long as the specified timestep. If False, the simulation runs as fast as possible.
"""
import pathlib

from scenic.domains.driving.model import *
from scenic.domains.driving.actions import *
from scenic.domains.driving.behaviors import *

from scenic.core.errors import InvalidScenarioError

try:
    from scenic.simulators.metadrive.simulator import MetaDriveSimulator
    from scenic.simulators.metadrive.utils import scenicToMetaDriveHeading
except ModuleNotFoundError:
    # for convenience when testing without the metadrive package
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('The "metadrive-simulator" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def MetaDriveSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'metadrive-simulator' package.

        :meta private:
        """
        raise RuntimeError('the "metadrive-simulator" package is required to run simulations '
                           'from this scenario')

if "sumo_map" not in globalParameters:
    sumo_map_path = str(pathlib.Path(globalParameters.map).with_suffix(".net.xml"))

    if not pathlib.Path(sumo_map_path).exists():
        raise InvalidScenarioError(
            f"Missing SUMO map: Expected '{sumo_map_path}' but the file does not exist.\n"
            "The SUMO map should have the same base name as the 'map' parameter, with the '.net.xml' extension.\n"
            "Ensure that the corresponding '.net.xml' file is located in the same directory as the '.xodr' file."
        )
else:
    sumo_map_path = globalParameters.sumo_map

param sumo_map = sumo_map_path
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

class Vehicle(Vehicle, Steers, MetaDriveActor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    def _reset_control(self):
        self._control = {"steering": 0, "throttle": 0, "brake": 0}

    def setThrottle(self, throttle):
        self._control["throttle"] = throttle

    def setSteering(self, steering):
        self._control["steering"] = steering

    def setBraking(self, braking):
        self._control["brake"] = braking

    def _collect_action(self):
        steering = -self._control["steering"]  # Invert the steering to match MetaDrive's convention
        action = [
            steering,
            self._control["throttle"] - self._control["brake"],
        ]
        return action

class Car(Vehicle):
    @property
    def isCar(self):
        return True

class Pedestrian(Pedestrian, MetaDriveActor, Walks):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._walking_direction = None
        self._walking_speed = None

    @property
    def isPedestrian(self):
        return True

    def setWalkingDirection(self, heading):
        self._walking_direction = scenicToMetaDriveHeading(heading)

    def setWalkingSpeed(self, speed):
        self._walking_speed = speed
