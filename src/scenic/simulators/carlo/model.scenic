"""Scenic world model for traffic scenarios in CARLO (low-budget CARLA).

The model supports the current CARLO functionality, which allows for custom
environment creation (through collidable and non-collidable decorations which
can comprise roads and off-roads, respectively) as well as the instantiation
of Cars and Pedestrians in the Carlo class. The speed and heading of these
classes can be set through the simple SetControl behavior.

The model defines several global parameters, whose default values can be overridden
in scenarios using the ``param`` statement or on the command line using the
:option:`--param` option:

Global Parameters:
    carla_map (str): Name of the CARLA map to use, e.g. 'Town01'. Can also be set
        to ``None``, in which case CARLA will attempt to create a world from the
        **map** file used in the scenario (which must be an ``.xodr`` file).
    timestep (float): Timestep to use for simulations (i.e., how frequently Scenic
        interrupts CARLA to run behaviors, check requirements, etc.), in seconds. Default
        is 0.1 seconds.

    render (int): Whether or not to have CARLA create a window showing the
        simulations from the point of view of the ego object: 1 for yes, 0
        for no. Default 1.
    record (str): If nonempty, folder in which to save CARLA record files for
        replaying the simulations.


"""

from scenic.domains.driving.model import *

from scenic.simulators.utils.colors import Color

try:
    from scenic.simulators.CARLO.simulator import CarloSimulator    # for use in scenarios
    from scenic.simulators.CARLO.actions import *
    import scenic.simulators.CARLO.utils.utils as _utils
except ModuleNotFoundError:
    # for convenience when testing without the carla package
    import warnings
    warnings.warn('the "carlo" library is not present; '
                  'will not be able to run simulations')

    def CarloSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'carlo' library.

        :meta private:
        """
        raise RuntimeError('the "carlo" library is required to run simulations '
                           'from this scenario')

param carla_map = None
param render = 1
if globalParameters.render not in [0, 1]:
    raise ValueError('render param must be either 0 or 1')
param record = ''
param timestep = 0.1

simulator CarloSimulator(
    world=
    render=bool(globalParameters.render),
)

class CarloActor(DrivingObject):
    """Abstract class for CARLA objects.

    Properties:
        carlaActor (dynamic): Set during simulations to the ``carla.Actor`` representing this
            object.
        blueprint (str): Identifier of the CARLA blueprint specifying the type of object.
        rolename (str): Used to identify. Default is "Car", options are "Car" and "Pedestrian."
            value ``None``.
    """
    rolename: "Car"
    color: None
    physics: True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    @property
    def control(self):
        if self._control is None:
            self._control = self.carlaActor.get_control()
        return self._control

    def setPosition(self, pos, elevation):
        self.carlaActor.set_location(_utils.scenicToCarlaLocation(pos, elevation))

    def setVelocity(self, vel):
        cvel = _utils.scenicToCarlaVector3D(*vel)
        if hasattr(self.carlaActor, 'set_target_velocity'):
            self.carlaActor.set_target_velocity(cvel)
        else:
            self.carlaActor.set_velocity(cvel)

class Vehicle(Vehicle, CarlaActor, Steers):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.control.throttle = throttle

    def setSteering(self, steering):
        self.control.steer = steering


class Car(Vehicle):
    """A car.
    """
    pass

class NPCCar(Car):  # no distinction between these in CARLA
    pass


class Pedestrian(Pedestrian, CarlaActor, Walks):
    """A pedestrian.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in `scenic.simulators.carla.blueprints.walkerModels`.
    """
    width: 0.5
    length: 0.5
    blueprint: Uniform(*blueprints.walkerModels)
    carlaController: None

    def setWalkingDirection(self, heading):
        direction = Vector(0, 1).rotatedBy(heading)
        zComp = self.control.direction.z
        self.control.direction = _utils.scenicToCarlaVector3D(*direction, zComp)

    def setWalkingSpeed(self, speed):
        self.control.speed = speed


## Utility functions

def _getClosestLandmark(vehicle, type, distance=100):
    if vehicle._intersection is not None:
        return None

    waypoint = simulation().map.get_waypoint(vehicle.carlaActor.get_transform().location)
    landmarks = waypoint.get_landmarks_of_type(distance, type)

    if landmarks:
        return min(landmarks, key=lambda l: l.distance)
    return None