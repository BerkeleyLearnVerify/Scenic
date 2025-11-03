"""Scenic world model for traffic scenarios in CARLA.

The model currently supports vehicles, pedestrians, and props. It implements the
basic `Car` and `Pedestrian` classes from the :obj:`scenic.domains.driving` domain,
while also providing convenience classes for specific types of objects like bicycles,
traffic cones, etc. Vehicles and pedestrians support the basic actions and behaviors
from the driving domain; several more are automatically imported from
:obj:`scenic.simulators.carla.actions` and :obj:`scenic.simulators.carla.behaviors`.

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
    snapToGroundDefault (bool): Default value for :prop:`snapToGround` on `CarlaActor` objects.
        Default is True if :ref:`2D compatibility mode` is enabled and False otherwise. 

    weather (str or dict): Weather to use for the simulation. Can be either a
        string identifying one of the CARLA weather presets (e.g. 'ClearSunset') or a
        dictionary specifying all the weather parameters (see `carla.WeatherParameters`_).
        Default is a uniform distribution over all the weather presets.

    address (str): IP address at which to connect to CARLA. Default is localhost
        (127.0.0.1).
    port (int): Port on which to connect to CARLA. Default is 2000.
    timeout (float): Maximum time to wait when attempting to connect to CARLA, in
        seconds. Default is 10.

    render (int): Whether or not to have CARLA create a window showing the
        simulations from the point of view of the ego object: 1 for yes, 0
        for no. Default 1.
    record (str): If nonempty, folder in which to save CARLA record files for
        replaying the simulations.

.. _carla.WeatherParameters: https://carla.readthedocs.io/en/latest/python_api/#carlaweatherparameters

"""
import pathlib
from scenic.domains.driving.model import *

import scenic.simulators.carla.blueprints as bp
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color

# Sensor imports
from scenic.simulators.carla.sensors import CarlaRGBSensor as RGBSensor
from scenic.simulators.carla.sensors import CarlaSSSensor as SSSensor

try:
    from scenic.simulators.carla.simulator import CarlaSimulator    # for use in scenarios
    from scenic.simulators.carla.actions import *
    from scenic.simulators.carla.actions import _CarlaVehicle, _CarlaPedestrian
    import scenic.simulators.carla.utils.utils as _utils
except ModuleNotFoundError:
    # for convenience when testing without the carla package
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('the "carla" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def CarlaSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'carla' package.

        :meta private:
        """
        raise RuntimeError('the "carla" package is required to run simulations '
                           'from this scenario')

    class _CarlaVehicle: pass
    class _CarlaPedestrian: pass

map_town = pathlib.Path(globalParameters.map).stem
param carla_map = map_town
param address = '127.0.0.1'
param port = 2000
param timeout = 10
param render = 1
if globalParameters.render not in [0, 1]:
    raise ValueError('render param must be either 0 or 1')
param record = ''
param timestep = 0.1
param weather = Uniform(
    'ClearNoon',
    'CloudyNoon',
    'WetNoon',
    'WetCloudyNoon',
    'SoftRainNoon',
    'MidRainyNoon',
    'HardRainNoon',
    'ClearSunset',
    'CloudySunset',
    'WetSunset',
    'WetCloudySunset',
    'SoftRainSunset',
    'MidRainSunset',
    'HardRainSunset'
)
param snapToGroundDefault = is2DMode()

simulator CarlaSimulator(
    carla_map=globalParameters.carla_map,
    map_path=globalParameters.map,
    address=globalParameters.address,
    port=int(globalParameters.port),
    timeout=int(globalParameters.timeout),
    render=bool(globalParameters.render),
    record=globalParameters.record,
    timestep=float(globalParameters.timestep)
)

class CarlaActor(DrivingObject):
    """Abstract class for CARLA objects.

    Properties:
        carlaActor (dynamic): Set during simulations to the ``carla.Actor`` representing this
            object.
        blueprint (str): Identifier of the CARLA blueprint specifying the type of object.
        defaultWidth (float): Default width to use if Scenic has no recorded dimensions for this blueprint. 
        defaultLength (float): Default length to use if Scenic has no recorded dimensions for this blueprint. 
        defaultHeight (float): Default height to use if Scenic has no recorded dimensions for this blueprint. 
        width (float): Width for this blueprint; uses Scenic's recorded dimensions when available, otherwise ``defaultWidth``. 
        length (float): Length for this blueprint; uses Scenic's recorded dimensions when available, otherwise ``defaultLength``. 
        height (float): Height for this blueprint; uses Scenic's recorded dimensions when available, otherwise ``defaultHeight``.
        rolename (str): Can be used to differentiate specific actors during runtime. Default
            value ``None``.
        physics (bool): Whether physics is enabled for this object in CARLA. Default true.
        snapToGround (bool): Whether or not to snap this object to the ground when placed in CARLA.
            The default is set by the ``snapToGroundDefault`` global parameter above.
    """
    carlaActor: None
    blueprint: None
    defaultWidth: 1 
    defaultLength: 1 
    defaultHeight: 1 
    width: bp.width(self.blueprint, self.defaultWidth) 
    length: bp.length(self.blueprint, self.defaultLength) 
    height: bp.height(self.blueprint, self.defaultHeight)
    rolename: None
    physics: True
    snapToGround: globalParameters.snapToGroundDefault

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

class Vehicle(CarlaActor, Vehicle, Steers, _CarlaVehicle):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.control.throttle = throttle

    def setSteering(self, steering):
        self.control.steer = steering

    def setBraking(self, braking):
        self.control.brake = braking

    def setHandbrake(self, handbrake):
        self.control.hand_brake = handbrake

    def setReverse(self, reverse):
        self.control.reverse = reverse

    def _getClosestTrafficLight(self, distance=100):
        return _getClosestTrafficLight(self, distance)

class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*bp.any_in("car"))
    defaultWidth: 2 
    defaultLength: 4.5 
    defaultHeight: 1.5

    @property
    def isCar(self):
        return True

class NPCCar(Car):  # no distinction between these in CARLA
    pass

class Bicycle(Vehicle):
    blueprint: Uniform(*bp.any_in("bicycle"))
    defaultWidth: 1 
    defaultLength: 2 
    defaultHeight: 1.5

class Motorcycle(Vehicle):
    blueprint: Uniform(*bp.any_in("motorcycle"))
    defaultWidth: 1
    defaultLength: 2 
    defaultHeight: 1.5

class Truck(Vehicle):
    blueprint: Uniform(*bp.any_in("truck"))
    defaultWidth: 2.5
    defaultLength: 7.5
    defaultHeight: 3

class Van(Vehicle):
    blueprint: Uniform(*bp.any_in("van"))
    defaultWidth: 2 
    defaultLength: 5 
    defaultHeight: 2

class Bus(Vehicle):
    blueprint: Uniform(*bp.any_in("bus"))
    defaultWidth: 4 
    defaultLength: 10
    defaultHeight: 4

class Pedestrian(CarlaActor, Pedestrian, Walks, _CarlaPedestrian):
    """A pedestrian.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.walkerModels`.
    """
    blueprint: Uniform(*bp.any_in("walker"))
    defaultWidth: 0.5 
    defaultLength: 0.5 
    defaultHeight: 1.5
    carlaController: None

    def setWalkingDirection(self, heading):
        direction = Vector(0, 1, 0).rotatedBy(heading)
        self.control.direction = _utils.scenicToCarlaVector3D(*direction)

    def setWalkingSpeed(self, speed):
        self.control.speed = speed


class Prop(CarlaActor):
    """Abstract class for props, i.e. non-moving objects.

    Properties:
        parentOrientation (Orientation): Default value overridden to have uniformly random yaw.
        physics (bool): Default value overridden to be false.
    """
    regionContainedIn: road
    position: new Point on road
    parentOrientation: Range(0, 360) deg
    defaultWidth: 0.5
    defaultLength: 0.5 
    defaultHeight: 0.5
    physics: False

class Trash(Prop):
    blueprint: Uniform(*bp.any_in("trash"))


class Cone(Prop):
    blueprint: Uniform(*bp.any_in("cone"))


class Debris(Prop):
    blueprint: Uniform(*bp.any_in("debris"))


class VendingMachine(Prop):
    blueprint: Uniform(*bp.any_in("vendingMachine"))
 

class Chair(Prop):
    blueprint: Uniform(*bp.any_in("chair"))
 

class BusStop(Prop):
    blueprint: Uniform(*bp.any_in("busStop"))


class Advertisement(Prop):
    blueprint: Uniform(*bp.any_in("advertisement"))


class Garbage(Prop):
    blueprint: Uniform(*bp.any_in("garbage"))

class Container(Prop):
    blueprint: Uniform(*bp.any_in("container"))


class Table(Prop):
    blueprint: Uniform(*bp.any_in("table"))


class Barrier(Prop):
    blueprint: Uniform(*bp.any_in("barrier"))


class PlantPot(Prop):
    blueprint: Uniform(*bp.any_in("plantpot"))


class Mailbox(Prop):
    blueprint: Uniform(*bp.any_in("mailbox"))

class Gnome(Prop):
    blueprint: Uniform(*bp.any_in("gnome"))


class CreasedBox(Prop):
    blueprint: Uniform(*bp.any_in("creasedbox"))


class Case(Prop):
    blueprint: Uniform(*bp.any_in("case"))


class Box(Prop):
    blueprint: Uniform(*bp.any_in("box"))


class Bench(Prop):
    blueprint: Uniform(*bp.any_in("bench"))


class Barrel(Prop):
    blueprint: Uniform(*bp.any_in("barrel"))


class ATM(Prop):
    blueprint: Uniform(*bp.any_in("atm"))


class Kiosk(Prop):
    blueprint: Uniform(*bp.any_in("kiosk"))


class IronPlate(Prop):
    blueprint: Uniform(*bp.any_in("ironplate"))


class TrafficWarning(Prop):
    blueprint: Uniform(*bp.any_in("trafficwarning"))


## Utility functions

def freezeTrafficLights():
    """Freezes all traffic lights in the scene.

    Frozen traffic lights can be modified by the user
    but the time will not update them until unfrozen.
    """
    simulation().world.freeze_all_traffic_lights(True)

def unfreezeTrafficLights():
    """Unfreezes all traffic lights in the scene."""
    simulation().world.freeze_all_traffic_lights(False)

def setAllIntersectionTrafficLightsStatus(intersection, color):
    for signal in intersection.signals:
        if signal.isTrafficLight:
            setTrafficLightStatus(signal, color)

def setTrafficLightStatus(signal, color):
    if not signal.isTrafficLight:
        raise RuntimeError('The provided signal is not a traffic light')
    color = utils.scenicToCarlaTrafficLightStatus(color)
    if color is None:
        raise RuntimeError('Color must be red/yellow/green/off/unknown.')
    landmarks = simulation().map.get_all_landmarks_from_id(signal.openDriveID)
    if landmarks:
        traffic_light = simulation().world.get_traffic_light(landmarks[0])
        if traffic_light is not None:
            traffic_light.set_state(color)

def getTrafficLightStatus(signal):
    if not signal.isTrafficLight:
        raise RuntimeError('The provided signal is not a traffic light')
    landmarks = simulation().map.get_all_landmarks_from_id(signal.openDriveID)
    if landmarks:
        traffic_light = simulation().world.get_traffic_light(landmarks[0])
        if traffic_light is not None:
            return utils.carlaToScenicTrafficLightStatus(traffic_light.state)
    return "None"

def _getClosestLandmark(vehicle, type, distance=100):
    if vehicle._intersection is not None:
        return None

    waypoint = simulation().map.get_waypoint(vehicle.carlaActor.get_transform().location)
    landmarks = waypoint.get_landmarks_of_type(distance, type)

    if landmarks:
        return min(landmarks, key=lambda l: l.distance)
    return None

def _getClosestTrafficLight(vehicle, distance=100):
    """Returns the closest traffic light affecting 'vehicle', up to a maximum of 'distance'"""
    landmark = _getClosestLandmark(vehicle, type="1000001", distance=distance)
    if landmark is not None:
        return simulation().world.get_traffic_light(landmark)
    return None

def withinDistanceToRedYellowTrafficLight(vehicle, thresholdDistance):
    traffic_light = _getClosestTrafficLight(vehicle, distance=thresholdDistance)
    if traffic_light is not None and str(traffic_light.state) in ("Red", "Yellow"):
        return True
    return False

def withinDistanceToTrafficLight(vehicle, thresholdDistance):
    traffic_light = _getClosestTrafficLight(vehicle, distance=thresholdDistance)
    if traffic_light is not None:
        return True
    return False

def getClosestTrafficLightStatus(vehicle, distance=100):
    traffic_light = _getClosestTrafficLight(vehicle, distance)
    if traffic_light is not None:
        return _utils.carlaToScenicTrafficLightStatus(traffic_light.state)
    return "None"

def setClosestTrafficLightStatus(vehicle, color, distance=100):
    color = _utils.scenicToCarlaTrafficLightStatus(color)
    if color is None:
        raise RuntimeError('Color must be red/yellow/green/off/unknown.')
    
    traffic_light = _getClosestTrafficLight(vehicle, distance)
    if traffic_light is not None:
        traffic_light.set_state(color)
