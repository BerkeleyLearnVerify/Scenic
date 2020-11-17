"""Scenic world model for traffic scenarios in CARLA.

The model currently supports vehicles, pedestrians, and props.

The model uses several global parameters to control weather (descriptions
are from the CARLA Python API reference):

    * ``cloudiness`` (float):
      Weather cloudiness. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``precipitation`` (float):
      Precipitation amount for controlling rain intensity. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``precipitation_deposits`` (float):
      Precipitation deposits for controlling the area of puddles on roads. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``wind_intensity`` (float):
      Wind intensity, it affects the clouds moving speed, the raindrop direction, and vegetation. This doesn't affect the car physics. Values range from 0 to 100.
    * ``sun_azimuth_angle`` (float):
      The azimuth angle of the sun in degrees. Values range from 0 to 360 (degrees).
    * ``sun_altitude_angle`` (float):
      Altitude angle of the sun in degrees. Values range from -90 to 90 (where 0 degrees is the horizon).
"""

from scenic.domains.driving.model import *

import scenic.simulators.carla.blueprints as blueprints
import scenic.simulators.carla.utils.utils as utils
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color

try:
    from scenic.simulators.carla.simulator import CarlaSimulator    # for use in scenarios
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    # for convenience when testing without the carla package
    import warnings
    warnings.warn('the "carla" package is not installed; '
                  'will not be able to run dynamic simulations')

    def CarlaSimulator(*args, **kwargs):
        raise RuntimeError('the "carla" package is required to run simulations '
                           'from this scenario')

if 'carla_map' not in globalParameters:
    raise RuntimeError('need to specify map before importing CARLA model '
                       '(set the global parameter "carla_map")')
simulator CarlaSimulator(globalParameters.carla_map)

precipitation = Options({0: 70, 1: 30}) * Range(0, 100)
param precipitation = precipitation
param precipitation_deposits = Range(precipitation, 100)
param cloudiness = Range(precipitation, 100)
param wind_intensity = Range(0, 100)
param sun_azimuth_angle = Range(0, 360)
param sun_altitude_angle = Range(-90, 90)


class CarlaActor(DrivingObject):
    carlaActor: None
    blueprint: None
    color: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    @property
    def control(self):
        if self._control is None:
            self._control = self.carlaActor.get_control()
        return self._control

    def setPosition(self, pos, elevation):
        self.carlaActor.set_location(utils.scenicToCarlaLocation(pos, elevation))

    def setVelocity(self, vel):
        self.carlaActor.set_target_velocity(utils.scenicToCarlaVector3D(*vel))


class Vehicle(Vehicle, CarlaActor, Steers):
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


class Car(Vehicle):
    blueprint: Uniform(*blueprints.carModels)

class NPCCar(Car):  # no distinction between these in CARLA
    pass

class Bicycle(Vehicle):
    width: 1
    length: 2
    blueprint: Uniform(*blueprints.bicycleModels)


class Motorcycle(Vehicle):
    width: 1
    length:2
    blueprint: Uniform(*blueprints.motorcycleModels)


class Truck(Vehicle):
    width: 3
    length: 7
    blueprint: Uniform(*blueprints.truckModels)


class Pedestrian(Pedestrian, CarlaActor, Walks):
    width: 0.5
    length: 0.5
    blueprint: Uniform(*blueprints.walkerModels)

    def setWalkingDirection(self, heading):
        direction = Vector(0, self.speed).rotatedBy(heading)
        zComp = self.control.direction.z
        self.control.direction = utils.scenicToCarlaVector3D(*direction, zComp)

    def setWalkingSpeed(self, speed):
        self.control.speed = speed


class Prop(CarlaActor):
    regionContainedIn: road
    position: Point on road
    heading: Range(0, 360) deg
    width: 0.5
    length: 0.5


class Trash(Prop):
    blueprint: Uniform(*blueprints.trashModels)


class Cone(Prop):
    blueprint: Uniform(*blueprints.coneModels)


class Debris(Prop):
    blueprint: Uniform(*blueprints.debrisModels)


class VendingMachine(Prop):
    blueprint: Uniform(*blueprints.vendingMachineModels)


class Chair(Prop):
    blueprint: Uniform(*blueprints.chairModels)


class BusStop(Prop):
    blueprint: Uniform(*blueprints.busStopsModels)


class Advertisement(Prop):
    blueprint: Uniform(*blueprints.advertisementModels)


class Garbage(Prop):
    blueprint: Uniform(*blueprints.garbageModels)


class Container(Prop):
    blueprint: Uniform(*blueprints.containerModels)


class Table(Prop):
    blueprint: Uniform(*blueprints.tableModels)


class Barrier(Prop):
    blueprint: Uniform(*blueprints.barrierModels)


class PlantPot(Prop):
    blueprint: Uniform(*blueprints.plantpotModels)


class Mailbox(Prop):
    blueprint: Uniform(*blueprints.mailboxModels)


class Gnome(Prop):
    blueprint: Uniform(*blueprints.gnomeModels)


class CreasedBox(Prop):
    blueprint: Uniform(*blueprints.creasedboxModels)


class Case(Prop):
    blueprint: Uniform(*blueprints.caseModels)


class Box(Prop):
    blueprint: Uniform(*blueprints.boxModels)


class Bench(Prop):
    blueprint: Uniform(*blueprints.benchModels)


class Barrel(Prop):
    blueprint: Uniform(*blueprints.barrelModels)


class ATM(Prop):
    blueprint: Uniform(*blueprints.atmModels)


class Kiosk(Prop):
    blueprint: Uniform(*blueprints.kioskModels)


class IronPlate(Prop):
    blueprint: Uniform(*blueprints.ironplateModels)


class TrafficWarning(Prop):
    blueprint: Uniform(*blueprints.trafficwarningModels)
