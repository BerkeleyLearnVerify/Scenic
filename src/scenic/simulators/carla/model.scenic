"""Scenic world model for traffic scenarios in CARLA."""

from scenic.domains.driving.model import *

import scenic.simulators.carla.blueprints as blueprints

try:
    from scenic.simulators.carla.simulator import CarlaSimulator    # for use in scenarios
    from scenic.simulators.carla.actions import *
    from scenic.simulators.carla.behaviors import *
except ModuleNotFoundError:
    # for convenience when testing without the carla package
    import warnings
    warnings.warn('the "carla" package is not installed; '
                  'will not be able to run dynamic simulations')

from scenic.simulators.utils.colors import Color


precipitation = Options({0: 70, 1: 30}) * (0, 100)
param precipitation = precipitation
param precipitation_deposits = (precipitation, 100)
param cloudiness = (precipitation, 100)
param wind_intensity = (0, 100)
param sun_azimuth_angle = (0, 360)
param sun_altitude_angle = (-90, 90)


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
        self.carlaActor.set_velocity(utils.scenicToCarlaVector3D(*vel))


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
    height: 2
    blueprint: Uniform(*blueprints.bicycleModels)


class Motorcycle(Vehicle):
    width: 1
    height:2
    blueprint: Uniform(*blueprints.motorcycleModels)


class Truck(Vehicle):
    width: 3
    height: 7
    blueprint: Uniform(*blueprints.truckModels)


class Pedestrian(Pedestrian, CarlaActor, Walks):
    width: 0.5
    height: 0.5
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
    heading: (0, 360) deg
    width: 0.5
    height: 0.5


class Trash(Prop):
    blueprint: Uniform(*blueprints.trashModels)


class Cone(Prop):
    blueprint: Uniform(*blueprints.coneModels)
