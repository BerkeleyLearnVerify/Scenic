"""Scenic world model for traffic scenarios in CARLA."""

import scenic.simulators.domains.driving.model as baseModel

import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.simulator import CarlaSimulator    # for use in scenarios

from scenic.simulators.utils.colors import Color


workspace = baseModel.workspace

network = baseModel.network
road = baseModel.road
roadDirection = baseModel.roadDirection
sidewalk = baseModel.sidewalk
intersection = baseModel.intersection

precipitation = Options({0: 70, 1: 30}) * (0, 100)
param precipitation = precipitation
param precipitation_deposits = (precipitation, 100)
param cloudiness = (precipitation, 100)
param wind_intensity = (0, 100)
param sun_azimuth_angle = (0, 360)
param sun_altitude_angle = (-90, 90)


class CarlaActor:
    carlaActor: None
    blueprint: None
    elevation: None
    speed: None
    color: None


class Vehicle(CarlaActor):
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2
    height: 5
    color: Color.defaultCarColor()


class Car(Vehicle):
    blueprint: Uniform(*blueprints.carModels)


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


class Pedestrian(CarlaActor):
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5
    blueprint: Uniform(*blueprints.walkerModels)


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
