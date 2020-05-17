"""Scenic world model for traffic scenarios in CARLA."""

import math
import time

import scenic.simulators.domains.driving.model as baseModel

from scenic.simulators.carla.models.vehicle_models import carModels, bicycleModels, motorcycleModels, truckModels
from scenic.simulators.carla.models.walker_models import walkerModels
from scenic.simulators.carla.models.prop_models import trashModels, coneModels

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
    blueprint: Uniform(*carModels)


class Bicycle(Vehicle):
    width: 1
    height: 2
    blueprint: Uniform(*bicycleModels)


class Motorcycle(Vehicle):
    width: 1
    height:2
    blueprint: Uniform(*motorcycleModels)


class Truck(Vehicle):
    width: 3
    height: 7
    blueprint: Uniform(*truckModels)


class Pedestrian(CarlaActor):
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5
    blueprint: Uniform(*walkerModels)


class Prop(CarlaActor):
    regionContainedIn: road
    position: Point on road
    heading: (0, 360) deg
    width: 0.5
    height: 0.5


class Trash(Prop):
    blueprint: Uniform(*trashModels)


class Cone(Prop):
    blueprint: Uniform(*coneModels)
