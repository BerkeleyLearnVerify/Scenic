import math
import time

from scenic.simulators.carla.interface import CarlaWorkspace
from scenic.simulators.carla.map import mapPath
from scenic.simulators.carla.car_models import carModels, bicycleModels, motorcycleModels, truckModels
from scenic.simulators.carla.prop_models import trashModels, coneModels

from scenic.simulators.gta.interface import CarColor

workspace = CarlaWorkspace(mapPath)

roadDirection = workspace.road_direction
road = workspace.drivable_region
sidewalk = workspace.sidewalk_region
intersection = workspace.intersection_region
# laneSectionDict is a dict from road id to a list of dicts from
# lane id to polygon, one dict per lane section.
laneSectionDict = workspace.lane_sec_dict

precipitation = Options({0: 70,1: 30}) * (0, 100)
param precipitation = precipitation
param precipitation_deposits = (precipitation, 100)
param cloudiness = (precipitation, 100)
param wind_intensity = (0, 100)
param sun_azimuth_angle = (0, 360)
param sun_altitude_angle = (-90, 90)

# TODO: Get vehicle models, dimensions from Carla
constructor Vehicle:
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2
    height: 5
    color: CarColor.defaultColor()

constructor Car(Vehicle):
    blueprint: Uniform(*carModels)

constructor Bicycle(Vehicle):
    width: 1
    height: 2
    blueprint: Uniform(*bicycleModels)

constructor Motorcycle(Vehicle):
    width: 1
    height:2
    blueprint: Uniform(*motorcycleModels)

constructor Truck(Vehicle):
    width: 3
    height: 7
    blueprint: Uniform(*truckModels)

constructor Pedestrian:
    regionContainedIn: sidewalk
    position: Point on sidewalk
    heading: (0, 360) deg
    width: 0.5
    height: 0.5

constructor Prop:
    regionContainedIn: road
    position: Point on road
    heading: (0, 360) deg
    width: 0.5
    height: 0.5

constructor Trash(Prop):
    blueprint: Uniform(*trashModels)

constructor Cone(Prop):
    blueprint: Uniform(*coneModels)
