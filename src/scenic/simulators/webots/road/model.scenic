"""Scenic world model for traffic scenarios in Webots."""

import math
import time

import scenic.simulators.webots.world_parser as parser
from scenic.simulators.webots.road.interface import WebotsWorkspace
from scenic.simulators.webots.road.world import worldPath
from scenic.simulators.webots.road.car_models import CarModel, carModels, \
	modelWithName, smallVehicles

from scenic.simulators.utils.colors import Color as CarColor

# Load map and set up workspace

if worldPath is None:
	raise RuntimeError('need to set Webots world for this scenario')
startTime = time.time()
world = parser.parse(worldPath)
totalTime = time.time() - startTime
verbosePrint(f'Parsed .wbt file in {totalTime:.3} seconds.')

workspace = WebotsWorkspace(world)

# Set up various regions

roadDirection = workspace.roadDirection
road = workspace.drivableRegion
curb = workspace.curbsRegion
intersection = workspace.crossroadsRegion
nonIntersection = workspace.roadsRegion
sidewalk = workspace.sidewalksRegion
crossing = workspace.crossingsRegion
walkway = workspace.walkableRegion

# types of objects

class WebotsObject:
	webotsName: 'unspecified_name'
	webotsObject: None 	# gets filled in at simulation time
	elevation: None 	# ditto (this is the Webots y coordinate)

class Car(WebotsObject):
	regionContainedIn: road
	position: Point on road
	heading: (roadDirection at self.position) + self.roadDeviation
	roadDeviation: 0
	model: Uniform(*carModels)
	width: self.model.width
	length: self.model.length
	webotsType: self.model.name
	viewAngle: 90 deg
	cameraOffset: 0 @ (self.length / 2)		# camera is at the front
	color: CarColor.defaultCarColor()

class SmallCar(Car):
	model: Uniform(*smallVehicles)

class BmwX5(Car):
	model: modelWithName['BmwX5']

class CitroenCZero(Car):
	model: modelWithName['CitroenCZero']

class LincolnMKZ(Car):
	model: modelWithName['LincolnMKZ']

class RangeRoverSportSVR(Car):
	model: modelWithName['RangeRoverSportSVR']

class ToyotaPrius(Car):
	model: modelWithName['ToyotaPrius']

class Bus(Car):
	model: modelWithName['Bus']

class Truck(Car):
	model: modelWithName['Truck']

class Tractor(Car):
	model: modelWithName['Tractor']

class Motorcycle(Car):
	model: modelWithName['MotorBikeSimple']
	primaryColor: CarColor.defaultCarColor()
	secondaryColor: CarColor.uniformColor()		# TODO improve

class Pedestrian(WebotsObject):
	regionContainedIn: walkway
	position: Point on walkway
	heading: Range(0, 360) deg
	width: 0.5
	length: 0.5
	shirtColor: CarColor.uniformColor()
	pantsColor: CarColor.uniformColor()
	shoesColor: CarColor.uniformColor()

class OilBarrel(WebotsObject):
	width: 0.61
	length: 0.61

class SolidBox(WebotsObject):
	width: 2
	length: 2

class TrafficCone(WebotsObject):
	width: 0.5
	length: 0.5

class WorkBarrier(WebotsObject):
	width: 1.2
	length: 0.4
