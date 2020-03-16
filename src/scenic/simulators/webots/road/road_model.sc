"""Scenic world model for traffic scenarios in Webots."""

import math
import time

import scenic.simulators.webots.world_parser as parser
from scenic.simulators.webots.road.interface import WebotsWorkspace
from scenic.simulators.webots.road.world import worldPath
from scenic.simulators.webots.road.car_models import CarModel, carModels, \
	modelWithName, smallVehicles

from scenic.simulators.gta.interface import CarColor	# TODO refactor

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

constructor WebotsObject:
	pass

constructor Car(WebotsObject):
	regionContainedIn: road
	position: Point on road
	heading: (roadDirection at self.position) + self.roadDeviation
	roadDeviation: 0
	model: Uniform(*carModels)
	width: self.model.width
	height: self.model.height
	webotsType: self.model.name
	viewAngle: 90 deg
	cameraOffset: 0 @ (self.height / 2)		# camera is at the front
	color: CarColor.defaultColor()

constructor SmallCar(Car):
	model: Uniform(*smallVehicles)

constructor BmwX5(Car):
	model: modelWithName['BmwX5']

constructor CitroenCZero(Car):
	model: modelWithName['CitroenCZero']

constructor LincolnMKZ(Car):
	model: modelWithName['LincolnMKZ']

constructor RangeRoverSportSVR(Car):
	model: modelWithName['RangeRoverSportSVR']

constructor ToyotaPrius(Car):
	model: modelWithName['ToyotaPrius']

constructor Bus(Car):
	model: modelWithName['Bus']

constructor Truck(Car):
	model: modelWithName['Truck']

constructor Tractor(Car):
	model: modelWithName['Tractor']

constructor Motorcycle(Car):
	model: modelWithName['MotorBikeSimple']
	primaryColor: CarColor.defaultColor()
	secondaryColor: CarColor.uniformColor()		# TODO improve

constructor Pedestrian(WebotsObject):
	regionContainedIn: walkway
	position: Point on walkway
	heading: (0, 360) deg
	width: 0.5
	height: 0.5
	shirtColor: CarColor.uniformColor()
	pantsColor: CarColor.uniformColor()
	shoesColor: CarColor.uniformColor()

constructor OilBarrel(WebotsObject):
	width: 0.61
	height: 0.61

constructor SolidBox(WebotsObject):
	width: 2
	height: 2

constructor TrafficCone(WebotsObject):
	width: 0.5
	height: 0.5

constructor WorkBarrier(WebotsObject):
	width: 1.2
	height: 0.4
