"""World model for GTA."""

import random

from scenic.simulators.gta.interface import (Map, MapWorkspace, GTA,
	CarModel)
from scenic.simulators.utils.colors import Color, ColorMutator

# Load map and set up regions, etc.
from scenic.simulators.gta.map import mapPath
if mapPath is None:
	raise RuntimeError('need to select GTA map for this scenario')
m = Map.fromFile(mapPath)

#: Vector field representing the nominal traffic direction at a point on the road
roadDirection = m.roadDirection

#: Region representing the roads in the GTA map.
road = m.roadRegion

#: Region representing the curbs in the GTA map.
curb = m.curbRegion

#: Workspace over the `road` Region.
workspace = MapWorkspace(m, road)

# Default values for time and weather
param time = Range(0 * 60, 24 * 60)	# 0000 to 2400 hours
param weather = Options({
			'NEUTRAL': 5,
			'CLEAR': 15,
			'EXTRASUNNY': 20,
			'CLOUDS': 15,
			'OVERCAST': 15,
			'RAIN': 5,
			'THUNDER': 5,
			'CLEARING': 5,
			'FOGGY': 5,
			'SMOG': 5,
			'XMAS': 1.25,
			'SNOWLIGHT': 1.25,
			'BLIZZARD': 1.25,
			'SNOW': 1.25
			})

class Car:
	"""Scenic class for cars.

	Properties:
		position: The default position is uniformly random over the `road`.
		heading: The default heading is aligned with `roadDirection`, plus an offset
		  given by ``roadDeviation``.
		roadDeviation (float): Relative heading with respect to the road direction
		  at the `Car`'s position. Used by the default value for ``heading``.
		model (`CarModel`): Model of the car.
		color (:obj:`Color` or RGB tuple): Color of the car.
	"""
	position: Point on road
	heading: (roadDirection at self.position) + self.roadDeviation
	roadDeviation: 0
	width: self.model.width
	length: self.model.length
	viewAngle: 80 deg
	visibleDistance: 30
	model: CarModel.defaultModel()
	color: Color.defaultCarColor()

	mutator[additive]: ColorMutator()

class EgoCar(Car):
	"""Convenience subclass with defaults for ego cars."""
	model: CarModel.egoModel()

class Bus(Car):
	"""Convenience subclass for buses."""
	model: CarModel.models['BUS']

class Compact(Car):
	"""Convenience subclass for compact cars."""
	model: CarModel.models['BLISTA']

def createPlatoonAt(car, numCars, model=None, dist=Range(2, 8),
                    shift=Range(-0.5, 0.5), wiggle=0):
	"""Create a platoon starting from the given car."""
	cars = [car]
	lastCar = car
	for i in range(numCars-1):
		center = follow roadDirection from (front of lastCar) for resample(dist)
		pos = OrientedPoint right of center by shift, facing resample(wiggle) relative to roadDirection
		lastCar = Car ahead of pos, with model (car.model if model is None else resample(model))
		cars.append(lastCar)
	return cars
