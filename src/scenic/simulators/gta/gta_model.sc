
import random

from scenic.simulators.gta.interface import Map, MapWorkspace, GTA, \
	CarModel, CarColor, NoisyColorDistribution

# Load map and set up regions, etc.
from scenic.simulators.gta.map import mapPath
if mapPath is None:
	raise RuntimeError('need to select GTA map for this scenario')
m = Map.fromFile(mapPath)

roadDirection = m.roadDirection
road = m.roadRegion
curb = m.curbRegion

workspace = MapWorkspace(m, road)

# Default values for time and weather
param time = (0 * 60, 24 * 60)	# 0000 to 2400 hours
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

# Define custom type of object for cars
constructor Car:
	position: Point on road
	heading: (roadDirection at self.position) + self.roadDeviation
	roadDeviation: 0
	width: self.model.width
	height: self.model.height
	viewAngle: 80 deg
	visibleDistance: 30
	model: CarModel.defaultModel()
	color: CarColor.defaultColor()

	mutator[additive]: CarColorMutator()

class CarColorMutator(Mutator):
	def appliedTo(self, obj):
		hueNoise = random.gauss(0, 0.05)
		satNoise = random.gauss(0, 0.05)
		lightNoise = random.gauss(0, 0.05)
		color = NoisyColorDistribution.addNoiseTo(obj.color, hueNoise, lightNoise, satNoise)
		return tuple([obj.copyWith(color=color), True])		# allow further mutation

# Convenience subclass with defaults for ego cars
constructor EgoCar(Car):
	model: CarModel.egoModel()

# Convenience subclass for buses
constructor Bus(Car):
	model: CarModel.models['BUS']

# Convenience subclass for compact cars
constructor Compact(Car):
	model: CarModel.models['BLISTA']

# Helper function for making platoons
def createPlatoonAt(car, numCars, model=None, dist=(2, 8), shift=(-0.5, 0.5), wiggle=0):
	cars = [car]
	lastCar = car
	for i in range(numCars-1):
		center = follow roadDirection from (front of lastCar) for resample(dist)
		pos = OrientedPoint right of center by shift, facing resample(wiggle) relative to roadDirection
		lastCar = Car ahead of pos, with model (car.model if model is None else resample(model))
		cars.append(lastCar)
	return cars
