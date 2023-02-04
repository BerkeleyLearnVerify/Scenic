param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

spot = OrientedPoint on lane
ego = Car behind spot by Range(3,5),
		facing Range(-10,10) deg relative to roadDirection
Car right of spot by Range(0.5, 1),
		facing Range(10,20) deg relative to ego.heading,
		with regionContainedIn None

require (distance to intersectionRegion) > 30