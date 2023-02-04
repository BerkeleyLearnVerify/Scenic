param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on lane, 
		facing Range(-10, 10) deg relative to roadDirection,
		with viewAngle 135 deg,
		with visibleDistance 30
car = Car ahead of ego by Range(3, 5),
		facing roadDirection
truck = Truck visible from car,
		facing Range(-20,0) deg relative to roadDirection
mc = Motorcycle left of car by Range(2,5),
		facing roadDirection

require ego can see mc
require (distance from truck to car) < 30