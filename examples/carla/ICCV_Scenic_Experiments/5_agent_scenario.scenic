param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on lane, 
		facing Range(-10, 10) deg relative to roadDirection,
		with viewAngle 135 deg, 
		with visibleDistance 30
bicycle = Bicycle ahead of ego by Range(3,5),
			facing Range(-10,10) deg relative to roadDirection
truck = Truck behind ego by Range(4, 10),
			facing roadDirection
mc = Motorcycle left of ego by Range(2,4),
		facing toward bicycle
car = Car right of bicycle by Normal(2, 1),
		with regionContainedIn roadRegion,
		facing roadDirection

require abs(relative heading of ego from mc) < 30 deg