param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model

param weather = 'ClearNoon'

# select a two lane road
two_lane_roads = filter(lambda i: len(i.lanes) == 2, network.roads)
two_lane_road = Uniform(*two_lane_roads)

ego = Car on Uniform(*two_lane_road.lanes),
		facing Range(-15,15) deg relative to roadDirection,
		with visibleDistance 50,
		with viewAngle 135 deg

otherCar = Car on visible ego.laneSection._laneToLeft,
			facing Range(-15, 15) deg relative to roadDirection