# bypass with a car on the right lane
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town10HD.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

# select a two lane road
two_lane_roads = filter(lambda i: len(i.lanes) == 2, network.roads)
two_lane_road = Uniform(*two_lane_roads)

offset = Uniform(-1,1) * Range(150, 180) deg

ego = Car on Uniform(*two_lane_road.lanes),
		facing offset relative to roadDirection,
		with visibleDistance 20,
		with viewAngle 135 deg

otherCar = Car on visible ego.laneSection._laneToLeft,
			facing Range(-15, 15) deg relative to roadDirection