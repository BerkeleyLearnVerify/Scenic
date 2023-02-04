param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on road,
		with visibleDistance 30

for i in range(2):
	spot = OrientedPoint on visible curb
	Car left of (spot offset by -0.25 @ 0)