param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

def placeObjs(numPeds):
	for i in range(numPeds):
		Pedestrian offset by Range(-5,5) @ Range(0,100),
			facing Range(-120, 120) deg relative to ego.heading

spawn_point = 207.26 @ 8.72
ego = Car at spawn_point,
		with visibleDistance 100

placeObjs(6)