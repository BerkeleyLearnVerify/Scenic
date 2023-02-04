param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

def placeObjs(car, numCars):
	for i in range(numCars):
		car = Pedestrian offset along 0 deg by 0 @ Normal(3,1)
		Car beyond car by Range(3,5) @ 0
		Pedestrian behind car by Range(0,1),
			facing (angle to car)

spawn_point = 207.26 @ 8.72
ego = Car at spawn_point
car = Car offset by 0 @ Range(-6,-5)

placeObjs(ego, 1)