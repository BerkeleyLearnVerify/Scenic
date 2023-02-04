param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

def placeObjs(car, numCars):
	for i in range(numCars):
		car = Car ahead of car by Range(4, 5)
		leftCar = Car left of car by Normal(2,0.1),
			facing roadDirection
		rightCar = Bicycle right of car by Normal(3,0.1),
			facing Range(0,10) deg relative to ego.heading
	return leftCar, rightCar

spawn_point = 207.26 @ 8.72
ego = Car at spawn_point
leftCar, rightCar = placeObjs(ego, 2)

require (distance to leftCar) < 50
require (distance to rightCar) < 50