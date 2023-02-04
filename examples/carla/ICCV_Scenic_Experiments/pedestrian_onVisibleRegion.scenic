param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on intersection,
		facing Range(-10, 10) deg relative to roadDirection,
		with visibleDistance 30

def placePedestrianOnVisibleRoad(numObj):
	for i in range(numObj):
		Pedestrian on visible anyArea, 
			facing Uniform(-1,1)*Range(80, 100) deg relative to ego.heading, 
			with regionContainedIn None

placePedestrianOnVisibleRoad(15)