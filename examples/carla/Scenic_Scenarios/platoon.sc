from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town10HD.xodr')

param map = localPath('../OpenDrive/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.domains.driving.model


# import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

# from scenic.simulators.domains.driving.network import loadNetwork
# loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/CARLA_0.9.9/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr')

# from scenic.simulators.carla.model import *
# from scenic.simulators.carla.behaviors import *


"""
Dynamic version of platoon scenario from https://arxiv.org/pdf/1809.09310.pdf
"""

TRAFFIC_SPEED = 10
EGO_SPEED = 8
DISTANCE_THRESHOLD = 4
BRAKE_ACTION = (0.8, 1.0)

behavior FollowTrafficBehavior(speed):
	print("position: ", ego.position)
	brake_intensity = resample(BRAKE_ACTION)
	try:
		FollowLaneBehavior(speed)

	interrupt when distanceToAnyObjs(self, DISTANCE_THRESHOLD):
		take SetBrakeAction(brake_intensity)

def createPlatoonAt(car, numCars, model=None, dist=(2, 8), shift=(-0.5, 0.5), wiggle=0):
	lastCar = car
	for i in range(numCars-1):
		center = follow roadDirection from (front of lastCar) for resample(dist)
		pos = OrientedPoint right of center by shift, facing resample(wiggle) relative to roadDirection
		lastCar = Car ahead of pos, with behavior FollowTrafficBehavior(TRAFFIC_SPEED)


param time = (8,20) * 60
roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

ego = Car on select_lane.centerline,
		 with visibleDistance 60,
		 with behavior FollowTrafficBehavior(EGO_SPEED)
platoon = createPlatoonAt(ego, 10, dist=(2,5))

