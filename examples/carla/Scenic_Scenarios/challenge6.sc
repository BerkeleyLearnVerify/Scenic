from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

param map = localPath('../OpenDrive/Town03.xodr')
param carla_map = 'Town03'

from scenic.domains.driving.behaviors import *
# from scenic.simulators.carla.model import *

model scenic.domains.driving.model
import scenic.simulators.carla.actions as actions

"""
Ego-vehicle must go around a blocking object
using the opposite lane, yielding to oncoming traffic.
Based on CARLA Challenge Scenario 6:
https://carlachallenge.org/challenge/nhtsa/
"""

#CONSTANTS
ONCOMING_THROTTLE = 0.6
EGO_SPEED = 10
ONCOMING_CAR_SPEED = 10
DIST_THRESHOLD = 18
YIELD_THRESHOLD = 10
BLOCKING_CAR_DIST = 24

#EGO BEHAVIOR
behavior EgoBehavior(path=[]):
	try:
		FollowLaneBehavior(EGO_SPEED,network)
		print(distance to blockingCar)

	interrupt when ((distance to blockingCar) < DIST_THRESHOLD):
		print('THRESHOLD PASSED: CHANGING LANES')
		#Switch Lanes
		while True:
			if ((distance to oncomingCar) < YIELD_THRESHOLD):
				take actions.SetBrakeAction(1.0)
			else:
				FollowTrajectoryBehavior(EGO_SPEED,path)

#OTHER BEHAVIORS
behavior OncomingCarBehavior(path = []):
	FollowLaneBehavior(ONCOMING_CAR_SPEED,network)


#GEOMETRY
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			if laneSec.laneToLeft.isForward is not laneSec.isForward:
				laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane with opposing \
	traffic direction in network.'

initLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = initLaneSec.laneToLeft

spawnPt = OrientedPoint on initLaneSec.centerline
leftPt = Uniform(*leftLaneSec.centerline)

#PLACEMENT
oncomingCar = Car on leftLaneSec.centerline,
	with behavior OncomingCarBehavior()

ego = Car at spawnPt,
	with behavior EgoBehavior([initLaneSec.centerline])

blockingCar = Car following roadDirection from ego by BLOCKING_CAR_DIST

require blockingCar can see oncomingCar