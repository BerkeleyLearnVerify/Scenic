model scenic.domains.driving.model

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

	interrupt when ((distance to blockingCar) < DIST_THRESHOLD):
		#Switch Lanes
		while True:
			#Wait until oncoming Car passes
			if ((distance to oncomingCar) < YIELD_THRESHOLD):
				take SetBrakeAction(1.0)
			else:
				FollowTrajectoryBehavior(EGO_SPEED,path)

#OTHER BEHAVIORS
behavior OncomingCarBehavior(path = []):
	FollowLaneBehavior(ONCOMING_CAR_SPEED,network)

#GEOMETRY

#Find lanes that have a lane to their left in the opposite direction
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

#Make sure the oncoming Car is at a visible section of the lane
require blockingCar can see oncomingCar