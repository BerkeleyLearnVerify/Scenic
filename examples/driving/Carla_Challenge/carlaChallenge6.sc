"""
Ego-vehicle must go around a blocking object
using the opposite lane, yielding to oncoming traffic.
Based on CARLA Challenge Scenario 6:
https://carlachallenge.org/challenge/nhtsa/
"""

param map = localPath('../../carla/OpenDrive/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.domains.driving.model


#CONSTANTS
ONCOMING_THROTTLE = 0.6
EGO_SPEED = 10
ONCOMING_CAR_SPEED = 10
DIST_THRESHOLD = 13
YIELD_THRESHOLD = 5
BLOCKING_CAR_DIST = 24
BREAK_INTENSITY = 0.8

#EGO BEHAVIOR
behavior EgoBehavior(path):
	current_lane = network.laneAt(self)

	try:
		FollowLaneBehavior(EGO_SPEED, laneToFollow=current_lane)

	interrupt when (distance to blockingCar) < DIST_THRESHOLD or (distance to oncomingCar) < DIST_THRESHOLD:
		if ego can see oncomingCar:
			take SetBrakeAction(BREAK_INTENSITY)
		elif (distance to oncomingCar) > YIELD_THRESHOLD:
			LaneChangeBehavior(path, EGO_SPEED)
		else:
			wait


#OTHER BEHAVIORS
behavior OncomingCarBehavior(path = []):
	FollowLaneBehavior(ONCOMING_CAR_SPEED)

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
leftLane = initLaneSec.laneToLeft

spawnPt = OrientedPoint on initLaneSec.centerline

#PLACEMENT
oncomingCar = Car on leftLane.centerline,
	with behavior OncomingCarBehavior()

ego = Car at spawnPt,
	with behavior EgoBehavior(leftLane)

blockingCar = Car following roadDirection from ego for BLOCKING_CAR_DIST

#Make sure the oncoming Car is at a visible section of the lane
require blockingCar can see oncomingCar
require (distance from blockingCar to oncomingCar) < 10
require (distance from blockingCar to intersection) > 20