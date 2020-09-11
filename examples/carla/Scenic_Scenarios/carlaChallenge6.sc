"""
Ego-vehicle must go around a blocking object
using the opposite lane, yielding to oncoming traffic.
Based on CARLA Challenge Scenario 6:
https://carlachallenge.org/challenge/nhtsa/
"""

param map = localPath('../OpenDrive/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.domains.driving.model


#CONSTANTS
ONCOMING_THROTTLE = 0.6
EGO_SPEED = 7
ONCOMING_CAR_SPEED = 10
DIST_THRESHOLD = 12
YIELD_THRESHOLD = 5
BLOCKING_CAR_DIST = Range(10, 20)
BREAK_INTENSITY = 0.8
BYPASS_DIST = 5
DIST_BTW_BLOCKING_ONCOMING_CARS = 10
DIST_TO_INTERSECTION = 10

#EGO BEHAVIOR
behavior EgoBehavior(path):
	current_lane = network.laneAt(self)
	laneChangeCompleted = False

	try:
		do FollowLaneBehavior(EGO_SPEED, laneToFollow=current_lane)

	interrupt when (distance to blockingCar) < DIST_THRESHOLD and not laneChangeCompleted:
		if ego can see oncomingCar:
			take SetBrakeAction(BREAK_INTENSITY)
		elif (distance to oncomingCar) > YIELD_THRESHOLD:
			do LaneChangeBehavior(path, is_oppositeTraffic=True, target_speed=EGO_SPEED)
			laneChangeCompleted = True
		else:
			wait

	interrupt when (blockingCar can see ego) and (distance from blockingCar to ego) < BYPASS_DIST:
		current_laneSection = network.laneSectionAt(self)
		rightLaneSec = current_laneSection._laneToLeft
		do LaneChangeBehavior(rightLaneSec, is_oppositeTraffic=False, target_speed=EGO_SPEED)


#OTHER BEHAVIORS
behavior OncomingCarBehavior(path = []):
	do FollowLaneBehavior(ONCOMING_CAR_SPEED)

#GEOMETRY

#Find lanes that have a lane to their left in the opposite direction
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec._laneToLeft is not None:
			if laneSec._laneToLeft.isForward is not laneSec.isForward:
				laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane with opposing \
	traffic direction in network.'

initLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = initLaneSec._laneToLeft

spawnPt = OrientedPoint on initLaneSec.centerline

#PLACEMENT
oncomingCar = Car on leftLaneSec.centerline,
	with behavior OncomingCarBehavior()

ego = Car at spawnPt,
	with behavior EgoBehavior(leftLaneSec),
	with blueprint 'vehicle.toyota.prius'

blockingCar = Car following roadDirection from ego for BLOCKING_CAR_DIST,
				with viewAngle 90 deg

#Make sure the oncoming Car is at a visible section of the lane
require blockingCar can see oncomingCar
require (distance from blockingCar to oncomingCar) < DIST_BTW_BLOCKING_ONCOMING_CARS
require (distance from blockingCar to intersection) > DIST_TO_INTERSECTION