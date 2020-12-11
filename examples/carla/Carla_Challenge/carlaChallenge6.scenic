""" Scenario Description
Traffic Scenario 06.
Vehicle passing dealing with oncoming traffic.
The ego-vehicle must go around a blocking object using the opposite lane, yielding to oncoming
traffic
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 7
ONCOMING_CAR_SPEED = 10
DIST_THRESHOLD = 13
YIELD_THRESHOLD = 5
BLOCKING_CAR_DIST = Range(15, 20)
BREAK_INTENSITY = 0.8
BYPASS_DIST = 5
DIST_BTW_BLOCKING_ONCOMING_CARS = 10
DIST_TO_INTERSECTION = 15

## DEFINING BEHAVIORS
behavior EgoBehavior():
    current_lane = ego.lane
    current_lane_sec = ego.laneSection
    left_lane_sec = ego.laneSection._laneToLeft

    try:
        do FollowLaneBehavior(EGO_SPEED, laneToFollow=current_lane)

    interrupt when (distance to blockingCar) < DIST_THRESHOLD:

        try:
            do LaneChangeBehavior(left_lane_sec, is_oppositeTraffic=True, target_speed=EGO_SPEED)
            do FollowLaneBehavior(EGO_SPEED, is_oppositeTraffic=True) until (distance from ego to blockingCar) > DIST_THRESHOLD

        interrupt when ego can see oncomingCar:
            take SetBrakeAction(BREAK_INTENSITY)

        do LaneChangeBehavior(current_lane_sec, is_oppositeTraffic=False, target_speed=EGO_SPEED)
        do FollowLaneBehavior(EGO_SPEED, is_oppositeTraffic=False) for 3 seconds
        terminate

behavior OncomingCarBehavior():
    do FollowLaneBehavior(ONCOMING_CAR_SPEED)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# Find lanes that have a lane to their left in the opposite direction
laneSecsWithLeftLane = []

for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec._laneToLeft is not None:
			if laneSec._laneToLeft.isForward is not laneSec.isForward:
				laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane with opposing \
	traffic direction in network.'

## OBJECT PLACEMENT
oncomingCar = Car on opp_lane_sec.centerline,
    with behavior OncomingCarBehavior()

ego = Car on ego_lane_sec.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

ego = Car at spawnPt,
	with behavior EgoBehavior(leftLaneSec)
	
blockingCar = Car following roadDirection from ego for BLOCKING_CAR_DIST,
				with viewAngle 90 deg

## EXPLICIT HARD CONSTRAINTS
require blockingCar can see oncomingCar
require (distance from blockingCar to oncomingCar) < 10
require (distance from blockingCar to intersection) > 10
require (ego.laneSection.isForward != oncomingCar.laneSection.isForward)
