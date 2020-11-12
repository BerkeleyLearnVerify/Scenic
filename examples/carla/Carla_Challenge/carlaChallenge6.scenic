""" Scenario Description
Traffic scenario 06.
Vehicle passing dealing with oncoming traffic.
The ego-vehicle must go around a blocking object using the opposite lane, yielding to oncoming traffic.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

## CONSTANTS
EGO_SPEED = 7
EGO_MODEL = "vehicle.lincoln.mkz2017"
BREAK_INTENSITY = 0.8
ONCOMING_CAR_SPEED = 10
DIST_THRESHOLD = 13
YIELD_THRESHOLD = 5
BLOCKING_CAR_DIST = Range(15, 20)
BYPASS_DIST = 5
DIST_BTW_BLOCKING_ONCOMING_CARS = 10
DIST_TO_INTERSECTION = 15

# BEHAVIORS
behavior EgoBehavior(path):
    current_lane = network.laneAt(self)
    laneChangeCompleted = False
    bypassed = False

    try:
        do FollowLaneBehavior(EGO_SPEED, laneToFollow=current_lane)

    interrupt when (distance to blockingCar) < DIST_THRESHOLD and not laneChangeCompleted:
        if ego can see oncomingCar:
            take SetBrakeAction(BREAK_INTENSITY)
        elif (distance to oncomingCar) > YIELD_THRESHOLD:
            do LaneChangeBehavior(path, is_oppositeTraffic=True, target_speed=EGO_SPEED)
            do FollowLaneBehavior(EGO_SPEED, is_oppositeTraffic=True) until (distance to blockingCar) > BYPASS_DIST
            laneChangeCompleted = True
        else:
            wait

    interrupt when (blockingCar can see ego) and (distance to blockingCar) > BYPASS_DIST and not bypassed:
        current_laneSection = network.laneSectionAt(self)
        rightLaneSec = current_laneSection._laneToLeft
        do LaneChangeBehavior(rightLaneSec, is_oppositeTraffic=False, target_speed=EGO_SPEED)
        bypassed = True


behavior OncomingCarBehavior(path = []):
    do FollowLaneBehavior(ONCOMING_CAR_SPEED)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# Find lanes that have a lane to their left in the opposite direction
laneSecsWithLeftLane = filter(lambda s: s._laneToLeft is not None and s._laneToLeft.isForward != s.isForward, network.laneSections)

assert len(laneSecsWithLeftLane) > 0, \
    'No lane sections with adjacent left lane with opposing \
    traffic direction in network.'

# make sure to put '*' to uniformly randomly select from all elements of the list
laneSection = Uniform(*laneSecsWithLeftLane)

## OBJECT PLACEMENT
oncomingCar = Car on laneSection._laneToLeft.centerline,
    with behavior OncomingCarBehavior()

ego = Car on laneSection.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(laneSection._laneToLeft)
	
blockingCar = Car following roadDirection from ego for BLOCKING_CAR_DIST,
    with viewAngle 90 deg

## EXPLICIT HARD CONSTRAINTS
require blockingCar can see oncomingCar
require (distance from blockingCar to oncomingCar) < DIST_BTW_BLOCKING_ONCOMING_CARS
require (distance from blockingCar to intersection) > DIST_TO_INTERSECTION
