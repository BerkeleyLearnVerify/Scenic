""" Scenario Description
Traffic Scenario 06.
Vehicle passing dealing with oncoming traffic.
The ego-vehicle must go around a blocking object using the opposite lane, yielding to oncoming
traffic
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 7
ONCOMING_CAR_SPEED = 10
BLOCKING_CAR_DIST = Range(15, 20)
BREAK_INTENSITY = 0.8
DIST_THRESHOLD = 15

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
            do FollowLaneBehavior(EGO_SPEED, is_oppositeTraffic=True) until (distance to blockingCar) > DIST_THRESHOLD

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
lanes_with_left_lane = filter(lambda s: s._laneToLeft is not None, network.laneSections)

assert len(lanes_with_left_lane) > 0, \
    'No lane sections with adjacent left lane in network.'

ego_lane_sec = Uniform(*lanes_with_left_lane)
opp_lane_sec = ego_lane_sec._laneToLeft

## OBJECT PLACEMENT
oncomingCar = Car on opp_lane_sec.centerline,
    with behavior OncomingCarBehavior()

ego = Car on ego_lane_sec.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

blockingCar = Car following roadDirection for BLOCKING_CAR_DIST,
    with viewAngle 90 deg

## EXPLICIT HARD CONSTRAINTS
require blockingCar can see oncomingCar
require (distance from blockingCar to oncomingCar) in Range(5, 15)
require (distance from blockingCar to intersection) > 50
require (ego.laneSection.isForward != oncomingCar.laneSection.isForward)