""" Scenario Description
Traffic Scenario 05.
Lane changing to evade slow leading vehicle.
The ego-vehicle performs a lane changing to evade a leading vehicle,
which is moving too slowly.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

LEAD_CAR_SPEED = 3

DIST_THRESHOLD = 15
BYPASS_DIST = 50


## BEHAVIORS
behavior EgoBehavior(leftpath, speed=10):
    laneChangeCompleted = False

    try: 
        # do FollowLaneBehavior(speed, laneToFollow=current_lane)
        do FollowLaneBehavior(speed, laneToFollow=network.laneAt(self))

    interrupt when withinDistanceToAnyObjs(self, DIST_THRESHOLD) and not laneChangeCompleted:
        do LaneChangeBehavior(laneSectionToSwitch=leftpath, target_speed=speed)
        do FollowLaneBehavior(speed, laneToFollow=leftpath) until (distance to lead) > BYPASS_DIST
        laneChangeCompleted = True

behavior LeadingCarBehavior(speed=3):
    do FollowLaneBehavior(speed)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

laneSecsWithLeftLane = filter(lambda s: s._laneToLeft is not None, network.laneSections)
assert len(laneSecsWithLeftLane) > 0, \
    'No lane sections with adjacent left lane in network.'

# make sure to put '*' to uniformly randomly select from all elements of the list
laneSection = Uniform(*laneSecsWithLeftLane)

ego = Car on laneSection.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(laneSection._laneToLeft, EGO_SPEED)

lead = Car following roadDirection from ego for Range(10, 25),
    with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

require (distance from ego to intersection) > 100
require (distance from lead to intersection) > 100
require (laneSection._laneToLeft is not None)
