""" Scenario Description
Traffic Scenario 05.
Lane changing to evade slow leading vehicle.
The ego-vehicle performs a lane changing to evade a leading vehicle, which is moving too slowly.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town03'
model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

LEAD_CAR_SPEED = 3

DIST_THRESHOLD = 15
BYPASS_DIST = 25

## BEHAVIORS
behavior EgoBehavior(speed=10):
    try: 
        do FollowLaneBehavior(speed)

    interrupt when withinDistanceToAnyObjs(self, DIST_THRESHOLD):
        # change to left (overtaking)
        faster_lane = self.laneSection.fasterLane
        do LaneChangeBehavior(laneSectionToSwitch=faster_lane, target_speed=speed)
        do FollowLaneBehavior(speed, laneToFollow=faster_lane.lane) until (distance to lead) > BYPASS_DIST

        # change to right
        slower_lane = self.laneSection.slowerLane
        do LaneChangeBehavior(laneSectionToSwitch=slower_lane, target_speed=speed)
        do FollowLaneBehavior(speed) for 5 seconds
        terminate

behavior LeadingCarBehavior(speed=3):
    do FollowLaneBehavior(speed)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

lane = Uniform(*network.lanes)

ego = Car on lane.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

lead = Car following roadDirection for Range(10, 25),
    with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

require (distance to intersection) > 50
require (distance from lead to intersection) > 50
require always (lead.laneSection._fasterLane is not None)
