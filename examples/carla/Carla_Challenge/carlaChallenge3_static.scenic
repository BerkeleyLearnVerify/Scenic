""" Scenario Description
Traffic Scenario 03 (static).
Obstacle avoidance without prior action.
The ego-vehicle encounters an obstacle / unexpected entity on the road and must perform an
emergency brake or an avoidance maneuver.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10
EGO_BRAKING_THRESHOLD = 12

BRAKE_ACTION = 1.0

## DEFINING BEHAVIORS
# EGO BEHAVIOR: Follow lane, and brake after passing a threshold distance to the leading car
behavior EgoBehavior(speed=10):	
    try: 
        do FollowLaneBehavior(speed)

    interrupt when withinDistanceToAnyObjs(self, EGO_BRAKING_THRESHOLD):
        take SetBrakeAction(BRAKE_ACTION)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# make sure to put '*' to uniformly randomly select from all elements of the list, 'lanes'
lane = Uniform(*network.lanes)

spawnPt = OrientedPoint on lane.centerline

obstacle = Trash at spawnPt offset by Range(1, -1) @ 0

ego = Car following roadDirection from spawnPt for Range(-50, -30),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

require (distance from ego to intersection) > 75
terminate when ego.speed < 0.1 and (distance to obstacle) < 15
