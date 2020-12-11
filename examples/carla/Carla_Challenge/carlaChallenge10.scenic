""" Scenario Description
Traffic Scenario 10.
Crossing negotiation at an unsignalized intersection.
The ego-vehicle needs to negotiate with other vehicles to cross an unsignalized intersection. In
this situation it is assumed that the first to enter the intersection has priority.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_INTER_DIST = [15, 20]
ADV_INTER_DIST = [15, 20]
EGO_SPEED = 10
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

##DEFINING BEHAVIORS
behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory = trajectory)
    terminate

behavior EgoBehavior(speed, trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        take SetBrakeAction(BRAKE_INTENSITY)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*fourWayIntersection)
ego_start_lane = Uniform(*intersec.incomingLanes)

ego_maneuver = Uniform(*ego_start_lane.maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuver = Uniform(*ego_maneuver.conflictingManeuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]
adv_start_lane = adv_maneuver.startLane

## OBJECT PLACEMENT
ego_spawn_pt = OrientedPoint in ego_maneuver.startLane.centerline
adv_spawn_pt = OrientedPoint in adv_maneuver.startLane.centerline

ego = Car at ego_spawn_pt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, ego_trajectory)

adversary = Car at adv_spawn_pt,
    with behavior AdversaryBehavior(adv_trajectory)

require (distance from ego to intersec) > EGO_INTER_DIST[0] and (distance from ego to intersec) < EGO_INTER_DIST[1]
require (distance from adversary to intersec) > ADV_INTER_DIST[0] and (distance from adversary to intersec) < ADV_INTER_DIST[1]
