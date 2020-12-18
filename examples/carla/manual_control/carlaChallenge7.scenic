""" Scenario Description
Traffic Scenario 07.
Crossing traffic running a red light at an intersection.
The ego-vehicle is going straight at an intersection but a crossing vehicle runs a red light,
forcing the ego-vehicle to avoid the collision.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

## CONSTANTS
EGO_INTER_DIST = [15, 20]
ADV_INTER_DIST = [10, 15]
EGO_SPEED = 10
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

## MONITORS
monitor TrafficLights:
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "red")
        wait

## DEFINING BEHAVIORS
behavior AdversaryBehavior(trajectory):
    while (ego.speed < 0.1):
        wait
    do FollowTrajectoryBehavior(trajectory = trajectory)
    terminate

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py
spawnAreas = []

"""The filter() is Scenic's built-in function equivalent to the following for-loop
fourWayIntersection = []
for i in network.intersections:
    if i.is4Way:
        fourWayIntersection.append(i)
"""
fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*fourWayIntersection)
ego_startLane = Uniform(*intersec.incomingLanes)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_startLane.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## OBJECT PLACEMENT
ego_spawn_pt = OrientedPoint in ego_maneuver.startLane.centerline
adv_spawn_pt = OrientedPoint in adv_maneuver.startLane.centerline

# Set a specific vehicle model for the Truck. 
# The referenceable types of vehicles supported in carla are listed in scenic/simulators/carla/model.scenic
# For each vehicle type, the supported models are listed in scenic/simulators/carla/blueprints.scenic
ego = Car at ego_spawn_pt,
    with rolename "hero"

adversary = Car at adv_spawn_pt,
    with behavior AdversaryBehavior(adv_trajectory)

require (distance from ego to intersec) > EGO_INTER_DIST[0] and (distance from ego to intersec) < EGO_INTER_DIST[1]
require (distance from adversary to intersec) > ADV_INTER_DIST[0] and (distance from adversary to intersec) < ADV_INTER_DIST[1]
