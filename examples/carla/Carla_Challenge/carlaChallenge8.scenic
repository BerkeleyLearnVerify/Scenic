""" Scenario Description
Traffic Scenario 08.
Unprotected left turn at intersection with oncoming traffic.
The ego-vehicle is performing an unprotected left turn at an intersection, yielding to oncoming
traffic.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_INTER_DIST = [25, 30]
ADV_INTER_DIST = [15, 20]
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
            setClosestTrafficLightStatus(adversary, "green")
        wait

## DEFINING BEHAVIORS
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

# The meaning of filter() function is explained in examples/carla/Carla_Challenge/carlaChallenge7.scenic
fourWayIntersection = filter(lambda i: i.is4Way and i.isSignalized, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*fourWayIntersection)
ego_start_lane = Uniform(*intersec.incomingLanes)

# Get the ego manuever
ego_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, ego_start_lane.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

ego_start_section = ego_maneuver.startLane.sections[-1]

# Get the adversary maneuver
adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

adv_start_lane = adv_maneuver.startLane
adv_end_section = adv_maneuver.endLane.sections[0]

## OBJECT PLACEMENT
ego_spawn_pt = OrientedPoint in ego_maneuver.startLane.centerline
adv_spawn_pt = OrientedPoint in adv_maneuver.startLane.centerline

ego = Car at ego_spawn_pt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, ego_trajectory)

adversary = Car at adv_spawn_pt,
    with behavior AdversaryBehavior(adv_trajectory)

require (ego_start_section.laneToLeft == adv_end_section)  # make sure the ego and adversary are spawned in opposite lanes
require (distance from ego to intersec) > EGO_INTER_DIST[0] and (distance from ego to intersec) < EGO_INTER_DIST[1]
require (distance from adversary to intersec) > ADV_INTER_DIST[0] and (distance from adversary to intersec) < ADV_INTER_DIST[1]
