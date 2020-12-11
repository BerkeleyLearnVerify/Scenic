""" Scenario Description
Traffic Scenario 09.
Right turn at an intersection with crossing traffic.
The ego-vehicle is performing a right turn at an intersection, yielding to crossing traffic.
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_INTER_DIST = [30, 35]
ADV_INTER_DIST = [10, 15]
EGO_SPEED = 10
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

## MONITORS
monitor TrafficLights:
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "red")
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

spawnAreas = []

# The meaning of filter() function is explained in examples/carla/Carla_Challenge/carlaChallenge7.scenic
fourWayIntersection = filter(lambda i: i.is4Way and i.isSignalized, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*fourWayIntersection)
startLane = Uniform(*intersec.incomingLanes)
straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
straight_trajectory = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]

conflicting_rightTurn_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, straight_maneuver.conflictingManeuvers)
ego_rightTurn_maneuver = Uniform(*conflicting_rightTurn_maneuvers)
ego_startLane = ego_rightTurn_maneuver.startLane
ego_trajectory = [ego_rightTurn_maneuver.startLane, ego_rightTurn_maneuver.connectingLane, \
								ego_rightTurn_maneuver.endLane]


## OBJECT PLACEMENT
ego_spawn_pt = OrientedPoint in ego_maneuver.startLane.centerline
adv_spawn_pt = OrientedPoint in adv_maneuver.startLane.centerline

ego = Car at ego_spawn_pt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, ego_trajectory)

adversary = Car at adv_spawn_pt,
    with behavior AdversaryBehavior(adv_trajectory)

require (ego_maneuver.endLane == adv_maneuver.endLane)
require (distance from ego to intersec) > EGO_INTER_DIST[0] and (distance from ego to intersec) < EGO_INTER_DIST[1]
require (distance from adversary to intersec) > ADV_INTER_DIST[0] and (distance from adversary to intersec) < ADV_INTER_DIST[1]
