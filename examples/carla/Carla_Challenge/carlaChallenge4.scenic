""" Scenario Description
Traffic Scenario 04.
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
THROTTLE_ACTION = 1.0
BRAKE_ACTION = 1.0
SAFETY_DISTANCE = 10

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory = trajectory)

    interrupt when withinDistanceToAnyCars(self, SAFETY_DISTANCE):
        take SetBrakeAction(BRAKE_ACTION)

behavior BicycleBehavior(throttle):
    while (distance from ego to self) > 15:
        wait
    take SetThrottleAction(throttle)

## GEOMETRY

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*network.intersections)
startLane = Uniform(*intersec.incomingLanes)
maneuver = Uniform(*startLane.maneuvers)
ego_trajectory = [maneuver.startLane, maneuver.connectingLane, maneuver.endLane]

ego = Car in maneuver.startLane.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(trajectory = ego_trajectory)

spotBicycle = OrientedPoint in maneuver.endLane.centerline,
    facing roadDirection
bicycle = Bicycle at spotBicycle offset by 3.5@0,
    with heading 90 deg relative to spotBicycle.heading,
    with behavior BicycleBehavior(THROTTLE_ACTION),
    with regionContainedIn None

require (distance from ego to intersec) < 25 and (distance from ego to intersec) > 10
require (distance from bicycle to intersec) < 10 and (distance from bicycle to intersec) > 5
