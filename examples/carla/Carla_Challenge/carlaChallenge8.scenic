""" Scenario Description
Based on 2019 Carla Challenge Traffic Scenario 08.
Unprotected left turn at intersection with oncoming traffic.
The ego-vehicle is performing an unprotected left turn at an intersection,
yielding to oncoming traffic."""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

# CONSTANTS
DELAY_TIME_1 = 1 # the delay time for ego
DELAY_TIME_2 = 40 # the delay time for the slow car
FOLLOWING_DISTANCE = 13 # normally 10, 40 when DELAY_TIME is 25, 50 to prevent collisions
DISTANCE_TO_INTERSECTION1 = Uniform(10, 15) * -1
DISTANCE_TO_INTERSECTION2 = Uniform(15, 20) * -1
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

## DEFINING BEHAVIORS
behavior CrossingCarBehavior(trajectory):
	do FollowTrajectoryBehavior(trajectory = trajectory)
	terminate

behavior EgoBehavior(trajectory):
	try :
		do FollowTrajectoryBehavior(trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(BRAKE_INTENSITY)


## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# The meaning of filter() function is explained in examples/carla/Carla_Challenge/carlaChallenge7.scenic
fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)

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
# Use the -1' index to get the last endpoint from the list of centerpoints in 'centerline'
ego_spawn_pt = ego_start_lane.centerline[-1] 
adv_spawn_pt = adv_start_lane.centerline[-1]

ego = Car following roadDirection from ego_spawn_pt for DISTANCE_TO_INTERSECTION2,
	with behavior EgoBehavior(ego_trajectory),
	with blueprint 'vehicle.tesla.model3'

crossing_car = Car following roadDirection from adv_spawn_pt for DISTANCE_TO_INTERSECTION1,
	with behavior CrossingCarBehavior(adv_trajectory)

require (ego_start_section.laneToLeft == adv_end_section)  # make sure the ego and adversary are spawned in opposite lanes