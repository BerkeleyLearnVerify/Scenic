""" Scenario Description
Based on 2019 Carla Challenge Traffic Scenario 09.
Ego-vehicle enters an unsignalized intersection and has to adapt to the flow of traffic
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

#CONSTANTS
EGO_DISTANCE_TO_INTERSECTION = Uniform(15, 20) * -1
ADV_DISTANCE_TO_INTERSECTION = Uniform(10, 15) * -1
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

##DEFINING BEHAVIORS
behavior CrossingCarBehavior(trajectory):
	do FollowTrajectoryBehavior(trajectory = trajectory)
	terminate

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(BRAKE_INTENSITY)

##DEFINING SPATIAL RELATIONS
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
# Use the -1' index to get the last endpoint from the list of centerpoints in 'centerline'
ego_spawn_point = ego_start_lane.centerline[-1]
adv_spawn_point = adv_start_lane.centerline[-1]

# Set a specific vehicle model for the Truck.
# The referenceable types of vehicles supported in carla are listed in scenic/simulators/carla/model.scenic
# For each vehicle type, the supported models are listed in scenic/simulators/carla/blueprints.scenic
ego = Car following roadDirection from ego_spawn_point for EGO_DISTANCE_TO_INTERSECTION,
	with behavior EgoBehavior(ego_trajectory)

adversary = Car following roadDirection from adv_spawn_point for ADV_DISTANCE_TO_INTERSECTION,
	with behavior CrossingCarBehavior(adv_trajectory)

