""" Scenario Description
Voyage OAS Scenario Unique ID: 3-2-NSW-I-L-CAR:S>W:02
At 3 way intersection. The ego car turns left. 
The other car, on a different leg of the intersection, 
has the right of the way and makes a left turn first because it is closer to the intersection.
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/LGSVL/cubetown.xodr')  # or other CARLA map that definitely works
param lgsvl_map = 'CubeTown'
model scenic.domains.driving.model

# Constants
EGO_OFFSET = -1 * Range(15,20)
OTHERCAR_OFFSET = -1* Range(1,3)

##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# The meaning of filter() function is explained in examples/carla/Carla_Challenge/carlaChallenge7.scenic
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list, 'threeWayIntersections'
intersection = Uniform(*threeWayIntersections)

# FIRST, DEFINE THE EGO'S STARTPOINT AND TRAJECTORY TO FOLLOW
left_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers)
ego_maneuver = Uniform(*left_maneuvers)
ego_L_centerlines = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
egoStart = OrientedPoint at ego_maneuver.startLane.centerline[-1]

# SECOND, DEFINE THE OTHER CAR'S STARTPOINT AND TRAJECTORY TO FOLLOW
conflicting_left = filter(lambda m: m.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
actor_maneuver = Uniform(*conflicting_left)
actor_centerlines = [actor_maneuver.startLane, actor_maneuver.connectingLane, actor_maneuver.endLane]
actorStart = actor_maneuver.startLane.centerline[-1]


# BEHAVIOR
behavior EgoBehavior(thresholdDistance, target_speed=10, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
		terminate

	interrupt when withinDistanceToAnyObjs(vehicle=self, thresholdDistance=thresholdDistance):
		take SetBrakeAction(brakeIntensity)


# PLACEMENT
ego = Car following roadDirection from egoStart for EGO_OFFSET,
		with behavior EgoBehavior(target_speed=10, trajectory=ego_L_centerlines, thresholdDistance = 20)

other = Car following roadDirection from actorStart for OTHERCAR_OFFSET,
		with behavior FollowTrajectoryBehavior(target_speed=10, trajectory=actor_centerlines)
