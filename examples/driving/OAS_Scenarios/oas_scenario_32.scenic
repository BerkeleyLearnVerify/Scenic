""" Scenario Description
Voyage OAS Scenario Unique ID: 3-2-W-I-L-CAR:N>S
At 3-way intersection, ego turns left and the other car on a different leg of the
intersection goes straight. There is no requirement on which vehicle has the right of the way.
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town10HD.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town10HD'
model scenic.domains.driving.model

#Constant
SAFE_DIST = 15

##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# The meaning of filter() function is explained in examples/carla/Carla_Challenge/carlaChallenge7.scenic
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list, 'threeWayIntersections'
intersection = Uniform(*threeWayIntersections)

# FIRST, DEFINE THE EGO'S STARTPOINT AND TRAJECTORY TO FOLLOW
straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane
centerlines = [startLane, connectingLane, endLane]

# SECOND, DEFINE THE OTHER CAR'S STARTPOINT AND TRAJECTORY TO FOLLOW
leftTurn_manuevers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, straight_maneuver.conflictingManeuvers)
leftTurn_maneuver = Uniform(*leftTurn_manuevers)
ego_L_startLane = leftTurn_maneuver.startLane
ego_L_connectingLane = leftTurn_maneuver.connectingLane
ego_L_endLane = leftTurn_maneuver.endLane
ego_L_centerlines = [ego_L_startLane, ego_L_connectingLane, ego_L_endLane]


##DEFINING BEHAVIORS
behavior FollowTrafficBehavior(target_speed, trajectory):
	do FollowTrajectoryBehavior(target_speed, trajectory)
	terminate

behavior SafeBehavior(thresholdDistance, target_speed=10, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
		terminate

	interrupt when withinDistanceToObjsInLane(vehicle=self, thresholdDistance=thresholdDistance):
		take SetBrakeAction(brakeIntensity)

# OBJECT PLACEMENT
ego = Car on ego_L_startLane.centerline,
		with behavior SafeBehavior(SAFE_DIST,target_speed=8, trajectory=ego_L_centerlines)

other = Car on startLane.centerline,
		with behavior FollowTrafficBehavior(target_speed=10, trajectory=centerlines)

