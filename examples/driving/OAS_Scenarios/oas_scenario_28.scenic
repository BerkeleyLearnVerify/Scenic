""" Scenario Description
Voyage OAS Scenario Unique ID: 3-2-ESW-I-STR-CAR:S>W:02
At three-way intersection. The ego vehicle goes straight. 
The other car, on the other leg of the intersection, takes a left turn first 
because it is closer to the intersection.
"""

param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.domains.driving.model

param time_step = 1.0/10

# Constants
EGO_OFFSET = -1 *Range(15,20)
OTHERCAR_OFFSET = -1* Range(1,3)

# GEOMETRY
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)

startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane

lane_traj = [startLane, connectingLane, endLane]
intersection_edge = startLane.centerline[-1]
egoStartPoint = OrientedPoint at intersection_edge

# --

conflicting_lefts = filter(lambda m: m.type == ManeuverType.LEFT_TURN, straight_maneuver.conflictingManeuvers)
leftTurn_maneuver = Uniform(*conflicting_lefts)

L_startLane = leftTurn_maneuver.startLane
L_connectingLane = leftTurn_maneuver.connectingLane
L_endLane = leftTurn_maneuver.endLane

L_centerlines = [L_startLane, L_connectingLane, L_endLane]

L_intersection_edge = L_startLane.centerline[-1]
actorStartPoint = OrientedPoint at L_intersection_edge

# BEHAVIOR
behavior EgoBehavior(target_speed=10, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)
		terminate

	interrupt when withinDistanceToAnyCars(car=self, thresholdDistance=15):
		take SetBrakeAction(brakeIntensity)

behavior OtherCarBehavior(trajectory, target_speed=10):
	brakeIntensity = 0.8
	do FollowTrajectoryBehavior(target_speed=8, trajectory=trajectory)
	take SetBrakeAction(brakeIntensity)


# PLACEMENT
ego = Car following roadDirection from egoStartPoint for EGO_OFFSET,
		with behavior EgoBehavior(target_speed=8, trajectory=lane_traj)

other = Car following roadDirection from actorStartPoint for OTHERCAR_OFFSET,
		with behavior FollowTrajectoryBehavior(target_speed=8, trajectory=L_centerlines)
