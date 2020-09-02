""" Scenario Description
At 3-way intersection, ego turns left and the other car goes straight
"""

param map = localPath('../../carla/OpenDrive/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.domains.driving.model

threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane
centerlines = [startLane, connectingLane, endLane]


leftTurn_manuevers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, straight_maneuver.conflictingManeuvers)
leftTurn_maneuver = Uniform(*leftTurn_manuevers)
ego_L_startLane = leftTurn_maneuver.startLane
ego_L_connectingLane = leftTurn_maneuver.connectingLane
ego_L_endLane = leftTurn_maneuver.endLane
ego_L_centerlines = [ego_L_startLane, ego_L_connectingLane, ego_L_endLane]


# PLACEMENT
ego = Car on ego_L_startLane.centerline,
		with behavior FollowTrajectoryBehavior(target_speed=8, trajectory=ego_L_centerlines)

other = Car on startLane.centerline,
		with behavior FollowTrajectoryBehavior(target_speed=10, trajectory=centerlines)