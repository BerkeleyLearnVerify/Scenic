model scenic.domains.driving.model

"""
Ego-vehicle is going straight at an intersection but a crossing vehicle 
runs a red light, forcing the ego-vehicle to perform a collision avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 07.
"""

DELAY_TIME_1 = 1 # the delay time for ego
DELAY_TIME_2 = 40 # the delay time for the slow car
FOLLOWING_DISTANCE = 13 # normally 10, 40 when DELAY_TIME is 25, 50 to prevent collisions

DISTANCE_TO_INTERSECTION1 = Uniform(15, 20) * -1
DISTANCE_TO_INTERSECTION2 = Uniform(10, 15) * -1
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0


behavior CrossingCarBehavior(trajectory):
	while True:
		FollowTrajectoryBehavior(trajectory = trajectory)

behavior EgoBehavior(trajectory):
	
	try:
		FollowTrajectoryBehavior(trajectory=trajectory)
	interrupt when distanceToAnyObjs(self, SAFETY_DISTANCE):
		terminate


spawnAreas = []
fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersection)

startLane = Uniform(*intersec.incomingLanes)
straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
ego_trajectory = [straight_maneuver.startLane.centerline, straight_maneuver.connectingLane.centerline, straight_maneuver.endLane.centerline]

conflicting_straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, straight_maneuver.conflictingManeuvers)
csm = Uniform(*conflicting_straight_maneuvers)
crossing_startLane = csm.startLane
crossing_car_trajectory = [csm.startLane.centerline, csm.connectingLane.centerline, csm.endLane.centerline]

ego_spwPt = startLane.centerline[-1]
csm_spwPt = crossing_startLane.centerline[-1]

ego = Car following roadDirection from ego_spwPt by DISTANCE_TO_INTERSECTION1,
		with behavior EgoBehavior(trajectory = ego_trajectory)

crossing_car = Car following roadDirection from csm_spwPt by DISTANCE_TO_INTERSECTION2,
				with behavior CrossingCarBehavior(crossing_car_trajectory)