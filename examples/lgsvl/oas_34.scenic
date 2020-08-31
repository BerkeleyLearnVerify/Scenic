# 3 way intersection. ego going straight. ego has right of way, but actor blocking it. 

param map = localPath('maps/cubetown.xodr')
param lgsvl_map = 'CubeTown'
param time_step = 1.0/10

model scenic.simulators.lgsvl.model

#CONSTANTS
MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20 / globalParameters.time_step
space = [2,3,4,5]


# GEOMETRY
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
startLane = straight_maneuver.startLane

centerlines = [straight_maneuver.startLane.centerline, straight_maneuver.connectingLane.centerline, straight_maneuver.endLane.centerline]
egoStart = (OrientedPoint at startLane.centerline[-1]) offset by Range(-2, 2) @ 0 

# --

conflicting_lefts = filter(lambda m: m.type == ManeuverType.LEFT_TURN, straight_maneuver.conflictingManeuvers)
leftTurn_maneuver = Uniform(*conflicting_lefts)
L_startLane = leftTurn_maneuver.startLane

L_centerlines = [leftTurn_maneuver.startLane.centerline, leftTurn_maneuver.connectingLane.centerline, leftTurn_maneuver.endLane.centerline]
actorStart = (OrientedPoint at L_startLane.centerline[-1]) offset by Range(-2, 2) @ 0 


# BEHAVIORS
behavior EgoBehavior(target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 1

	try: 
		do FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=10):
		take SetBrakeAction(brakeIntensity)


# PLACEMENT 
ego = Car following roadDirection from egoStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=15, trajectory=centerlines)

other = Car following roadDirection from actorStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=L_centerlines)

terminate when ego in straight_maneuver.endLane
terminate when simulation().currentTime > TERMINATE_TIME

# require that ego car reaches the intersection before the other car
