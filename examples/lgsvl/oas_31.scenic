# ego turns left. ego has right of way but actor takes it (goes straight)

param map = localPath('maps/cubetown.xodr')
param lgsvl_map = 'CubeTown'

model scenic.simulators.lgsvl.model

# CONSTANTS
space = [2,3,4,5]

# GEOMETRY
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

left_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers)
ego_maneuver = Uniform(*left_maneuvers)
centerlines = [ego_maneuver.startLane.centerline, ego_maneuver.connectingLane.centerline, ego_maneuver.endLane.centerline]
egoStart = (OrientedPoint at ego_maneuver.startLane.centerline[-1]) offset by Range(-2, 2) @ 0 

# ---

conflicting_straight = filter(lambda m: m.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
actor_maneuver = Uniform(*conflicting_straight)
actor_centerlines = [actor_maneuver.startLane.centerline, actor_maneuver.connectingLane.centerline, actor_maneuver.endLane.centerline]
actorStart = actor_maneuver.startLane.centerline[-1]


# PLACEMENT
ego = Car following roadDirection from egoStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=10, trajectory=centerlines)

other = Car following roadDirection from actorStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=actor_centerlines)

# require ego arrives right before other does
