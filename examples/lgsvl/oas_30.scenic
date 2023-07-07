# 3 way intersection. ego turns left. ego has right of way.

param map = localPath('../../assets/maps/LGSVL/cubetown.xodr')
param lgsvl_map = 'CubeTown'
param time_step = 1.0/10

model scenic.simulators.lgsvl.model

# CONSTANTS
space = [2,3,4,5]

# GEOMETRY
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

left_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers)
ego_maneuver = Uniform(*left_maneuvers)
ego_L_centerlines = [ego_maneuver.startLane.centerline, ego_maneuver.connectingLane.centerline, ego_maneuver.endLane.centerline]
egoStart = (new OrientedPoint at ego_maneuver.startLane.centerline[-1]) offset by Range(-2, 2) @ 0 

# ---

conflicting_left = filter(lambda m: m.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
actor_maneuver = Uniform(*conflicting_left)
actor_centerlines = [actor_maneuver.startLane.centerline, actor_maneuver.connectingLane.centerline, actor_maneuver.endLane.centerline]
actorStart = actor_maneuver.startLane.centerline[-1]


# BEHAVIORS
behavior EgoBehavior(thresholdDistance, target_speed=20, trajectory = None):
    assert trajectory is not None
    brakeIntensity = 0.7

    try: 
        do FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

    interrupt when distanceToAnyCars(car=self, thresholdDistance=thresholdDistance):
        take SetBrakeAction(brakeIntensity)


# PLACEMENT
ego = new Car following roadDirection from egoStart for -Uniform(*space),
        with blueprint 'vehicle.tesla.model3',
        with behavior EgoBehavior(target_speed=10, trajectory=ego_L_centerlines, thresholdDistance = 20)

other = new Car following roadDirection from actorStart for -Uniform(*space),
        with blueprint 'vehicle.tesla.model3',
        with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=actor_centerlines)


# require that ego car reaches the intersection before the other car
