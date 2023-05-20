
param map = localPath('maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
model scenic.simulators.lgsvl.model
param time_step = 1.0/10

# CONSTANTS
uberSpeed = Range(30, 50)
turnSpeed = Range(20, 25)
brakeIntensity = Range(0.6, 0.8)

# LANES 
fourLane = filter(lambda i: i.is4Way, network.intersections)
intersection = Uniform(*fourLane)

# uber's lane
rightManeuvers = filter(lambda m: m.type == ManeuverType.RIGHT_TURN, intersection.maneuvers) # because only the rightmost lane will have a right maneuver
rightTurn = Uniform(*rightManeuvers)
egoStartLane = rightTurn.startLane

straightManeuvers = filter(lambda  m: m.type == ManeuverType.STRAIGHT, egoStartLane.maneuvers)  # problem with borregas - right lane is only for right turns
egoManeuver = Uniform(*straightManeuvers)

# actor's lane
conflictingLefts = filter(lambda  m: m.type == ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers) 
actorTurn = Uniform(*conflictingLefts)
actorStart = actorTurn.startLane

# stopped car's lane
stationaryLane = egoStartLane.sections[-1].laneToLeft

# POSITIONS
pos1 = (new OrientedPoint at stationaryLane.centerline[-1]) offset by Range(-2, 2) @ 0 
pos2 = (new OrientedPoint at stationaryLane.centerline[-1]) offset by Range(-2, 2) @ 0
dist1 = Range(1, 5)
dist2 = Range(10, 15)

egoPos = (new OrientedPoint at egoStartLane.centerline[-1]) offset by Range(-2, 2) @ 0 
egoDist = Range(20, 25)

actorPos = (new OrientedPoint at actorStart.centerline[-1]) offset by Range(-2, 2) @ 0 
actorDist = Range(2, 5)

# trajectories
egoTrajectory = [egoManeuver.startLane.centerline, egoManeuver.connectingLane.centerline, egoManeuver.endLane.centerline]
actorTrajectory = [actorTurn.startLane.centerline, actorTurn.connectingLane.centerline, actorTurn.endLane.centerline]


# BEHAVIORS
behavior actorCarBehavior(target_speed, trajectory, brake):
    try: 
        do FollowTrajectoryBehavior(target_speed = turnSpeed, trajectory = actorTrajectory)

    interrupt when(actor in actorTurn.endLane):
        take SetThrottleAction(0.0), SetBrakeAction(brakeIntensity)

# PLACEMENT
# Cars blocking view
stopped1 = new NPCCar following roadDirection from pos1 for -dist1
stopped2 = new NPCCar following roadDirection from pos2 for -dist2

# Uber
ego = new Car following roadDirection from egoPos for -egoDist,
    with speed uberSpeed,
    with behavior FollowTrajectoryBehavior(target_speed=uberSpeed, trajectory=egoTrajectory)

#assert (ego is right of stopped1)

# Turning Car
actor = new Car following roadDirection from actorPos for -actorDist,
    with behavior actorCarBehavior(target_speed = turnSpeed, trajectory = actorTrajectory, brake = brakeIntensity),
    with speed turnSpeed
