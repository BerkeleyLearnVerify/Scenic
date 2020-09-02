# Traffic Scenario 04: Obstacle avoidance with prior action
# Definition: While performing a maneuver, the ego-vehicle finds an obstacle / unexpected entity on the road and must perform an emergency brake or an avoidance maneuver. 
# ego at intersection where a maneuver is available, ego takes any turn. obstacle is stationary according to sheets... but i want it to move

param map = localPath('maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10

model scenic.simulators.lgsvl.model

# CONSTANTS
TERMINATE_TIME = 20 / globalParameters.time_step
index1 = Uniform(0, 1, 2, 3)

# GEOMETRY
fourLane = []
for i in network.intersections:
	if (len(i.incomingLanes) >= 8):
		fourLane.append(i)

intersection = Uniform(*fourLane)
lane = intersection.incomingLanes[index1]
pos = (OrientedPoint at lane.centerline[-1]) offset by Range(-2, 2) @ 0 # at last stretch of centerline, off center by at most 2
turn = Uniform(*lane.maneuvers)
trajectory = [turn.startLane.centerline, turn.connectingLane.centerline, turn.endLane.centerline]
pt = Point on turn.connectingLane.centerline # point in the ego's path that the pedestrian should walk towards

# BEHAVIOR
behavior CrossingBehavior():
	randomSpeedup = Range(0, 1)
	startWalkingDist = 100#(10, 15)
	while True:
		egoDist = distance from ego to pt
		walkDist = distance from p to pt
		if (ego.speed == 0):
			egoSpeed = 1
		else:
			egoSpeed = ego.speed
		walkSpeed = randomSpeedup + ((egoSpeed * walkDist) / egoDist)
		if(egoDist <= startWalkingDist):
			take SetSpeedAction(30)
		else:
			take SetSpeedAction(0.0)

behavior EgoBehavior():
	brakeIntensity = Range(0.7, 1)
	try:
		do FollowTrajectoryBehavior(target_speed=(20, 30), trajectory=trajectory)
	interrupt when (distance to p) < 10:
		take SetThrottleAction(0), SetBrakeAction(brakeIntensity)


# PLACEMENT
ego = Car at pos, facing roadDirection, with speed 5, with behavior EgoBehavior

# change this - crossings added
# (using intersection boundary for LGSVL version since maps have no sidewalks)
walkerSpawn = Point on visible intersection.boundary
p = Pedestrian at walkerSpawn offset by Range(-2,2) @ Range(-2,2),
	facing toward pt,
	with regionContainedIn None,
	with behavior CrossingBehavior

terminate when ego in turn.endLane
terminate when simulation().currentTime > TERMINATE_TIME