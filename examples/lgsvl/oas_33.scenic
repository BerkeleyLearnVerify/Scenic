# 3 way intersection. ego goes straight. not specified.

param map = localPath('maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10

model scenic.simulators.lgsvl.model

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20


behavior EgoBehavior(target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		do FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=10):
		take SetBrakeAction(brakeIntensity)


fourLane = []
for i in network.intersections:
	if (len(i.incomingLanes) >= 8):
		fourLane.append(i)

intersection = fourLane[0] # hard coded so I don't have to fix the double-sampling
maneuvers = intersection.maneuvers

straight_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.STRAIGHT:
		straight_manuevers.append(m)

straight_maneuver = straight_manuevers[0]
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane

centerlines = [startLane.centerline, connectingLane.centerline, endLane.centerline]


leftTurn_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.LEFT_TURN:
		leftTurn_manuevers.append(m)

leftTurn_maneuver = leftTurn_manuevers[0]
L_startLane = leftTurn_maneuver.startLane
L_connectingLane = leftTurn_maneuver.connectingLane
L_endLane = leftTurn_maneuver.endLane

L_centerlines = [L_startLane.centerline, L_connectingLane.centerline, L_endLane.centerline]

ego = Car on startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=15, trajectory=centerlines)

other = Car on L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=5, trajectory=L_centerlines)
 
