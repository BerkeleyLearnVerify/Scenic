# 3 way intersection. ego turns left. ego has right of way.

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import *

simulator = LGSVLSimulator('BorregasAve')
param time_step = 1.0/10

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20


behavior EgoBehavior(thresholdDistance, target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=thresholdDistance):
		take SetBrakeAction(brakeIntensity)


fourLane = []
for i in network.intersections:
	if (len(i.incomingLanes) >= 8):
		fourLane.append(i)

intersection = fourLane[0] # hard coded so I don't have to fix the double-sampling
maneuvers = intersection.maneuvers

leftTurn_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.LEFT_TURN:
		leftTurn_manuevers.append(m)

leftTurn_maneuver = leftTurn_manuevers[1]
ego_L_startLane = leftTurn_maneuver.startLane
ego_L_connectingLane = leftTurn_maneuver.connectingLane
ego_L_endLane = leftTurn_maneuver.endLane

ego_L_centerlines = [ego_L_startLane.centerline, ego_L_connectingLane.centerline, ego_L_endLane.centerline]


leftTurn_maneuver = leftTurn_manuevers[0]
other_L_startLane = leftTurn_maneuver.startLane
other_L_connectingLane = leftTurn_maneuver.connectingLane
other_L_endLane = leftTurn_maneuver.endLane

other_L_centerlines = [other_L_startLane.centerline, other_L_connectingLane.centerline, other_L_endLane.centerline]

ego = EgoCar on ego_L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=10, trajectory=ego_L_centerlines, thresholdDistance = 20)

other = EgoCar on other_L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=other_L_centerlines)


# require that ego car reaches the intersection before the other car
