# 3 way intersection. ego going straight. ego has right of way, but actor blocking it. 

import lgsvl
from scenic.core.distributions import TruncatedNormal
import scenic.simulators.lgsvl.actions as actions
from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *
import scenic.simulators.domains.driving.roads as roads
import time
from shapely.geometry import LineString
from scenic.core.regions import regionFromShapelyObject
from scenic.simulators.domains.driving.roads import ManeuverType
from scenic.simulators.lgsvl.behaviors import *

simulator = LGSVLSimulator('BorregasAve')
param time_step = 1.0/10

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20

def concatenateCenterlines(centerlines=[]):
	line = []
	if centerlines != []:
		for centerline in centerlines:
			for point in centerline:
				if point not in line:
					line.append(point)

	return regionFromShapelyObject(LineString(line))


behavior EgoBehavior(target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 1

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=10):
		take actions.SetBrakeAction(brakeIntensity)


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
for m in straight_maneuver.conflictingManeuvers:
	if m.type == ManeuverType.LEFT_TURN:
		leftTurn_manuevers.append(m)

leftTurn_maneuver = leftTurn_manuevers[0]
L_startLane = leftTurn_maneuver.startLane
L_connectingLane = leftTurn_maneuver.connectingLane
L_endLane = leftTurn_maneuver.endLane

L_centerlines = [L_startLane.centerline, L_connectingLane.centerline, L_endLane.centerline]

ego = EgoCar on startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=15, trajectory=centerlines)

edge = OrientedPoint at L_startLane.centerline[-1], facing L_startLane.centerline.orientation
other = EgoCar behind edge by 3,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=L_centerlines),
		with regionContainedIn None

terminate when ego in endLane

# require that ego car reaches the intersection before the other car