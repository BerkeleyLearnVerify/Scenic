# 3 way intersection. ego car turns left. actor has right of way.

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/cubetown.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import *

simulator = LGSVLSimulator('CubeTown')
param time_step = 1.0/10

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20


space = [2,3,4,5]

# GEOMETRY

threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)


startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane

centerlines = [startLane.centerline, connectingLane.centerline, endLane.centerline]
egoStart = startLane.centerline[-1]



left_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers)
leftTurn_maneuver1 = Uniform(left_maneuvers)
ego_L_startLane = leftTurn_maneuver.startLane
ego_L_connectingLane = leftTurn_maneuver.connectingLane
ego_L_endLane = leftTurn_maneuver.endLane
ego_L_centerlines = [ego_L_startLane.centerline, ego_L_connectingLane.centerline, ego_L_endLane.centerline]

# ---

conflicting_lefts = filter(lambda m: m.type == ManeuverType.LEFT_TURN, straight_maneuver.conflictingManeuvers)
leftTurn_maneuver = Uniform(*conflicting_lefts)

L_startLane = leftTurn_maneuver.startLane
L_connectingLane = leftTurn_maneuver.connectingLane
L_endLane = leftTurn_maneuver.endLane

L_centerlines = [L_startLane.centerline, L_connectingLane.centerline, L_endLane.centerline]
actorStart = L_startLane.centerline[-1]





leftTurn_maneuver2 = leftTurn_manuevers[0]
other_L_startLane = leftTurn_maneuver.startLane
other_L_connectingLane = leftTurn_maneuver.connectingLane
other_L_endLane = leftTurn_maneuver.endLane
other_L_centerlines = [other_L_startLane.centerline, other_L_connectingLane.centerline, other_L_endLane.centerline]







# BEHAVIOR
behavior EgoBehavior(thresholdDistance, target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=thresholdDistance):
		take SetBrakeAction(brakeIntensity)

ego = EgoCar on ego_L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=10, trajectory=ego_L_centerlines, thresholdDistance = 20)

other = EgoCar on other_L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=other_L_centerlines)


# require that other car reaches the intersection before the ego car
