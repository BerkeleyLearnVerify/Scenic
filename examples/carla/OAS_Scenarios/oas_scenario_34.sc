
import scenic.simulators.carla.actions as actions
import time
from shapely.geometry import LineString
from scenic.core.regions import regionFromShapelyObject
from scenic.simulators.domains.driving.network import loadNetwork
from scenic.simulators.domains.driving.roads import ManeuverType
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *
from scenic.simulators.carla.behaviors import *

simulator = CarlaSimulator('Town01')

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
	brakeIntensity = 0.7

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=10):
		take actions.SetBrakeAction(brakeIntensity)


threeWayIntersections = []
for intersection in network.intersections:
	if intersection.is3Way:
		threeWayIntersections.append(intersection)

# intersection = Uniform(*fourWayIntersections)
intersection = threeWayIntersections[5]
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

# require that ego car reaches the intersection before the other car