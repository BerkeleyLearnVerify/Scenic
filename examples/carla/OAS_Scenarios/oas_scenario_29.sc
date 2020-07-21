
import scenic.simulators.carla.actions as actions
import time
from shapely.geometry import LineString
from scenic.core.regions import regionFromShapelyObject
from scenic.simulators.domains.driving.network import loadNetwork
from scenic.simulators.domains.driving.roads import ManeuverType
loadNetwork('/home/carla_challenge/Downloads/Town03.xodr')

from scenic.simulators.carla.model import *
simulator = CarlaSimulator('Town03')

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


behavior FollowWayPoints(target_speed=20, waypoints = None):
	assert waypoints is not None

	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	while True:
		nearest_line_points = waypoints.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)


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


rightTurn_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.RIGHT_TURN:
		rightTurn_manuevers.append(m)

rightTurn_maneuver = rightTurn_manuevers[1]
R_startLane = rightTurn_maneuver.startLane
R_connectingLane = rightTurn_maneuver.connectingLane
R_endLane = rightTurn_maneuver.endLane

R_centerlines = [R_startLane.centerline, R_connectingLane.centerline, R_endLane.centerline]

ego = Car on startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowWayPoints(target_speed=25, waypoints=concatenateCenterlines(centerlines))

other = Car on R_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowWayPoints(target_speed=5, waypoints=concatenateCenterlines(R_centerlines))