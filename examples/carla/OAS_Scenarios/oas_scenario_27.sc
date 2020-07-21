
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

behavior FollowWayPoints(target_speed=20, waypoints = None):
	assert waypoints is not None

	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	while True:
		nearest_line_points = waypoints.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)


fourWayIntersections = []
for intersection in network.intersections:
	if intersection.is4Way:
		fourWayIntersections.append(intersection)

# intersection = Uniform(*fourWayIntersections)
intersection = fourWayIntersections[5]
maneuvers = intersection.maneuvers

straight_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.STRAIGHT:
		straight_manuevers.append(m)

straight_maneuver = straight_manuevers[0]
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane

print("startLane: ", startLane.centerline.points)
print("connectingLane: ", connectingLane.centerline.points)

def concatenateCenterlines(centerlines=[]):

	line = []
	if centerlines != []:
		for centerline in centerlines:
			for point in centerline:
				if point not in line:
					line.append(point)

	return regionFromShapelyObject(LineString(line))

centerlines = [startLane.centerline, connectingLane.centerline, endLane.centerline]

ego = Car on startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowWayPoints(target_speed=25, waypoints=concatenateCenterlines(centerlines))

# other = Car at lanes[2].centerline[-1]