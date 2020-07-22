
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

behavior FollowLane(target_speed=20):

	""" Follow the lane that the vehicle is currently on """
	# target_speed in km/hr

	while True:
		nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)

twoLane_roads = []
for r in network.roads:
	if len(r.lanes) == 2:
		twoLane_roads.append(r)

selected_road = Uniform(*twoLane_roads)
lane = selected_road.lanes[1]

ego = Car on lane

spot = OrientedPoint on visible curb
perturbation_angle = (-10, 10) deg

parkedCar = Car at (left of spot),
				facing perturbation_angle relative to roadDirection

doubleParkedCar = Car at (left of parkedCar)

oncomingCar = Car on selected_road.backwardLanes[0],
				with behavior FollowLane(25)