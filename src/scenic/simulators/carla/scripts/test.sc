
import scenic.simulators.carla.actions as actions
import random
from scenic.core.geometry import subtractVectors
from scenic.core.vectors import Vector
from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *
from scenic.core.regions import PolygonalRegion, PolylineRegion
import math
import numpy as np

simulator = CarlaSimulator('Town01')
	

behavior FollowLaneBehavior():
	""" Follow the lane that the vehicle is currently on """
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	KP = 1
	KD = 0
	KI = 0
	target_speed = 10 # km/hr

	while True:
		nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)

		# cte = self.lane.centerline.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)


intersections = network.intersections

threeway_intersections = []
for inter in intersections:
	if inter.is3Way:
		threeway_intersections.append(inter)

threeway_intersection = threeway_intersections[0]
incoming_lanes = threeway_intersection.incomingLanes

print("incoming_lanes: ", incoming_lanes[0].centerline)


roads = network.roads

ego = Car on roads[1],
		with behavior FollowLaneBehavior