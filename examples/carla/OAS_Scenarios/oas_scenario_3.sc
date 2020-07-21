
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
	

behavior FollowLaneBehavior(target_speed=20):

	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	""" Follow the lane that the vehicle is currently on """
	# target_speed = 25 # km/hr

	while True:
		nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)

behavior CollisionAvoidance(safety_distance=10, brake_intensity=0.3):
	while (distance to other) < safety_distance:
		print("ego applying break!")
		take actions.SetBrakeAction(brake_intensity)


behavior FollowLeadCarBehavior(safety_distance=10):

	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	try: 
		FollowLaneBehavior(25)

	interrupt when ((distance to other) < safety_distance):
		CollisionAvoidance()


roads = network.roads

ego = Car on roads[1],
		with behavior FollowLeadCarBehavior(10),
		with blueprint 'vehicle.tesla.model3'

other = Car ahead of ego by 10,
		with behavior FollowLaneBehavior
