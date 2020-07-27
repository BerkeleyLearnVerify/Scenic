
import scenic.simulators.carla.actions as actions
import time

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *
simulator = CarlaSimulator('Town01')

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20

behavior FollowLane(target_speed=20):

	""" Follow the lane that the vehicle is currently on """
	# target_speed = 25 # km/hr
	while True:
		nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)

behavior CollisionAvoidance(safety_distance=10, brake_intensity=1):
	while (distance to other) < safety_distance:
		print("ego applying break!")
		take actions.SetBrakeAction(brake_intensity)


behavior FollowLeadCar(safety_distance=10):

	try: 
		FollowLane(25)

	interrupt when ((distance to other) < safety_distance):
		CollisionAvoidance()


behavior LeadCarSuddenlyStopsAndGo():

	sudden_stop_time = (3, 6)
	start_time = time.time()

	try:
		FollowLane(25)

	interrupt when (time.time()-start_time > sudden_stop_time) and (time.time()-start_time < sudden_stop_time+1): 
		# Unnatural to enforce time constraint
		take actions.SetBrakeAction(MAX_BREAK_THRESHOLD)


roads = network.roads
ego = Car on roads[1],
		with behavior FollowLeadCar(10),
		with blueprint 'vehicle.tesla.model3'

other = Car ahead of ego by 10,
		with behavior LeadCarSuddenlyStopsAndGo,
		with blueprint 'vehicle.tesla.model3'

#current_time = time.time()
#terminate when (time.time()-current_time > TERMINATE_TIME)