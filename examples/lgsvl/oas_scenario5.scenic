

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/cubetown.xodr')
from scenic.simulators.lgsvl.model import *

simulator LGSVLSimulator('CubeTown')
timestep = 1.0/10
param time_step = timestep


MAX_BRAKE_THRESHOLD = 1
TERMINATE_TIME = 15 / timestep
STOP_LENGTH = 5 / timestep


behavior FollowLane(target_speed=20):

	""" Follow the lane that the vehicle is currently on """
	# target_speed = 25 # km/hr
	while True:
		take SetThrottleAction(0.5)
		# nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		# nearest_line_segment = PolylineRegion(nearest_line_points)
		# cte = nearest_line_segment.signedDistanceTo(self.position)
		# take actions.FollowLaneAction(target_speed, cte)

behavior CollisionAvoidance(safety_distance=10, brake_intensity=1):
	while (distance to other) < safety_distance:
		take SetBrakeAction(brake_intensity), SetThrottleAction(0)


behavior FollowLeadCar(safety_distance=10):

	try: 
		FollowLane(25)

	interrupt when ((distance to other) < safety_distance):
		CollisionAvoidance()


behavior LeadCarSuddenlyStopsAndGo():

	sudden_stop_time = (3, 6) * 10

	try:
		FollowLane(25)

	interrupt when (simulation().currentTime > sudden_stop_time
					and simulation().currentTime < sudden_stop_time+STOP_LENGTH):
		# TODO: Unnatural to enforce time constraint
		take SetBrakeAction(MAX_BRAKE_THRESHOLD), SetThrottleAction(0)



ego = EgoCar with behavior FollowLeadCar(10),
		with blueprint 'vehicle.tesla.model3'

other = EgoCar ahead of ego by 10,		
		with behavior LeadCarSuddenlyStopsAndGo,
		with blueprint 'vehicle.tesla.model3'
	
require (Point ahead of ego by 100) in road


terminate when simulation().currentTime > TERMINATE_TIME
