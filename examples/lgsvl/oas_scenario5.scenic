import lgsvl
import scenic.simulators.lgsvl.actions as actions
import time
from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/cubetown.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import DriveTo, FollowWaypoints

simulator = LGSVLSimulator('CubeTown')
param time_step = 1.0/10


MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20

behavior FollowLane(target_speed=20):

	""" Follow the lane that the vehicle is currently on """
	# target_speed = 25 # km/hr
	while True:
		take actions.SetThrottleAction(0.5)
		# nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		# nearest_line_segment = PolylineRegion(nearest_line_points)
		# cte = nearest_line_segment.signedDistanceTo(self.position)
		# take actions.FollowLaneAction(target_speed, cte)

behavior CollisionAvoidance(safety_distance=10, brake_intensity=1):
	while (distance to other) < safety_distance:
		print("ego applying break!")
		take actions.SetBrakeAction(brake_intensity)
		take actions.SetThrottleAction(0)


behavior FollowLeadCar(safety_distance=10):

	try: 
		FollowLane(25)

	interrupt when ((distance to other) < safety_distance):
		CollisionAvoidance()


behavior LeadCarSuddenlyStopsAndGo():

	sudden_stop_time = (3, 6) * 10

	try:
		FollowLane(25)

	interrupt when (simulation().currentTime > sudden_stop_time) and (simulation().currentTime < sudden_stop_time+20):
		# Unnatural to enforce time constraint
		take actions.SetBrakeAction(MAX_BREAK_THRESHOLD)
		take actions.SetThrottleAction(0)
		print("leadcar brake")



ego = EgoCar with behavior FollowLeadCar(10),
		with blueprint 'vehicle.tesla.model3'

other = EgoCar ahead of ego by 10,		
		with behavior LeadCarSuddenlyStopsAndGo,
		with blueprint 'vehicle.tesla.model3'
	
require (Point ahead of ego by 100) in road

#current_time = time.time()
#terminate when (time.time()-current_time > TERMINATE_TIME)
