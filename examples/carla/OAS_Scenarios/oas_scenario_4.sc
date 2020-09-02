# from scenic.domains.driving.network import loadNetwork
# loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

# param map = localPath('../OpenDrive/Town01.xodr')
# param carla_map = 'Town01'

# from scenic.domains.driving.behaviors import *
# # from scenic.simulators.carla.model import *

param map = localPath('../../lgsvl/maps/cubetown.xodr')
param lgsvl_map = 'CubeTown'

model scenic.domains.driving.model

MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -1*Uniform(5, 10)
STEPS_PER_SEC = 10

behavior LeadCarBehavior():
	try:
		FollowLaneBehavior()
	interrupt when simulation().currentTime > 5 * STEPS_PER_SEC:
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior CollisionAvoidance():
	while distanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior FollowLeadCarBehavior():

	try: 
		FollowLaneBehavior()

<<<<<<< HEAD
	interrupt when distanceToAnyObjs(self, SAFETY_DISTANCE):
		CollisionAvoidance()
=======
	# interrupt when ((distance to other) < safety_distance):
	# 	CollisionAvoidance(brake_intensity=0.9)


# behavior TerminateAfterTime(time_threshold=5):
# 	start_time = time.time()

# 	while time.time() - start_time < time_threshold:
# 		wait

# 	terminate when True

# behavior LeadCarSuddenlyStops():

# 	sudden_stop_time = Range(5, 8)
# 	start_time = time.time()

# 	try:
# 		FollowLane(25)

# 	interrupt when time.time()-start_time > sudden_stop_time: 
# 		take actions.SetBrakeAction(MAX_BREAK_THRESHOLD)
# 		TerminateAfterTime()
>>>>>>> dynamics2


roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

other = Car on select_lane.centerline,
		with behavior LeadCarBehavior()

ego = Car following roadDirection from other by INITIAL_DISTANCE_APART,
		with behavior FollowLeadCarBehavior()