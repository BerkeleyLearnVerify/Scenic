""" Scenario Description
ego vehicle follows the lead car
"""

param map = localPath('../../carla/OpenDrive/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.domains.driving.model
	
SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -10

behavior CollisionAvoidance(brake_intensity=0.3):
	while distanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(brake_intensity)


behavior FollowLeadCarBehavior():

	try: 
		do FollowLaneBehavior()

	interrupt when distanceToAnyObjs(self, SAFETY_DISTANCE):
		do CollisionAvoidance()


roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

other = Car on select_lane.centerline,
		with behavior FollowLaneBehavior()

ego = Car following roadDirection from other for INITIAL_DISTANCE_APART,
		with behavior FollowLeadCarBehavior()
