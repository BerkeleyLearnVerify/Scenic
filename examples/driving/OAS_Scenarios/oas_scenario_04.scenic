""" Scenario Description
Voyage OAS Scenario Unique ID: 2-2-XX-CF-STR-CAR:01
The ego vehicle follows the lead car which suddenly stops
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/LGSVL/cubetown.xodr')  # or other CARLA map that definitely works
param lgsvl_map = 'CubeTown'
model scenic.domains.driving.model

#CONSTANTS
MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -1*Uniform(5, 10)
STEPS_PER_SEC = 10

##DEFINING BEHAVIORS
behavior LeadCarBehavior():
	try:
		do FollowLaneBehavior()
	interrupt when simulation().currentTime > 5 * STEPS_PER_SEC:
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior CollisionAvoidance():
	while withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior FollowLeadCarBehavior():
	try: 
		do FollowLaneBehavior()

	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		do CollisionAvoidance()


##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

roads = network.roads

# make sure to put '*' to uniformly randomly select from all elements of the list, 'network.roads'
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

other = Car on select_lane.centerline,
		with behavior LeadCarBehavior()

ego = Car following roadDirection from other for INITIAL_DISTANCE_APART,
		with behavior FollowLeadCarBehavior()
