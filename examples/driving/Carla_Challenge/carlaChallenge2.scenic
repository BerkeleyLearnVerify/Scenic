""" Scenario Description
Based on 2019 Carla Challenge Traffic Scenario 02.
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Note: The scenario may fail if the leadCar or the ego get past the intersection while following the roadDirection
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.domains.driving.model

#CONSTANTS
EGO_SPEED = 10
LEAD_CAR_SPEED = 10

BRAKE_ACTION = 1.0
THROTTLE_ACTION = 0.6

EGO_TO_LEADCAR = -30
EGO_BRAKING_THRESHOLD = 15

LEADCAR_TO_OBSTACLE = -20
LEADCAR_BRAKING_THRESHOLD = 15

## DEFINING BEHAVIORS
#EGO BEHAVIOR: Follow lane, and brake after passing a threshold distance to the leading car
behavior EgoBehavior(speed=10):
	
	try: 
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyCars(self, EGO_BRAKING_THRESHOLD):
		take SetBrakeAction(BRAKE_ACTION)

#LEAD CAR BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior LeadingCarBehavior(speed=10):

	try: 
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyCars(self, LEADCAR_BRAKING_THRESHOLD):
		take SetBrakeAction(BRAKE_ACTION)

##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# make sure to put '*' to uniformly randomly select from all elements of the list, 'network.lanes'
lane = Uniform(*network.lanes)

##OBJECT PLACEMENT
obstacle = Car on lane

leadCar = Car following roadDirection from obstacle for LEADCAR_TO_OBSTACLE,
	with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

ego = Car following roadDirection from leadCar for EGO_TO_LEADCAR,
	with behavior EgoBehavior(EGO_SPEED)


