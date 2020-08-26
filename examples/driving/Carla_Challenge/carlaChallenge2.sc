model scenic.domains.driving.model

"""
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 02.
"""

#CONSTANTS
EGO_SPEED = 10
LEAD_CAR_SPEED = 10

BRAKE_ACTION = 1.0
THROTTLE_ACTION = 0.6

EGO_TO_LEADCAR = -30
EGO_BRAKING_THRESHOLD = 15

LEADCAR_TO_OBSTACLE = -20
LEADCAR_BRAKING_THRESHOLD = 15


#EGO BEHAVIOR: Follow lane, and brake after passing a threshold distance to the leading car
behavior EgoBehavior(speed=10):
	
	try: 
		FollowLaneBehavior(speed)

	interrupt when distanceToAnyCars(self, EGO_BRAKING_THRESHOLD):
		take SetBrakeAction(BRAKE_ACTION)

#LEAD CAR BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior LeadingCarBehavior(speed=10):

	try: 
		FollowLaneBehavior(speed)

	interrupt when distanceToAnyCars(self, LEADCAR_BRAKING_THRESHOLD):
		take SetBrakeAction(BRAKE_ACTION)

#GEOMETRY
lane = Uniform(*network.lanes)
lanesec = Uniform(*lane.sections)

#PLACEMENT
obstacle = Car on lanesec

leadCar = Car following roadDirection from obstacle by LEADCAR_TO_OBSTACLE,
	with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

ego = Car following roadDirection from leadCar by EGO_TO_LEADCAR,
	with behavior EgoBehavior(EGO_SPEED)
