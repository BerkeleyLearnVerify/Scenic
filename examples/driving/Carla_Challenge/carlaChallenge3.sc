model scenic.domains.driving.model

"""
Leading vehicle decelerates suddenly due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 03.
"""

#CONSTANTS
EGO_SPEED = 10
THROTTLE_ACTION = 0.6
BRAKE_ACTION = 1.0
EGO_TO_OBSTACLE = (-20, -15)
EGO_BRAKING_THRESHOLD = 11
TRASH_POSITION_OFFSET = (-0.5,0.5)

#EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10):
	
	try: 
		FollowLaneBehavior(speed)

	interrupt when distanceToAnyObjs(self, EGO_BRAKING_THRESHOLD):
		take SetBrakeAction(BRAKE_ACTION)

#GEOMETRY

initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

obstacle = Trash at spawnPt offset by TRASH_POSITION_OFFSET @ 0

ego = Car following roadDirection from spawnPt by EGO_TO_OBSTACLE,
	with behavior EgoBehavior(EGO_SPEED)
