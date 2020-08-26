from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

param map = localPath('../OpenDrive/Town01.xodr')
param carla_map = 'Town01'

from scenic.domains.driving.behaviors import *
# from scenic.simulators.carla.model import *

model scenic.domains.driving.model
import scenic.simulators.carla.actions as actions

"""
Leading vehicle decelerates suddently due to an obstacle and 
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

#EGO BEHAVIOR
behavior EgoBehavior(speed=10):
	
	try: 
		FollowLaneBehavior(speed)

	interrupt when distanceToAnyObjs(self, EGO_BRAKING_THRESHOLD):
		take actions.SetBrakeAction(BRAKE_ACTION)

#GEOMETRY

initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

obstacle = Trash at spawnPt offset by TRASH_POSITION_OFFSET @ 0

ego = Car following roadDirection from spawnPt by EGO_TO_OBSTACLE,
	with behavior EgoBehavior(EGO_SPEED)
