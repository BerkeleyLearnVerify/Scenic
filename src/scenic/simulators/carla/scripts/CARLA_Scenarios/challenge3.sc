import scenic.simulators.carla.actions as actions

import random

from scenic.core.geometry import subtractVectors
from scenic.core.vectors import Vector

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')
from scenic.simulators.carla.model import *

"""
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 03.
"""

#CONSTANTS
THROTTLE_ACTION = 0.6
BRAKE_ACTION = 1.0
EGO_TO_OBSTACLE = (-20, -15)
EGO_BRAKING_THRESHOLD = 8
TRASH_POSITION_OFFSET = (-1,1)

#EGO BEHAVIOR
behavior EgoBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	
	while ((distance from ego to obstacle) > EGO_TO_OBSTACLE_THRESHOLD):
		take actions.SetThrottleAction(THROTTLE_ACTION)
	take actions.SetBrakeAction(BRAKE_ACTION)

#GEOMETRY
possiblelanes = []
for lane in network.lanes:
	for laneSec in lane.sections:
		possiblelanes.append(laneSec)

initLane = network.lanes[0] # NOTE: Hard coded for testing
initLaneSec = initLane.sections[0] # NOTE: Hard coded for testing

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline  

obstacle = Trash at spawnPt offset by TRASH_POSITION_OFFSET @ 0

ego = Car following roadDirection from spawnPt by EGO_TO_OBSTACLE

