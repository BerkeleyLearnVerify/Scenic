import scenic.simulators.carla.actions as actions

import random

from scenic.core.geometry import subtractVectors
from scenic.core.vectors import Vector

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')
from scenic.simulators.carla.models.model import *

"""
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 03.
"""

behavior EgoBehavior():
	while ((distance from ego to obstacle) > 8):
		take actions.SetManualGearShiftAction(False)
		take actions.SetThrottleAction(0.6)
	take actions.SetBrakeAction(1.0)

# NOTE: List comprehension do not work in Scenic.
possiblelanes = []
for lane in network.lanes:
	for laneSec in lane.sections:
		possiblelanes.append(laneSec)

#initLaneSec = network.lanes[0]
initLaneSec = possiblelanes[20] # NOTE: Hard coded for testing

spawnPt = initLaneSec.centerline[3]  # NOTE: Hard coded for testing
spawnVec = Vector(spawnPt[0], spawnPt[1])


obstacle = Car at spawnVec

ego = Car behind obstacle by 15, 
	with speed 12,
	with behavior EgoBehavior

