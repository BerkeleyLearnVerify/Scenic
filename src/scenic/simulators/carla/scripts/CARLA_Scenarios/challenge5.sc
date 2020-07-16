import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *


"""
Ego-vehicle performs a lane changing to evade a 
leading vehicle, which is moving too slowly.
Based on 2019 Carla Challenge Traffic Scenario 05.

"""

#CONSTANT
SLOW_CAR_THROTTLE = 0.3
EGO_TO_SLOWCAR = (8,20)

#EGO BEHAVIOR 
behavior EgoBehavior():
	# take actions.SetSteerAction(0.3)
	# counter = 0
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	while ((distance from ego to slowCar) > 8):
		take actions.SetThrottleAction(0.6)
	take actions.SetSteerAction(-0.6)
	for _ in range(10):
		take None
	take actions.SetSteerAction(0.5)
	for _ in range(9):
		take None
	take actions.SetSteerAction(0.1)
	for _ in range(9):
		take None
	take actions.SetSteerAction(0)
	for _ in range(20):
		take None

#OTHER BEHAVIOR
behavior SlowCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	while True:
		take actions.SetThrottleAction(SLOW_CAR_THROTTLE)

# GEOMETRY
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane in network.'

initLaneSec = laneSecsWithLeftLane[22] # NOTE: Hard coded for testing

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car at spawnPt

slowCar = Car following roadDirection from ego by EGO_TO_SLOWCAR,
	with speed 0,
	with behavior SlowCarBehavior
