import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *


"""
Ego-vehicle must go around a blocking object 
using the opposite lane, yielding to oncoming traffic.
Based on CARLA Challenge Scenario 5: 
https://carlachallenge.org/challenge/nhtsa/
"""

ONCOMING_THROTTLE = 0.6

behavior OncomingCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	while True:
		take actions.SetThrottleAction(ONCOMING_THROTTLE)	

# NOTE: List comprehension do not work in Scenic.
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			if laneSec.laneToLeft.isForward is not laneSec.isForward:
				laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane with opposing \
	traffic direction in network.'

#initLaneSec = Uniform(laneSecsWithLeftLane)
initLaneSec = laneSecsWithLeftLane[1] # NOTE: Hard coded for testing
leftLaneSec = initLaneSec.laneToLeft

spawnPt = Options(initLaneSec.centerline)  # NOTE: Hard coded for testing

leftPt = Options(leftLaneSec.centerline)
#leftPt = leftLaneSec.centerline[3]  # NOTE: Hard coded for testing

oncomingCar = Car at leftPt,
	with speed 0,
	with behavior OncomingCarBehavior

ego = Car at spawnPt,
	with speed 0

blockingCar = Car following roadDirection from ego by 16,
	with speed 0

require blockingCar can see oncomingCar

#oncoming = Car offset by -2 @ 0 #left of ego by 2

