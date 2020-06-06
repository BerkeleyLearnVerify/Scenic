import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.models.model import *


"""
Ego encounters an unexpected obstacle and must perform and emergency brake or avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 05.

In this visualization, let: 
	V := Slow vehicle that ego wants to pass.
	E_i := Ego vehicle changing lanes right (i=1),
		   speeding up past slow vehicle (i=2),
		   then returning ot its original lane (i=3).

-----------------------
initLane    E_1  V  E_3
-----------------------
rightLane	    E_2	 
-----------------------
"""

behavior SlowCarBehavior():
	take actions.SetThrottleAction(0.3)

behavior EgoBehavior():
	take actions.SetThrottleAction(0.6)
	for _ in range(9):
		take None
	print('Ego changing lanes left')
	take actions.SetSteerAction(-0.55)
	for _ in range(8):
		take None
	take actions.SetSteerAction(0.3)
	for _ in range(9):
		take None
	take actions.SetSteerAction(0)
	for _ in range(20):
		take None
	print('Ego changing lanes right')
	take actions.SetSteerAction(0.3)
	for _ in range(6):
		take None
	take actions.SetSteerAction(-0.3)
	for _ in range(6):
		take None
	take actions.SetSteerAction(0)

# NOTE: List comprehension do not work in Scenic.
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane in network.'

#initLaneSec = random.choice(laneSecsWithLeftLane)
initLaneSec = laneSecsWithLeftLane[22] # NOTE: Hard coded for testing
leftLaneSec = initLaneSec.laneToLeft

spawnPt = initLaneSec.centerline[3]  # NOTE: Hard coded for testing
spawnVec = Vector(spawnPt[0], spawnPt[1])

slowCar = Car at spawnVec,
	with speed 5,
	with behavior SlowCarBehavior

ego = Car behind slowCar by 10, 
	with speed 12,
	with behavior EgoBehavior
