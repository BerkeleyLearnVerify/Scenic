import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *


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

DELAY_TIME_1 = 1 # the delay time for ego
DELAY_TIME_2 = 40 #the delay time for the slow car
FOLLOWING_DISTANCE = 13 #normally 10, 40 when DELAY_TIME is 25, 50 to prevent collisions


behavior SlowCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	take actions.SetThrottleAction(0.1)
	print("zoom")


behavior EgoBehavior():
	# take actions.SetSteerAction(0.3)
	# counter = 0
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	take actions.SetThrottleAction(0.6)
	
	

# NOTE: List comprehension do not work in Scenic.
''' Writing it like this makes it take a very long time:
crossing = Options(network.intersections)
lane = Options(crossing.incomingLanes)
initLaneSec = lane.sections[0] # NOTE: Hard coded for testing

spawnPt = initLaneSec.centerline[0]  # NOTE: Hard coded for testing'''

spawnAreas = []
pedAreas = []
crossing = network.intersections[0]

for lane in crossing.incomingLanes:
	spawnAreas.append(lane)
for pedx in crossing.crossings:
	pedAreas.append(pedx)

initLaneSec = spawnAreas[0].sections[0]
spawnPt = initLaneSec.centerline[0]
spawnVec = Vector(spawnPt[0], spawnPt[1])

initPedSec = pedAreas[0]
pedPt = initPedSec.startSidewalk
pedVec = Vector(pedPt[0], pedPt[1])

	#with speed 0,
	#with behavior SlowCarBehavior

ped = Pedestrian at pedVec

ego = Car at spawnVec offset by 0 @ -5,
	with speed 0,
	with behavior EgoBehavior