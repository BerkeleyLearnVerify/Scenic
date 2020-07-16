import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town05.xodr')

from scenic.simulators.carla.model import *
from scenic.simulators.domains.driving.roads import *

"""
Ego encounters an unexpected obstacle and must perform and emergency brake or avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 09.

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


behavior CrossingCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	while True:
		take actions.SetThrottleAction(0.6)
	
	

# NOTE: List comprehension do not work in Scenic.
''' Writing it like this makes it take a very long time:
crossing = Options(network.intersections)
lane = Options(crossing.incomingLanes)
initLaneSec = lane.sections[0] # NOTE: Hard coded for testing

spawnPt = initLaneSec.centerline[0]  # NOTE: Hard coded for testing'''

spawnAreas = []
intersec = network.intersections[0] #Get one intersection

viable_Rturns = []
for lane in intersec.incomingLanes:
	lane_sec = lane.sections[-1]
	point = lane_sec.centerline[-1]
	spawnAreas.append(lane_sec)
	if ManeuverType.RIGHT_TURN in lane.maneuvers:
		viable_Rturns.append(lane_sec)



crossSec = spawnAreas[0]
egoSec = viable_Rturns[0]

cross = Car in crossSec
ego = Car in egoSec