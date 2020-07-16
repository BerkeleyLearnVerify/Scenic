import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

# from scenic.simulators.carla.models.model import *
from scenic.simulators.carla.model import * #note in scenic-devel-findaheng this is scenic.simulators.carla.models.model 


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
FOLLOWING_DISTANCE = 10 #normally 10, 40 when DELAY_TIME is 25, 50 to prevent collisions

behavior FastCarBehavior():
	# # take actions.SetSteerAction(0.3)
	# take actions.SetThrottleAction(0.9)
	# for _ in range(DELAY_TIME_2):
	# 	take None
	# take actions.SetThrottleAction(0.0)
	# # for _ in range(DELAY_TIME_2):
	# 	# take None
	# # take actions.SetThrottleAction(0.9)
	# # for _ in range(DELAY_TIME_2):
	# 	# take None
	# # take actions.SetThrottleAction(0.0)
	# # for _ in range(DELAY_TIME_2):
	# 	# take None
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	#TODO: Add a buffer
	take actions.SetThrottleAction(0.9)
	print("zoom")

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

initLaneSec = Options(laneSecsWithLeftLane)
# initLaneSec = laneSecsWithLeftLane[22] # NOTE: Hard coded for testing
leftLaneSec = initLaneSec.laneToLeft

spawnPt = initLaneSec.centerline[3]  # NOTE: Hard coded for testing
spawnVec = Vector(spawnPt[0], spawnPt[1])

slowCar = Car at spawnVec,
	with speed 0,
	with behavior SlowCarBehavior

fastCar = Car behind slowCar by 2*FOLLOWING_DISTANCE,
	with speed 0,
	with behavior FastCarBehavior

ego = Car behind slowCar by 4*FOLLOWING_DISTANCE,
	with speed 0, # changed to 4 from 12 so the SetThrottle action is more obvious visually.
	with behavior EgoBehavior

# pedestrian1 = Pedestrian right of slowCar by 5

# pedestrian2 = Pedestrian left  of slowCar by 5

# pedestrian3 = Pedestrian behind pedestrian1 by 5

# pedestrian4 = Pedestrian behind pedestrian3 by 5

# pedestrian5 = Pedestrian behind pedestrian4 by 5

# pedestrian6 = Pedestrian behind pedestrian5 by 5

# pedestrian7 = Pedestrian behind pedestrian2 by 5

# pedestrian8 = Pedestrian behind pedestrian7 by 5

# pedestrian9 = Pedestrian behind pedestrian8 by 5

# pedestrian10 = Pedestrian behind pedestrian9 by 5


#try adding pedestrians to increase scale