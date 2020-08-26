from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

param map = localPath('../OpenDrive/Town03.xodr')
param carla_map = 'Town03'

from scenic.domains.driving.behaviors import *
# from scenic.simulators.carla.model import *

model scenic.domains.driving.model
import scenic.simulators.carla.actions as actions

"""
Ego-vehicle performs a lane changing to evade a 
leading vehicle, which is moving too slowly.
Based on 2019 Carla Challenge Traffic Scenario 05.

"""

#CONSTANTS
#SLOW_CAR_THROTTLE = 0.3
EGO_SPEED = 10
SLOW_CAR_SPEED = 6
EGO_TO_SLOWCAR = (15,20)
DIST_THRESHOLD = 10

#EGO BEHAVIOR 
behavior EgoBehavior(origpath=[],leftpath=[]):
	
	try: 
		FollowLaneBehavior(EGO_SPEED)

	# interrupt when ((distance to slowCar) < DIST_THRESHOLD):
	interrupt when distanceToAnyObjs(self, DIST_THRESHOLD):
		print('THRESHOLD PASSED: CHANGING LANES')
		FollowTrajectoryBehavior(EGO_SPEED,leftpath)

#OTHER BEHAVIOR
behavior SlowCarBehavior():
	FollowLaneBehavior(SLOW_CAR_SPEED)

#GEOMETRY
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane in network.'

initLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = initLaneSec.laneToLeft

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car at spawnPt,
	with behavior EgoBehavior([initLaneSec.centerline], [leftLaneSec.centerline])

slowCar = Car following roadDirection from ego by EGO_TO_SLOWCAR,
	with behavior SlowCarBehavior()
