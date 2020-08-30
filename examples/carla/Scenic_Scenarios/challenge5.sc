"""
Ego-vehicle performs a lane changing to evade a 
leading vehicle, which is moving too slowly.
Based on 2019 Carla Challenge Traffic Scenario 05.
"""
param map = localPath('../../carla/OpenDrive/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

#CONSTANTS
EGO_SPEED = 10
SLOW_CAR_SPEED = 6
EGO_TO_BICYCLE = 10
DIST_THRESHOLD = 15

#EGO BEHAVIOR: Follow lane, then perform a lane change
behavior EgoBehavior(leftpath, origpath=[]):
	
	try: 
		FollowLaneBehavior(EGO_SPEED)

	interrupt when distanceToAnyObjs(self, DIST_THRESHOLD):
		#print('THRESHOLD PASSED: CHANGING LANES')
		# FollowTrajectoryBehavior(EGO_SPEED, leftpath)
		LaneChangeBehavior(laneToSwitch=leftpath, target_speed=10)

#OTHER BEHAVIOR
behavior SlowCarBehavior():
	FollowLaneBehavior(SLOW_CAR_SPEED)

#GEOMETRY
laneSecsWithRightLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToRight != None:
			laneSecsWithRightLane.append(laneSec)

assert len(laneSecsWithRightLane) > 0, \
	'No lane sections with adjacent left lane in network.'

initLaneSec = Uniform(*laneSecsWithRightLane)
rightLane = initLaneSec.laneToRight

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Truck at spawnPt,
	with behavior EgoBehavior(rightLane, [initLaneSec])

cyclist = Bicycle following roadDirection from ego for EGO_TO_BICYCLE,
	with behavior SlowCarBehavior()

require (distance from ego to intersection) > 10
require (distance from cyclist to intersection) > 10