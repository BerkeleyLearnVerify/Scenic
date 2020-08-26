model scenic.domains.driving.model

"""
Ego-vehicle performs a lane changing to evade a 
leading vehicle, which is moving too slowly.
Based on 2019 Carla Challenge Traffic Scenario 05.
"""

#CONSTANTS
EGO_SPEED = 10
SLOW_CAR_SPEED = 6
EGO_TO_SLOWCAR = (15,20)
DIST_THRESHOLD = 10

#EGO BEHAVIOR: Follow lane, then perform a lane change
behavior EgoBehavior(origpath=[],leftpath=[]):
	
	try: 
		FollowLaneBehavior(EGO_SPEED)

	interrupt when distanceToAnyObjs(self, DIST_THRESHOLD):
		#print('THRESHOLD PASSED: CHANGING LANES')
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
