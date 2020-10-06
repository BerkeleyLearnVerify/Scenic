""" Scenario Description
Based on 2019 Carla Challenge Traffic Scenario 05.
Ego-vehicle performs a lane changing to evade a leading vehicle, which is moving too slowly.
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

#CONSTANTS
EGO_SPEED = 10
MOTORCYCLE_SPEED = 6
EGO_TO_MOTORCYCLE = 10
DIST_THRESHOLD = 15

## DEFINING BEHAVIORS
behavior EgoBehavior(leftpath, origpath=[]):
	#EGO BEHAVIOR: Follow lane, then perform a lane change once within DIST_THRESHOLD is breached and laneChange is yet completed
	laneChangeCompleted = False

	try: 
		do FollowLaneBehavior(EGO_SPEED)

	interrupt when withinDistanceToAnyObjs(self, DIST_THRESHOLD) and not laneChangeCompleted:
		do LaneChangeBehavior(laneSectionToSwitch=leftpath, target_speed=10)
		laneChangeCompleted = True

behavior CyclistBehavior():
	#OTHER CAR'S BEHAVIOR
	do FollowLaneBehavior(MOTORCYCLE_SPEED)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py
laneSecsWithRightLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec._laneToRight != None:
			laneSecsWithRightLane.append(laneSec)

assert len(laneSecsWithRightLane) > 0, \
	'No lane sections with adjacent left lane in network.'

# make sure to put '*' to uniformly randomly select from all elements of the list
initLaneSec = Uniform(*laneSecsWithRightLane)
rightLane = initLaneSec._laneToRight

#OJBECT PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car at spawnPt,
	with behavior EgoBehavior(rightLane, [initLaneSec])

# Set a specific vehicle model for the Bicycle. 
# The referenceable types of vehicles supported in carla are listed in scenic/simulators/carla/model.scenic
cyclist = Motorcycle following roadDirection from ego for EGO_TO_MOTORCYCLE,
	with behavior CyclistBehavior()

#EXPLICIT HARD CONSTRAINTS
require (distance from ego to intersection) > 10
require (distance from cyclist to intersection) > 10