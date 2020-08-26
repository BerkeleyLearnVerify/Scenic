# from scenic.domains.driving.network import loadNetwork
# loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

# param map = localPath('../OpenDrive/Town01.xodr')
# param carla_map = 'Town01'

# from scenic.domains.driving.behaviors import *
# # from scenic.simulators.carla.model import *

# model scenic.domains.driving.model

param map = localPath('../../lgsvl/maps/cubetown.xodr')
param lgsvl_map = 'CubeTown'

model scenic.domains.driving.model

MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 8
PARKING_SIDEWALK_OFFSET_RANGE = 2
CUT_IN_TRIGGER_DISTANCE = Uniform(10, 12)

behavior CutInBehavior(laneToFollow):
	while (distance from self to ego) > CUT_IN_TRIGGER_DISTANCE:
		wait

	FollowLaneBehavior(laneToFollow = laneToFollow)

behavior CollisionAvoidance():
	while distanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior EgoBehavior():
	try: 
		FollowLaneBehavior()

	interrupt when distanceToAnyObjs(self, SAFETY_DISTANCE):
		CollisionAvoidance()


roads = network.roads
select_road = Uniform(*roads)
# select_lane = Uniform(*select_road.lanes)
ego_lane = select_road.lanes[0]

ego = Car on ego_lane.centerline,
		with behavior EgoBehavior()
		
spot = OrientedPoint on visible curb
parkedHeadingAngle = Uniform(-1,1)*(10,20) deg

other = Car left of (spot offset by PARKING_SIDEWALK_OFFSET_RANGE @ 0), facing parkedHeadingAngle relative to ego.heading,
			with behavior CutInBehavior(ego_lane),
			with regionContainedIn None

require (angle from ego to other) - ego.heading < 0 deg
require (distance from ego to other) > 10 and (distance from ego to other) < 20