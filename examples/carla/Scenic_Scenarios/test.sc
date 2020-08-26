from scenic.domains.driving.network import loadNetwork
map_name = 'Town01'

loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/' + map_name +'.xodr')

param map = localPath('../OpenDrive/' + map_name + '.xodr')
param carla_map = map_name
model scenic.domains.driving.model

# # ego = Pedestrian at 72.5396450994685 @ -149.31024140469097
# ego = Pedestrian at 65 @ -142, 
# 		facing roadDirection,
# 		with regionContainedIn None

# ego = Car

# spot = OrientedPoint on visible curb
# badAngle = Uniform(-1, 1) * (10, 20) deg
# parkedCar = Car left of spot by 0.5, 
# 	facing badAngle relative to roadDirection


behavior PullIntoRoad():
    while (distance from self to ego) > 15:
        wait
    FollowLaneBehavior(laneToFollow=ego.lane)

# ego = Car at 110.40003786419028 @ -197.04445318855383,
# 		with blueprint 'vehicle.bmw.grandtourer'
# with behavior DriveAvoidingCollisions(avoidance_threshold=5)
# ego = Car at -249.28150609142915 @ -35.55545204647514,
# 		with blueprint 'vehicle.toyota.prius'

# ego = Car at -255 @ -37,
		# facing -1.5721371180409895
		# with blueprint 'vehicle.toyota.prius'
		# with blueprint 'vehicle.bmw.grandtourer'
ego = Car with blueprint 'vehicle.mercedes-benz.coupe'

# rightCurb = ego.laneGroup.rightEdge
# spot = OrientedPoint on visible rightCurb
# badAngle = Uniform(1.0, -1.0) * (10, 20) deg
# parkedCar = Car left of spot by 0.5,
#                 facing badAngle relative to roadDirection,
#                 with behavior PullIntoRoad

# require (distance to parkedCar) > 20
# require ego.lane != parkedCar.lane

# monitor StopAfterInteraction:
# 	for i in range(50):
# 		wait
# 	while ego.speed > 2:
# 		wait
# 	for i in range(50):
# 		wait
# 	terminate