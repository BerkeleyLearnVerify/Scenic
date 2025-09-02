'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.carla.model --simulate
'''

# param map = localPath('../../assets/maps/CARLA/Town05.xodr')
# param carla_map = 'Town05'
# param time_step = 1.0/10

# model scenic.simulators.metadrive.model

# target = Vector(178.9, 138.0)
# lg = network.laneGroupAt(target, reject=False)
# curb = lg.curb

# ego = new Car on curb,
#         with regionContainedIn None,
#         # facing roadDirection

# # ego = new Car at (185.20, 77.41),
# #         with regionContainedIn None

# # ego = new Car at (185.20,77.41),
# #         with regionContainedIn None,
# #         facing roadDirection
'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.carla.model --simulate
'''

param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.metadrive.model

behavior PullIntoRoad():
    while (distance from self to ego) > 15:
        wait
    do FollowLaneBehavior(laneToFollow=ego.lane)

ego = new Car with behavior DriveAvoidingCollisions(avoidance_threshold=5)

rightCurb = ego.laneGroup.curb
spot = new OrientedPoint on visible rightCurb
badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
# parkedCar = new Car left of spot by 0.5,
#                 facing badAngle relative to spot.heading,
#                 with behavior PullIntoRoad
parkedCar = new Car on spot, facing roadDirection,
                with behavior PullIntoRoad

require (distance to parkedCar) > 20

monitor StopAfterInteraction():
    for i in range(50):
        wait
    while ego.speed > 2:
        wait
    for i in range(50):
        wait
    terminate
require monitor StopAfterInteraction()
terminate after 15 seconds   # in case ego never breaks
