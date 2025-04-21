'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.carla.model --simulate
'''
import os
root_user = os.path.expanduser("~")
param map = localPath(root_user + '/ScenicGym/assets/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
param time_step = 1.0/10

model scenic.domains.driving.model

# behavior PullIntoRoad():
    # while (distance from self to ego) > 15:
        # wait
    # do FollowLaneBehavior(laneToFollow=ego.lane)

ego = new Car with behavior DriveAvoidingCollisions(avoidance_threshold=5)
car2 = new Car ahead of ego by 1
# rightCurb = ego.laneGroup.curb
# spot = new OrientedPoint on visible rightCurb
# badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
# parkedCar = new Car left of spot by 0.5,
                # facing badAngle relative to roadDirection,
                # with behavior PullIntoRoad

# require (distance to parkedCar) > 20

# monitor StopAfterInteraction():
    # for i in range(50):
        # wait
    # while ego.speed > 2:
        # wait
    # for i in range(50):
        # wait
    # terminate
# require monitor StopAfterInteraction()
terminate after 15 seconds   # in case ego never breaks
