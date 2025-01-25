param map = localPath('../../assets/maps/CARLA/Town06.xodr')
param carla_map = 'Town06'
param render = False
model scenic.simulators.newtonian.driving_model

import math
import shapely

from scenic.core.distributions import distributionFunction
from scenic.core.type_support import toVector

from scenic.contracts.utils import leadDistance

STARTING_DISTANCE = 5

roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

behavior BrakeChecking():
    while True:
        do FollowLaneBehavior() for Range(1,4) seconds
        while self.speed > 0.1:
            take SetBrakeAction(1)

# Set up lead and ego cars
leadCar = new Car on select_lane.centerline,
        with behavior BrakeChecking()


class EgoCar(Car):
    targetDir[dynamic, final]: float(roadDirection[self.position].yaw)

ego = new EgoCar at roadDirection.followFrom(toVector(leadCar), -STARTING_DISTANCE, stepSize=0.1),
        with leadDist STARTING_DISTANCE,
        with behavior FollowLaneBehavior(), with name "EgoCar", with timestep 0.1

# Create/activate monitor to store lead distance
monitor UpdateDistance(tailCar, leadCar):
    while True:
        tailCar.leadDist = float(leadDistance(tailCar, leadCar, network))
        wait

require monitor UpdateDistance(ego, leadCar)

record ego.leadDist
record ego.targetDir
