param map = localPath('../../assets/maps/CARLA/Town06.xodr')
param carla_map = 'Town06'
model scenic.simulators.newtonian.driving_model

import math
import shapely

from scenic.core.distributions import distributionFunction
from scenic.core.type_support import toVector

STARTING_DISTANCE = 5

roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

# Lead distance functions
def leadDistanceInner(pos, tpos, lane, maxDistance):
    pos = lane.centerline.project(toVector(pos))
    tpos = toVector(tpos)
    if not lane.containsPoint(tpos):
        # Check if we are in the same lane as the target. If not,
        # advance to the start of the any possible successor lane.
        covered_dist = lane.centerline.length - shapely.line_locate_point(lane.centerline.lineString, shapely.Point(*pos))
        succ_lanes = [m.connectingLane if m.connectingLane else m.endLane for m in lane.maneuvers]
        new_pos = lane.centerline.end

        remMaxDistance = maxDistance-covered_dist
        if remMaxDistance <= 0:
            return float("inf")

        rem_dist = min((leadDistanceInner(new_pos, tpos, new_lane, remMaxDistance) for new_lane in succ_lanes), default=maxDistance)
        return covered_dist + rem_dist

    # If we're in the same lane as the target, return the accumulated distance plus
    # the remaining distance to the point
    passed_dist = shapely.line_locate_point(lane.centerline.lineString, shapely.Point(*pos))
    total_dist = shapely.line_locate_point(lane.centerline.lineString, shapely.Point(*tpos))
    return total_dist - passed_dist

def leadDistance(source, target, maxDistance=250):
    # Find all lanes this point could be a part of and recurse on them.
    viable_lanes = [lane for lane in network.lanes if lane.containsPoint(source.position)]

    return min(min((leadDistanceInner(source, target, lane, maxDistance) for lane in viable_lanes), default=maxDistance), maxDistance)

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
        with leadDist STARTING_DISTANCE, with behavior FollowLaneBehavior(), with name "EgoCar", with timestep 0.1

# Create/activate monitor to store lead distance
monitor UpdateDistance(tailCar, leadCar):
    while True:
        tailCar.leadDist = float(leadDistance(tailCar, leadCar))
        wait

require monitor UpdateDistance(ego, leadCar)

record ego.leadDist
record ego.targetDir
