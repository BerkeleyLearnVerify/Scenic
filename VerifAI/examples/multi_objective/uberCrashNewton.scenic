#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
mapPath = localPath('maps/Town05.xodr')
param map = mapPath
param carla_map = mapPath
param render = True
model scenic.domains.driving.model
param verifaiSamplerType = 'ce'

# Parameters of the scenario.
param DISTANCE_TO_INTERSECTION = VerifaiRange(-20, -10)
param HESITATION_TIME = VerifaiRange(0, 10)
param UBER_SPEED = VerifaiRange(10, 20)

# Ego vehicle just follows the trajectory specified later on.
behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.UBER_SPEED)
    terminate

# Crossing car hesitates for a certain amount of time before starting to turn.
behavior CrossingCarBehavior(trajectory):
    while simulation().currentTime < globalParameters.HESITATION_TIME:
        wait
    do FollowTrajectoryBehavior(trajectory = trajectory)
    terminate

# Find all 4-way intersections and set up trajectories for each vehicle.
fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersection)
rightLanes = filter(lambda lane: all([section._laneToRight is None for section in lane.sections]), intersec.incomingLanes)
startLane = Uniform(*rightLanes)
straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)

otherLane = straight_maneuver.endLane.group.opposite.lanes[-1]
left_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, otherLane.maneuvers)
left_maneuver = Uniform(*left_maneuvers)

ego_trajectory = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]
crossing_car_trajectory = [left_maneuver.startLane, left_maneuver.connectingLane, left_maneuver.endLane]

# Spawn each vehicle in the middle of its starting lane.
uberSpawnPoint = startLane.centerline[-1]
crossingSpawnPoint = otherLane.centerline[-1]

ego = new Car following roadDirection from uberSpawnPoint for globalParameters.DISTANCE_TO_INTERSECTION,
        with behavior EgoBehavior(trajectory = ego_trajectory)

crossing_car = new Car at crossingSpawnPoint,
                with behavior CrossingCarBehavior(crossing_car_trajectory)
