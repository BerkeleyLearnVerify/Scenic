""" Scenario Description
A parked car is placed off the curb. When the ego vehicle approaches, a pedestrian steps out from in front of the parked car and crosses the road.
The ego is expected to detect the pedestrian and brake before reaching them.

To run this file using the MetaDrive simulator:
    scenic examples/driving/pedestrianJaywalking.scenic --2d --model scenic.simulators.metadrive.model --simulate
"""
param map = localPath('../../assets/maps/CARLA/Town01.xodr')
model scenic.domains.driving.model

#CONSTANTS
PEDESTRIAN_TRIGGER_DISTANCE = 15     # Distance at which pedestrian begins to cross
BRAKE_TRIGGER_DISTANCE = 10          # Distance at which ego begins braking
EGO_TO_PARKED_CAR_MIN_DIST = 30      # Ensure ego starts far enough away
PEDESTRIAN_OFFSET = 3                # Offset for pedestrian placement ahead of parked car
PARKED_CAR_OFFSET = 1                # Offset for parked car from the curb

#EGO BEHAVIOR: Ego drives by following lanes, but brakes if a pedestrian is close
behavior DriveAndBrakeForPedestrians():
    try:
        do FollowLaneBehavior()
    interrupt when withinDistanceToAnyPedestrians(self, BRAKE_TRIGGER_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)

#PEDESTRIAN BEHAVIOR: Pedestrian crosses road when ego is near
behavior CrossRoad():
    while distance from self to ego > PEDESTRIAN_TRIGGER_DISTANCE:
        wait
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(1)

#SCENE SETUP
ego = new Car with behavior DriveAndBrakeForPedestrians()

rightCurb = ego.laneGroup.curb
spot = new OrientedPoint on visible rightCurb

parkedCar = new Car right of spot by PARKED_CAR_OFFSET, with regionContainedIn None

require distance from ego to parkedCar > EGO_TO_PARKED_CAR_MIN_DIST

new Pedestrian ahead of parkedCar by PEDESTRIAN_OFFSET,
    facing 90 deg relative to parkedCar,
    with behavior CrossRoad()

terminate after 30 seconds
