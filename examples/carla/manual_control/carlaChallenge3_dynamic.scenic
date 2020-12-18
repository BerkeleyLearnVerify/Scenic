""" Scenario Description
Traffic Scenario 03 (dynamic).
Obstacle avoidance without prior action.
The ego-vehicle encounters an obstacle / unexpected entity on the road and must perform an
emergency brake or an avoidance maneuver.
"""

# SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

# CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

PEDESTRIAN_SPEED = 5

THRESHOLD = 15

behavior PedestrianBehavior(speed=3):
    while (ego.speed <= 0.1):
        wait

    while (distance from self to ego) > THRESHOLD:
        wait

    take SetWalkingDirectionAction(0)
    take SetWalkingSpeedAction(speed)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py 

# make sure to put '*' to uniformly randomly select from all elements of the list, 'network.lanes'
lane = Uniform(*network.lanes)

spot = OrientedPoint on lane.centerline
vending_spot = OrientedPoint following roadDirection from spot for -3

pedestrian = Pedestrian right of spot by 3,
    with heading 90 deg relative to spot.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior(PEDESTRIAN_SPEED)

vending_machine = VendingMachine right of vending_spot by 3,
    with heading -90 deg relative to vending_spot.heading,
    with regionContainedIn None

ego = Car following roadDirection from spot for Range(-30, -20),
    with blueprint EGO_MODEL,
    with rolename "hero"

require (distance from ego to intersection) > 50
terminate when (distance to spot) > 50