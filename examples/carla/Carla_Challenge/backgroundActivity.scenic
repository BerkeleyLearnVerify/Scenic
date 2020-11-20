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

EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

# EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10):
    do FollowLaneBehavior(speed)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py 

# background_activity = []
# for i in range(10):

# make sure to put '*' to uniformly randomly select from all elements of the list, 'network.lanes'
lane = Uniform(*network.lanes)
spot = OrientedPoint on lane.centerline

background_car = Car at spot,
    with autopilot True
# background_activity.append(background_car)

ego = Car following roadDirection from spot for Range(-30, -20),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)