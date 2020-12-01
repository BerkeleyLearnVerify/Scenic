""" Scenario Description
Background Activity
The simulation is filled with vehicles that freely roam around the town.
This simulates normal driving conditions, without any abnormal behaviors
"""
# SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

# EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when withinDistanceToObjsInLane(self, 10):
        take SetBrakeAction(1.0)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py 

background_activity = []
for i in range(40):

    # make sure to put '*' to uniformly randomly select from all elements of the list, 'network.lanes'
    lane = Uniform(*network.lanes)
    spot = OrientedPoint on lane.centerline

    background_car = Car at spot,
        with autopilot True
    background_activity.append(background_car)

ego = Car following roadDirection from spot for Range(-30, -20),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)
