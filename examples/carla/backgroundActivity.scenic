""" Scenario Description
Background Activity
The simulation is filled with vehicles that freely roam around the town.
This simulates normal driving conditions, without any abnormal behaviors
"""
# SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../assets/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.lincoln.mkz_2017"
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

# Background activity
background_vehicles = []
for _ in range(25):
    lane = Uniform(*network.lanes)
    spot = new OrientedPoint on lane.centerline

    background_car = new Car at spot,
        with behavior AutopilotBehavior()
    background_vehicles.append(background_car)

background_walkers = []
for _ in range(10):
    sideWalk = Uniform(*network.sidewalks)
    background_walker = new Pedestrian in sideWalk,
        with behavior WalkBehavior()
    background_walkers.append(background_walker)


ego = new Car following roadDirection from spot for Range(-30, -20),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)
