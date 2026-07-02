""" Scenario Description
Voyage OAS Scenario Unique ID: 2-2-XX-CF-STR-CAR
The ego vehicle follows the lead car

To run this file using the MetaDrive simulator:
    scenic examples/driving/OAS_Scenarios/oas_scenario_03.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/OAS_Scenarios/oas_scenario_03.scenic --2d --model scenic.simulators.carla.model --simulate
"""

param map = localPath('../../../assets/maps/CARLA/Town04.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town04'
model scenic.domains.driving.model

SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -10

behavior CollisionAvoidance(brake_intensity=0.3):
    while withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        take SetBrakeAction(brake_intensity)


behavior FollowLeadCarBehavior():

    try:
        do FollowLaneBehavior()

    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        do CollisionAvoidance()


roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

leadCar = new Car on select_lane.centerline,
        with behavior FollowLaneBehavior()

ego = new Car following roadDirection from leadCar for INITIAL_DISTANCE_APART,
        with behavior FollowLeadCarBehavior()
