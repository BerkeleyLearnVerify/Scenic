""" Scenario Description
Voyage OAS Scenario Unique ID: 2-2-XX-CF-STR-CAR:01
The ego vehicle follows the lead car which suddenly stops

To run this file using the MetaDrive simulator:
    scenic examples/driving/OAS_Scenarios/oas_scenario_04.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/OAS_Scenarios/oas_scenario_04.scenic --2d --model scenic.simulators.carla.model --simulate
"""

param map = localPath('../../../assets/maps/CARLA/Town07.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town07'
model scenic.domains.driving.model

MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -1*Uniform(5, 10)
STEPS_PER_SEC = 10

behavior LeadCarBehavior():
    try:
        do FollowLaneBehavior()
    interrupt when simulation().currentTime > 5 * STEPS_PER_SEC:
        take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior CollisionAvoidance():
    while withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior FollowLeadCarBehavior():
    try:
        do FollowLaneBehavior()

    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        do CollisionAvoidance()


roads = network.roads
select_road = Uniform(*roads)
select_lane = Uniform(*select_road.lanes)

other = new Car on select_lane.centerline,
        with behavior LeadCarBehavior()

ego = new Car following roadDirection from other for INITIAL_DISTANCE_APART,
        with behavior FollowLeadCarBehavior()
