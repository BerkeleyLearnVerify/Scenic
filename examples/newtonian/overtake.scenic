""" Scenario Description
Voyage OAS Scenario Unique ID: 2-2-XX-CF-STR-CAR:01
The ego vehicle follows the lead car which suddenly stops
"""

param map = localPath('./maps/Straight2LaneSame.xodr')
model scenic.simulators.newtonian.model

MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 6
INITIAL_DISTANCE_APART = -1*Uniform(5, 10)
STEPS_PER_SEC = 10

behavior AdoSafeCarBehavior(lane_section):
	try:
        do LaneChangeBehavior(lane_section, target_speed=7) for 30 steps
        do FollowLaneBehavior(target_speed=5)
	interrupt when withinDistanceToAnyCars(self, SAFETY_DISTANCE):
        do FollowLaneBehavior(target_speed=2) for 5 steps

behavior AdoDangerousCarBehavior(lane_section):
    danger_distance = 5
    try:
        do LaneChangeBehavior(lane_section, target_speed=7) for 30 steps
        do FollowLaneBehavior(target_speed=7)
    interrupt when withinDistanceToAnyCars(self, danger_distance):
        do FollowLaneBehavior(target_speed=9) for 5 steps


behavior CollisionAvoidance():
	while withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


roads = network.roads
select_road = roads[0]
select_lane = select_road.lanes[0]
other_lane = select_road.lanes[1]

other = Car at 3.72 @ -25,
		with behavior Uniform(AdoDangerousCarBehavior(select_lane.sections[0]), AdoSafeCarBehavior(select_lane.sections[0]))

ego = Car at 7.42 @ -25,
		with behavior FollowLaneBehavior(target_speed=6)
terminate when simulation().currentTime > 10 * STEPS_PER_SEC