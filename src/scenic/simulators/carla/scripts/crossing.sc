"""CARLA Challenge #4."""

import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.models.model import *


# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================

behavior FollowWaypointsBehavior(waypoints, threshold=0.01):
	"""Folllow waypoints at a constant speed."""
	assert threshold >= 0, 'Cannot have a negative threshold.'

	for i in range(len(waypoints) - 1):
		currWaypoint, nextWaypoint = waypoints[i], waypoints[i+1]
		newVel = self.speed * subtractVectors(nextWaypoint, currWaypoint)
		take actions.SetVelocityAction(newVel)
		while distance from self to nextWaypoint > threshold:
			take None


behavior DriveLaneBehavior():
	"""Drive along centerline of current lane at a constant speed."""
	
	currLane = Network.get_lane_at(self.position)
	remainingLaneWaypoints = list(currLane.centerline)

	# NOTE: Ego should have spawned at a waypoint in its current lane
	assert self.position in remainingLaneWaypoints,\
		f"{self}'s position {self.position} is not a waypoint in lane {currLane}."

	for waypoint in currLane.centerline:
		if waypoint is not self.position:
			remainingLaneWaypoints.remove(waypoint)

	FollowWaypointsBehavior(remainingLaneWaypoints, threshold=0.01)


behavior SuddenBrakeBehavior():
	"""Apply brakes and emergency brake suddenly."""
	while self.speed > 0:
		take actions.SetBrakeAction(1.0)
	take actions.SetHandBrakeAction(True)
	take actions.SetBrakeAction(0.0)  # Release foot brake after stopped


behavior EgoBehavior(obstacle, brakeDist=5.0):
	"""Drive forward, braking suddenly if needed."""
	while True:
		if distance from self to obstacle <= brakeDist:
			SuddenBrakeBehavior()
		else:
			DriveLaneBehavior()


# ============================================================================
# -- SCENARIO ----------------------------------------------------------------
# ============================================================================

'''
Ego encounters an unexpected obstacle and must perform and emergency brake or avoidance maneuver. 
Based on 2019 Carla Challenge Traffic Scenario 04.

In this visualization, let: 
	P := Pedestrian walking on crosswalk.
	E := Ego vehicle driving on lane that intersects crosswalk.

---------| |---------
		 | |
---------| |---------
    E	 | |	
---------| |---------
		 | |
---------|P|---------
'''

intersectingRoad = None
while intersectingRoad is None:
	crossing = Uniform(Network.crossings)
	
	# Find a road, if any, that intersects with crossing
	for road in Network.roads:
		if crossing in road.crossings:
			intersectingRoad = road
			obstacle = Pedestrian at crossing.centerline[0],
				with behavior FollowWaypointsBehavior(crossing.centerline),
				with speed (2, 4)  # Q: is "self.speed" already part of Scenic syntax?
			break

MIN_SPAWN_DIST = 15

lane = Uniform(intersectingRoad.lanes)
pointBeforeCrosswalk = None
for waypoint in lane.centerline:
	if distance from waypoint to crossing <= MIN_SPAWN_DIST:
		pointBeforeCrosswalk = waypoint
		break

assert pointBeforeCrosswalk is not None, \
	'Could not find a spawn point for ego. Try changing minimum spawn distance.'

ego = Car at pointBeforeCrosswalk,
	with behavior behaviors.EgoBehavior(obstacle, brakeDist=5.0),
	with speed(10, 20)
