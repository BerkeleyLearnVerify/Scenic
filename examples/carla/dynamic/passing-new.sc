"""CARLA Challenge #5."""

from scenic.core.vectors import Vector

import scenic.simulators.carla.actions as actions
from scenic.simulators.domains.driving.network import loadLocalNetwork
loadLocalNetwork(__file__, '../OpenDrive/Town01.xodr')
from scenic.simulators.carla.model import *

simulator = CarlaSimulator('Town01')

# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================

behavior FollowWaypointsBehavior(waypoints, threshold=0.01):
	"""Folllow waypoints at a constant speed."""
	assert threshold >= 0, 'Cannot have a negative threshold.'

	for i in range(len(waypoints) - 1):
		currWaypoint, nextWaypoint = waypoints[i], waypoints[i+1]
		newVel = self.speed * (nextWaypoint - currWaypoint)
		take actions.SetVelocityAction(newVel)
		while (distance from self to nextWaypoint) > threshold:
			take None


behavior DriveLaneBehavior():
	"""Drive along centerline of current lane at a constant speed."""
	
	currLane = network.laneAt(self.position)
	remainingLaneWaypoints = list(currLane.centerline)

	# NOTE: All vehicle spawns should be at a waypoint in its current lane
	assert self.position in remainingLaneWaypoints, \
		f"{self}'s position {self.position} is not a waypoint in lane {currLane}."

	for waypoint in currLane.centerline:
		if waypoint is not self.position:
			remainingLaneWaypoints.remove(waypoint)

	FollowWaypointsBehavior(remainingLaneWaypoints, threshold=0.01)


behavior LaneChangeBehavior(newLane, steer=0.2, threshold=0.01):
	assert threshold >= 0, 'Cannot have a negative threshold.'
	assert 0.0 < steer <= 1.0,\
		'(Absolute value of) steer must be in range (0.0, 1.0].'

	currLane = network.laneAt(self.position)

	assert newLane is currLane.laneToLeft \
		or newLane is currLane.laneToRight, \
		'Lane to change is not adjacent to current lane.'

	if newLane is currLane.laneToLeft:
		steer *= -1.0
		adjacentEdge = currLane.laneToLeft.leftEdge
	else:
		adjacentEdge = currLane.laneToRight.rightEdge

	take actions.SetSteerAction(steer)
	while (distance from self to adjacentEdge) > threshold:
		take None

	take actions.SetSteerAction(-steer)
	while (distance from self to newLane.centerline) > threshold:
		take None

	take actions.SetSteerAction(0.0)


behavior AccelerateBehavior(newSpeed, throttle=0.2):
	assert 0.0 < throttle <= 1.0, 'Throttle must be in range (0.0, 1.0].'

	take actions.SetThrottleAction(throttle)
	while self.speed < newSpeed:
		take None
	take actions.SetThrottleAction(0.0)


behavior DecelerateBehavior(newSpeed, brake=0.2):
	assert 0.0 < brake <= 1.0, 'Brake must be in range (0.0, 1.0].'

	take actions.SetBrakeAction(brake)
	while self.speed > newSpeed:
		take None
	take actions.SetBrakeAction(0.0)


behavior PassingBehavior(carToPass, newLane, minDist=5.0):
	assert minDist > 0.0, 'Minimum distance must be positive.'

	oldLane = network.laneAt(self.position)
	road = oldLane.road
	oldSpeed = self.speed

	while (distance from self to carToPass) > minDist:
		DriveLaneBehavior()
	
	LaneChangeBehavior(newLane, steer=0.2, threshold=0.01)
	# Q: how to extract y-pos in local coord system?
	while road.distanceAlong(self) < road.distanceAlong(carToPass) + minDist:
		AccelerateBehavior(self.speed + 5.0, throttle=0.2)
	LaneChangeBehavior(oldLane, steer=0.2, threshold=0.01)
	DecelerateBehavior(oldSpeed, brake=0.2)
	
	while True:
		DriveLaneBehavior()


# ============================================================================
# -- SCENARIO ----------------------------------------------------------------
# ============================================================================

"""
Ego encounters an unexpected obstacle and must perform and emergency brake or avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 05.

In this visualization, let: 
	V := Slow vehicle that ego wants to pass.
	E_i := Ego vehicle changing lanes right (i=1),
		   speeding up past slow vehicle (i=2),
		   then returning ot its original lane (i=3).

-----------------------
initLane    E_1  V  E_3
-----------------------
rightLane	    E_2	 
-----------------------
"""

# Find all lane sections which have an adjacent lane to their left

laneSecs = network.laneSections.filter(lambda ls: ls.laneToLeft)

assert len(laneSecs) > 0, \
	'No lane sections with adjacent left lane in network.'

# Pick a random lane section

initLaneSec = Uniform(laneSecs)
leftLaneSec = initLaneSec.laneToLeft

# Create cars

behavior SlowCarBehavior():
	take actions.SetThrottleAction(0.3)

slowCar = Car on initLaneSec.centerline
	with speed 5,
	with behavior SlowCarBehavior

ego = Car behind slowCar by 10,
	with speed 12,
	with behavior PassingBehavior(slowCar, leftLaneSec)
