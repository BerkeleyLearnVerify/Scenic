"""CARLA Challenge #5."""

import random

from scenic.core.geometry import subtractVectors
from scenic.core.vectors import Vector

import scenic.simulators.carla.actions as actions
from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')
from scenic.simulators.carla.models.model import *


# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================
'''
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
	
	currLane = network.get_lane_at(self.position)
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

	currLane = network.get_lane_at(self.position)

	assert newLane is currLane.laneToLeft \
		or newLane is currLane.laneToRight, \
		'Lane to change is not adjacent to current lane.'

	if newLane is currLane.laneToLeft:
		steer *= -1.0
		adjacentEdge = currLane.laneToLeft.leftEdge
	else:
		adjacentEdge = currLane.laneToRight.rightEdge

	take actions.SetSteerAction(steer)
	while distance from self to adjacentEdge > threshold:
		take None

	take actions.SetSteerAction(-steer)
	while distance from self to newLane.centerline > threshold:
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

	oldLane = network.get_lane_at(self.position)
	oldSpeed = self.speed

	while distance from self to carToPass > minDist:
		DriveLaneBehavior()
	
	LaneChangeBehavior(newLane, steer=0.2, threshold=0.01)
	# Q: how to extract y-pos in local coord system?
	while self.position.y < carToPass.position.y + minDist:
		AccelerateBehavior(self.speed + 5.0, throttle=0.2)
	LaneChangeBehavior(oldLane, steer=0.2, threshold=0.01)
	DecelerateBehavior(oldSpeed, brake=0.2)
	
	while True:
		DriveLaneBehavior()
'''

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

# NOTE: List comprehension do not work in Scenic.
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane in network.'

#initLaneSec = random.choice(laneSecsWithLeftLane)
initLaneSec = laneSecsWithLeftLane[22] # NOTE: Hard coded for testing
leftLaneSec = initLaneSec.laneToLeft

spawnPt = initLaneSec.centerline[3]
spawnVec = Vector(spawnPt[0], spawnPt[1])


behavior SlowCarBehavior():
	take actions.SetThrottleAction(0.3)

behavior EgoBehavior():
	take actions.SetThrottleAction(0.6)
	for _ in range(9):
		take None
	print('Ego changing lanes left')
	take actions.SetSteerAction(-0.55)
	for _ in range(8):
		take None
	take actions.SetSteerAction(0.3)
	for _ in range(9):
		take None
	take actions.SetSteerAction(0)
	for _ in range(20):
		take None
	print('Ego changing lanes right')
	take actions.SetSteerAction(0.3)
	for _ in range(6):
		take None
	take actions.SetSteerAction(-0.3)
	for _ in range(6):
		take None
	take actions.SetSteerAction(0)


slowCar = Car at spawnVec,
	with speed 5,
	with behavior SlowCarBehavior

ego = Car behind slowCar by 10, 
	with speed 12,
	with behavior EgoBehavior
