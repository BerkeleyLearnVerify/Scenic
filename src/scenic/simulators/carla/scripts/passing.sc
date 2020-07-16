"""CARLA Challenge #5."""

import random
import math

from scenic.core.geometry import subtractVectors
from scenic.core.vectors import Vector

import scenic.simulators.carla.actions as actions
from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')
from scenic.simulators.carla.model import * #note in scenic-devel-findaheng this is scenic.simulators.carla.models.model 


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
		while (distance from self to nextWaypoint) > threshold:
			take None


behavior DriveLaneBehavior():
	"""Drive along centerline of current lane at a constant speed."""
	print("DriveLaneBehavior()")
	
	currLane = network.get_lane_at(self.position)
	remainingLaneWaypoints = list(currLane.centerline)

	# NOTE: All vehicle spawns should be at a waypoint in its current lane
	assert self.position in remainingLaneWaypoints, \
		f"{self}'s position {self.position} is not a waypoint in lane {currLane}."

	for waypoint in currLane.centerline:
		if waypoint is not self.position:
			remainingLaneWaypoints.remove(waypoint)

	FollowWaypointsBehavior(remainingLaneWaypoints, threshold=0.01)




behavior AccelerateBehavior(newSpeed, throttle=0.2):
	assert 0.0 < throttle <= 1.0, 'Throttle must be in range (0.0, 1.0].'
	print("AccelerateBehavior()")

	take actions.SetThrottleAction(throttle)
	while self.speed < newSpeed:
		take None
	take actions.SetThrottleAction(0.0)


behavior DecelerateBehavior(newSpeed, brake=0.2):
	assert 0.0 < brake <= 1.0, 'Brake must be in range (0.0, 1.0].'
	print("DecelerateBehavior()")

	take actions.SetBrakeAction(brake)
	while self.speed > newSpeed:
		take None
	take actions.SetBrakeAction(0.0)


behavior PassingBehavior(carToPass, oldLane, newLane, spawnPt, minDist=5.0):
	assert minDist > 0.0, 'Minimum distance must be positive.'

	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	take actions.SetThrottleAction(0.7)

	LaneChangeBehavior(newLane, steer=0.2, threshold=0.1)
	print("PassingBehavior()")

	# oldLane = network.get_lane_at(self.position)
	# oldSpeed = self.speed

	# while (distance from self to carToPass) > minDist:
	# 	print("entering while loop")
	# 	DriveLaneBehavior()
	
	# LaneChangeBehavior(newLane, steer=0.2, threshold=0.01)
	# Q: how to extract y-pos in local coord system?
	# while self.position.y < carToPass.position.y + minDist:
	# 	AccelerateBehavior(self.speed + 5.0, throttle=0.2)
	# LaneChangeBehavior(oldLane, steer=0.2, threshold=0.01)
	# DecelerateBehavior(oldSpeed, brake=0.2)
	
	# while True:
	# 	DriveLaneBehavior()

behavior LaneChangeBehavior(newLane, steer=0.2, threshold=0.1):
	print('applesauce1')
	assert threshold >= 0, 'Cannot have a negative threshold.'
	assert 0.0 < steer <= 1.0,\
		'(Absolute value of) steer must be in range (0.0, 1.0].'

	print("LaneChangeBehavior()")

	# currLane = network.laneAt(self.position)
	position = Vector(self.position[0], self.position[1])
	currLane = None

	while currLane is None:
		if network.laneAt(position):
			print("Hacker voice: I'm in")
			for lane in network.lanes:
				for section in lane.sections:
					if position in section:
						currLane = section
		else:
			# take actions.SetThrottleAction(0.4)
			wait
			position = self.position

	assert newLane is currLane.laneToLeft \
		or newLane is currLane.laneToRight, \
		'Lane to change is not adjacent to current lane.'

	if newLane is currLane.laneToLeft:
		steer *= -1.0
		adjacentEdge = currLane.laneToLeft.rightEdge
	else:
		adjacentEdge = currLane.laneToRight.leftEdge

	take actions.SetSteerAction(steer)
	print("threshold is: ", threshold)
	while (distance from self to adjacentEdge) > threshold:
		# print(distance from self to adjacentEdge)
		take None

	while (distance to newLane.centerline) > (10 * threshold):
		print(distance to newLane.centerline)
		take None

	take actions.SetSteerAction(-steer)
	refPoint = (Point on newLane.centerline).toVector() #compare to roadDirection of the position of the car
	print(refPoint)
	while not isParallel(newLane.road.defaultHeadingAt(refPoint), threshold):
		print(relative heading of newLane.road.defaultHeadingAt(refPoint))
		take None

	take actions.SetSteerAction(0.0)



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

def isParallel(heading2, threshold, heading1 = None):
	if heading1 is None:
		result = not (relative heading of heading2) > threshold and (relative heading of heading2) < (math.pi - threshold)
		print(result)
		return result
	else:
		return (relative heading of heading2 from heading1) > threshold and (relative heading of heading2 from heading1) < (math.pi - threshold)

# NOTE: List comprehension do not work in Scenic.
laneSecsWithLeftLane = []
for lane in network.lanes:
	for laneSec in lane.sections:
		if laneSec.laneToLeft is not None:
			laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, \
	'No lane sections with adjacent left lane in network.'

# initLaneSec = Options(laneSecsWithLeftLane)
initLaneSec = laneSecsWithLeftLane[22] # NOTE: Hard coded for testing
currentLane = initLaneSec.lane
leftLaneSec = initLaneSec.laneToLeft

spawnPt = initLaneSec.centerline[3]
spawnVec = Vector(spawnPt[0], spawnPt[1])

print(spawnPt)


# for lane in laneSecsWithLeftLane:
# 	if lane.containsPoint(spawnPt):
# 		currentLane = lane

# initLaneSec = None

# for section in currentLane.sections:
# 	if section.containsPoint(spawnVec):
# 		initLaneSec = section


behavior SlowCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)
	take actions.SetThrottleAction(0.3)


slowCar = Car at spawnPt,
	with speed 0,
	with behavior SlowCarBehavior

ego = Car behind slowCar by 10,
	with speed 0,
	with behavior PassingBehavior(slowCar, currentLane, leftLaneSec, spawnPt)


# Future improvements:
#	stablize after changing lanes
# 	merge back into the other lane
