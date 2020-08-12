"""Behaviors for dynamic agents in LGSVL."""

from scenic.simulators.lgsvl.actions import FollowWaypointsAction, SetDestinationAction

behavior DriveTo(target):
	action = SetDestinationAction(target)
	while True:
		take action

behavior FollowWaypoints(waypoints):
	action = FollowWaypointsAction(waypoints)
	while True:
		take action
