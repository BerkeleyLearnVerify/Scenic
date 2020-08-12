"""Behaviors for dynamic agents in LGSVL."""

from scenic.domains.driving.behaviors import *	# use all common driving behaviors
from scenic.simulators.lgsvl.actions import *

behavior DriveTo(target):
	action = SetDestinationAction(target)
	while True:
		take action

behavior FollowWaypoints(waypoints):
	action = FollowWaypointsAction(waypoints)
	while True:
		take action
