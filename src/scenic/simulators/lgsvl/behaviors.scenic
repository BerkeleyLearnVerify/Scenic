"""Behaviors for dynamic agents in LGSVL."""

from scenic.simulators.lgsvl.simulator import FollowWaypointsAction, SetDestinationAction

## Behaviors

behavior DriveTo(target):
    take SetDestinationAction(target)

behavior FollowWaypoints(waypoints):
    take FollowWaypointsAction(waypoints)
