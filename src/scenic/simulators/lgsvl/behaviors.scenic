"""Behaviors for dynamic agents in LGSVL."""

from scenic.simulators.lgsvl.simulator import FollowWaypointsAction, SetDestinationAction

## Behaviors

def DriveTo(target):
    def behavior(self):
        nonlocal target
        action = SetDestinationAction(target)
        invoke DoForever(action)
    return behavior

def FollowWaypoints(waypoints):
    def behavior(self):
        nonlocal waypoints
        action = FollowWaypointsAction(waypoints)
        invoke DoForever(action)
    return behavior

## Utilities

def DoForever(action):
    while True:
        yield action
