
import scenic.simulators.carla.actions as actions
from scenic.simulators.carla.model import roadDirection

behavior AccelerateForwardBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

behavior LanekeepingBehavior(gain=0.1):
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

	while True:
		delta = self.heading relative to (roadDirection at self.position)
		take actions.SetSteerAction(-gain * delta)

behavior WalkForwardBehavior():
	take actions.SetSpeedAction(0.5)

behavior ConstantThrottleBehavior(x):
    take actions.SetThrottleAction(x)


behavior FollowLaneBehavior(speed = 25):

	while True:
		nearest_line_points = waypoints.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		take actions.FollowLaneAction(target_speed, cte)



# behavior FollowManeuverAtIntersection(maneuver, intersection_type):
# 	assert 