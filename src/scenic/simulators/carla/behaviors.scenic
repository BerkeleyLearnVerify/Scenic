
import math

from scenic.simulators.domains.driving.roads import ManeuverType
from scenic.core.regions import regionFromShapelyObject

from scenic.simulators.carla.actions import *
import scenic.simulators.carla.controllers as controllers
from scenic.simulators.carla.model import roadDirection

def concatenateCenterlines(centerlines=[]):
	return PolylineRegion.unionAll(centerlines)

def distance(pos1, pos2):
	""" pos1, pos2 = (x,y) """
	return math.hypot(pos1[0]-pos2[0], pos1[1]-pos2[1])

def distanceToAnyCars(car, thresholdDistance):
	""" returns boolean """
	objects = simulation().objects
	for obj in objects:
		if distance(car.position, obj.position) < 0.1:
			# this means obj==car
			pass
		elif distance(car.position, obj.position) < thresholdDistance:
			return True
	return False

behavior AccelerateForwardBehavior():
	take SetReverseAction(False), SetHandBrakeAction(False), SetThrottleAction(0.5)

behavior LanekeepingBehavior(gain=0.1):
	take SetReverseAction(False), SetHandBrakeAction(False), SetThrottleAction(0.5)

	while True:
		delta = self.heading relative to (roadDirection at self.position)
		take SetSteerAction(-gain * delta)

behavior WalkForwardBehavior():
	take SetSpeedAction(0.5)

behavior ConstantThrottleBehavior(x):
    take SetThrottleAction(x)


behavior FollowLaneBehavior(target_speed = 25, network = None):

	# instantiate longitudinal and latitudinal pid controllers
	dt = simulation().timestep
	_lon_controller = controllers.PIDLongitudinalController(dt=dt)
	_lat_controller = controllers.PIDLateralController(dt=dt)
	past_steer_angle = 0

	while True:
		cte = self.lane.centerline.signedDistanceTo(self.position)
		speed_error = target_speed - self.speed

		# compute throttle : Longitudinal Control
		throttle = _lon_controller.run_step(speed_error)

		# compute steering : Latitudinal Control
		current_steer_angle = _lat_controller.run_step(cte)

		take FollowLaneAction(throttle=throttle,
		                      current_steer=current_steer_angle,
		                      past_steer=past_steer_angle)
		past_steer_angle = current_steer_angle


behavior FollowTrajectoryBehavior(target_speed = 25, trajectory = None):
	assert trajectory is not None

	trajectory_line = concatenateCenterlines(trajectory)

	# instantiate longitudinal and latitudinal pid controllers
	dt = simulation().timestep
	_lon_controller = actions.PIDLongitudinalController(dt=dt)
	_lat_controller = actions.PIDLateralController(dt=dt)
	past_steer_angle = 0

	while True:
		cte = trajectory_line.signedDistanceTo(self.position)
		speed_error = target_speed - self.speed

		# compute throttle : Longitudinal Control
		throttle = _lon_controller.run_step(speed_error)

		# compute steering : Latitudinal Control
		current_steer_angle = _lat_controller.run_step(cte)

		take FollowLaneAction(throttle=throttle,
		                      current_steer=current_steer_angle,
		                      past_steer=past_steer_angle)
		past_steer_angle = current_steer_angle

