
import scenic.simulators.carla.actions as actions
from scenic.simulators.carla.model import roadDirection
from scenic.simulators.domains.driving.roads import ManeuverType
from scenic.core.regions import regionFromShapelyObject
from shapely.geometry import LineString
import math

def concatenateCenterlines(centerlines=[]):

	line = []
	if centerlines != []:
		for centerline in centerlines:
			for point in centerline:
				if point not in line:
					line.append(point)

	return regionFromShapelyObject(LineString(line))

def distance(pos1, pos2):
	""" pos1, pos2 = (x,y) """
	return math.sqrt(math.pow(pos1[0]-pos2[0],2) + math.pow(pos1[1]-pos2[1],2))

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


behavior FollowLaneBehavior(target_speed = 25, network = None):
	assert network is not None

	# instantiate longitudinal and latitudinal pid controllers
	_lon_controller = actions.PIDLongitudinalController(self)
	_lat_controller = actions.PIDLateralController(self)
	past_steer_angle = 0

	while True:

		if self.speed is not None:
			current_speed = self.speed
		else:
			current_speed = 0
			print("self.speed is None")

		print("current_speed: ", self.speed)

		nearest_line_points = network.laneAt(self).centerline.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		speed_error = target_speed - current_speed

		# compute throttle : Longitudinal Control
		throttle = _lon_controller.run_step(speed_error)

		# compute steering : Latitudinal Control
		current_steer_angle = _lat_controller.run_step(cte)

		take actions.FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
		# take actions.FollowLaneAction(throttle=throttle, current_steer=0, past_steer=0)
		past_steer_angle = current_steer_angle

	

behavior FollowTrajectoryBehavior(target_speed = 25, trajectory = None):
	assert trajectory is not None

	trajectory_line = concatenateCenterlines(trajectory)

	# instantiate longitudinal and latitudinal pid controllers
	_lon_controller = actions.PIDLongitudinalController(self)
	_lat_controller = actions.PIDLateralController(self)
	past_steer_angle = 0

	while True:
		if self.speed is not None:
			current_speed = self.speed
		else:
			current_speed = 0
			print("self.speed is None")
		print("current_speed: ", self.speed)

		nearest_line_points = trajectory_line.nearestSegmentTo(self.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		cte = nearest_line_segment.signedDistanceTo(self.position)
		speed_error = target_speed - current_speed

		# compute throttle : Longitudinal Control
		throttle = _lon_controller.run_step(speed_error)

		# compute steering : Latitudinal Control
		current_steer_angle = _lat_controller.run_step(cte)

		take actions.FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
		# take actions.FollowLaneAction(throttle=throttle, current_steer=0, past_steer=0)
		past_steer_angle = current_steer_angle

