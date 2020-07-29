import math
import carla
import time

import scenic.simulators as simulators
import scenic.simulators.carla.utils.utils as utils

import numpy as np
from scenic.simulators.carla.misc import get_speed
from collections import deque

################################################
# Actions available to all carla.Actor objects #
################################################

class OffsetAction(simulators.Action):
	"""Teleports actor forward (in direction of its heading) by some offset."""
	
	def __init__(self, offset):
		super().__init__()
		self.offset = offset

	def applyTo(self, obj, carlaActor, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		carlaActor.set_location(loc)


class SetLocationAction(simulators.Action):
	def __init__(self, pos):
		super().__init__()
		self.pos = pos  # NOTE: Must translate to Carla coords

	def applyTo(self, obj, carlaActor, sim):
		loc = utils.scenicToCarlaLocation(self.pos, z=obj.elevation)
		carlaActor.set_location(loc)


class SetVelocityAction(simulators.Action):
	def __init__(self, xVel, yVel, zVel=0):
		super().__init__()
		self.xVel = xVel
		self.yVel = yVel
		self.zVel = zVel

	def applyTo(self, obj, carlaActor, sim):
		newVel = utils.scalarToCarlaVector3D(xVel, yVel, zVel)
		carlaActor.set_velocity(newVel) 


class SetSpeedAction(simulators.Action):
	def __init__(self, speed):
		super().__init__()
		self.speed = speed

	def applyTo(self, obj, carlaActor, sim):
		newVel = utils.scenicSpeedToCarlaVelocity(speed, carlaActor.heading)
		carlaActor.set_velocity(newVel)


class SetAngularVelocityAction(simulators.Action):
	def __init__(self, angularVel):
		super().__init__()
		self.angularVel = angularVel

	def applyTo(self, obj, carlaActor, sim):
		xAngularVel = self.angularVel * math.cos(obj.heading)
		yAngularVel = self.angularVel * math.sin(obj.heading)
		newAngularVel = utils.scalarToCarlaVector3D(xAngularVel, yAngularVel)
		carlaActor.set_angular_velocity(newAngularVel)


class SetTransformAction(simulators.Action):
	def __init__(self, pos, heading):
		super().__init__()
		self.pos = pos  # Scenic position
		self.heading = heading  # Scenic heading

	def applyTo(self, obj, carlaActor, sim):
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		rot = utils.scenicToCarlaRotation(heading)
		transform = carla.Transform(loc, rot)
		carlaActor.set_transform(transform)


#############################################
# Actions specific to carla.Vehicle objects #
#############################################

class SetThrottleAction(simulators.Action):
	def __init__(self, throttle):
		assert 0.0 <= throttle <= 1.0, \
			'Throttle must be a float in range [0.0, 1.0].'
		super().__init__()
		# print(self.throttle)
		self.throttle = throttle
		# print("The value of the throttle is: ", self.throttle)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.throttle = self.throttle
		vehicle.apply_control(ctrl)
		print("The applied throttle is: ", vehicle.get_control().throttle)


class SetSteerAction(simulators.Action):
	def __init__(self, steer):
		assert -1.0 <= steer <= 1.0, \
			'Steer must be a float in range [-1.0, 1.0].'
		super().__init__()
		self.steer = steer

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.steer = self.steer
		vehicle.apply_control(ctrl)


class AlignSteerToLaneAction(simulators.Action):
	'''Sets steer to match lane heading.'''

	def __init__(self):
		super().__init__()

	def applyTo(self, obj, vehicle, sim):
		# Compute heading's deviation from lane direction
		map_ = sim.world.get_map()
		# TODO: Finish implementation


class SetBrakeAction(simulators.Action):
	def __init__(self, brake):
		assert 0.0 <= brake <= 1.0, \
			'Brake must be a float in range [0.0, 1.0].'
		super().__init__()
		self.brake = brake

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.throttle = 0
		ctrl.brake = self.brake
		vehicle.apply_control(ctrl)


class SetHandBrakeAction(simulators.Action):
	def __init__(self, handBrake):
		assert isinstance(handBrake, bool), \
			'Hand brake must be a boolean.'
		super().__init__()
		self.handBrake = handBrake

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.hand_brake = self.handBrake
		vehicle.apply_control(ctrl)


class SetReverseAction(simulators.Action):
	def __init__(self, reverse):
		assert isinstance(reverse, bool), \
			'Reverse must be a boolean.'
		super().__init__()
		self.reverse = reverse

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.reverse = self.reverse
		vehicle.apply_control(ctrl)


class SetManualGearShiftAction(simulators.Action):
	def __init__(self, manualGearShift):
		assert isinstance(manualGearShift, bool), \
			'Manual gear shift must be a boolean.'
		super().__init__()
		self.manualGearShift = manualGearShift  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		vehicle.apply_control(ctrl)


class SetGearAction(simulators.Action):
	def __init__(self, gear):
		# TODO: assert statement
		super().__init__()
		self.gear = gear

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = self.gear
		vehicle.apply_control(ctrl)

class SetManualFirstGearShiftAction(simulators.Action):
	def __init__(self):
		super().__init__()
		self.manualGearShift = True  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		ctrl.gear = 1
		vehicle.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))


#################################################
# Actions available to all carla.Walker objects #
#################################################

class SetRelativeDirectionAction(simulators.Action):
	'''Offsets direction counterclockwise relative to walker's forward vector.'''

	def __init__(self, offset, degrees=False):
		super().__init__()
		self.offset = math.radians(offset) if degrees else offset

	def applyTo(self, obj, walker, sim):
		ctrl = walker.get_control()
		currDir = ctrl.direction
		sinOffset, cosOffset = math.cos(self.offset), math.sin(self.offset)
		newX = currDir.x * cosOffset - currDir.y * sinOffset
		newY = currDir.x * sinOffset + currDir.y * cosOffset
		ctrl.direction = utils.scalarToCarlaVector3D(newX, newY, currDir.z)
		walker.apply_control(ctrl)


class SetSpeedAction(simulators.Action):
	def __init__(self, speed):
		assert speed >= 0.0, \
			'Speed must be a non-negative float.'
		super().__init__()
		self.speed = speed  # float

	def applyTo(self, obj, walker, sim):
		ctrl = walker.get_control()
		ctrl.speed = self.speed
		walker.apply_control(ctrl)


class SetJumpAction(simulators.Action):
	def __init__(self, jump):
		assert isinstance(jump, bool), \
			'Jump must be a boolean.'
		super().__init__()
		self.jump = jump  # boolean

	def applyTo(self, obj, walker, sim):
		ctrl = walker.get_control()
		ctrl.jump = self.jump
		walker.apply_control(ctrl)



class FollowLaneAction(simulators.Action):
	"""
	VehiclePIDController is the combination of two PID controllers
	(lateral and longitudinal) to perform the
	low level control a vehicle from client side
	"""

	def __init__(self, throttle, current_steer, past_steer, args_lateral=None, args_longitudinal=None, max_throttle=0.5, max_brake=0.5, max_steering=0.8):
		"""
		Constructor method.

		:param vehicle: actor to apply to local planner logic onto
		:param args_lateral: dictionary of arguments to set the lateral PID controller
		using the following semantics:
			K_P -- Proportional term
			K_D -- Differential term
			K_I -- Integral term
		:param args_longitudinal: dictionary of arguments to set the longitudinal
		PID controller using the following semantics:
			K_P -- Proportional term
			K_D -- Differential term
			K_I -- Integral term
		"""
		super().__init__()

		self.max_brake = max_brake
		self.max_throt = max_throttle
		self.max_steer = max_steering
		self.args_longitudinal = args_longitudinal
		self.args_lateral = args_lateral
		self.throttle = throttle
		self.current_steer = current_steer
		self.past_steer = past_steer

	def applyTo(self, obj, vehicle, sim):
		"""
		Execute one step of control invoking both lateral and longitudinal
		PID controllers to reach a target waypoint
		at a given target_speed.

			:param target_speed: desired vehicle speed
			:param waypoint: target location encoded as a waypoint
			:return: distance (in meters) to the waypoint
		"""
		# past_steering = vehicle.get_control().steer

		# if self.args_longitudinal!=None:
		# 	_lon_controller = PIDLongitudinalController(vehicle, **self.args_longitudinal)
		# else:
		# 	_lon_controller = PIDLongitudinalController(vehicle)

		# if self.args_lateral!=None:
		# 	_lat_controller = PIDLateralController(vehicle, **self.args_lateral)
		# else: 
		# 	_lat_controller = PIDLateralController(vehicle)

		# acceleration = _lon_controller.run_step(self.target_speed)
		# current_steering = _lat_controller.run_step(self.cte)

		control = carla.VehicleControl()

		if self.throttle >= 0.0:
			control.throttle = min(self.throttle, self.max_throt)
			control.brake = 0.0
		else:
			control.throttle = 0.0
			control.brake = min(abs(self.throttle), self.max_brake)

		# Steering regulation: changes cannot happen abruptly, can't steer too much.

		if self.current_steer > self.past_steer + 0.1:
		    self.current_steer = self.past_steer + 0.1
		elif self.current_steer < self.past_steer - 0.1:
		    self.current_steer = self.past_steer - 0.1

		if self.current_steer >= 0:
		    steering = min(self.max_steer, self.current_steer)
		else:
		    steering = max(-self.max_steer, self.current_steer)

		print("steer: ", steering)
		print("throttle: ", control.throttle)
		print("brake: ", control.brake)
		control.steer = steering
		control.hand_brake = False
		control.manual_gear_shift = False
		vehicle.apply_control(control)


class PIDLongitudinalController():
	"""
	PIDLongitudinalController implements longitudinal control using a PID.
	"""


	def __init__(self, vehicle, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
		"""
		Constructor method.

			:param vehicle: actor to apply to local planner logic onto
			:param K_P: Proportional term
			:param K_D: Differential term
			:param K_I: Integral term
			:param dt: time differential in seconds
		"""
		self.vehicle = vehicle
		self._k_p = K_P
		self._k_d = K_D
		self._k_i = K_I
		self._dt = dt
		self._error_buffer = deque(maxlen=10)
		print("PIDLongitudinalController Instantiated")

	def run_step(self, speed_error, debug=False):
		"""
		Execute one step of longitudinal control to reach a given target speed.
		Estimate the throttle/brake of the vehicle based on the PID equations
			:param speed_error:  target speed - current speed (in Km/h)
			:return: throttle/brake control
		"""
		error = speed_error
		self._error_buffer.append(error)

		if len(self._error_buffer) >= 2:
			_de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
			_ie = sum(self._error_buffer) * self._dt
		else:
			_de = 0.0
			_ie = 0.0

		return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController():
	"""
	PIDLateralController implements lateral control using a PID.
	"""

	# def __init__(self, vehicle, K_P=0.01, K_D=0.000001, K_I=0.1, dt=0.1):
	def __init__(self, vehicle, K_P=0.01, K_D=0.01, K_I=0, dt=0.1):
		"""
		Constructor method. 0.0000005

			:param vehicle: actor to apply to local planner logic onto
			:param K_P: Proportional term
			:param K_D: Differential term
			:param K_I: Integral term
			:param dt: time differential in seconds
		"""
		self.vehicle = vehicle
		self.Kp = K_P
		self.Kd = K_D
		self.Ki = K_I
		self.PTerm = 0
		self.ITerm = 0
		self.DTerm = 0
		self.sample_time = dt
		self.last_error = 0
		self.windup_guard = 20.0
		self.current_time = time.time()
		self.last_time = self.current_time
		self.output = 0
		print("PIDLateralController Instantiated")

	def run_step(self, cte):
		"""
		Execute one step of lateral control to steer
		the vehicle towards a certain waypoin.

			:param waypoint: target waypoint
			:return: steering control in the range [-1, 1] where:
			-1 maximum steering to left
			+1 maximum steering to right
		"""
		# return self._pid_control(waypoint, self.vehicle.get_transform())
		return self._pid_control(cte)

	def _pid_control(self, cte):
		"""
		Estimate the steering angle of the vehicle based on the PID equations

		    :param waypoint: target waypoint
		    :param vehicle_transform: current transform of the vehicle
		    :return: steering control in the range [-1, 1]
		"""

		# define Centerline-Tracking-Error (CTE):
		error = cte

		self.current_time = time.time()
		delta_time = self.current_time - self.last_time
		delta_error = error - self.last_error
		print("delta_time: ", delta_time)

		# if (delta_time >= self.sample_time):
		self.PTerm = self.Kp * error
		self.ITerm += error * delta_time

		if (self.ITerm < -self.windup_guard):
			self.ITerm = -self.windup_guard
		elif (self.ITerm > self.windup_guard):
			self.ITerm = self.windup_guard

		self.DTerm = 0.0
		if delta_time > 0:
			self.DTerm = delta_error / delta_time

		# Remember last time and last error for next calculation
		self.last_time = self.current_time
		self.last_error = error

		self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
		print("cte: ", cte)

		return np.clip(self.output, -1, 1)