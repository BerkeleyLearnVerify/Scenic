import math

import carla
import numpy as np

from scenic.core.simulators import Action
import scenic.simulators.carla.utils.utils as utils
from scenic.simulators.carla.misc import get_speed

################################################
# Actions available to all carla.Actor objects #
################################################

class OffsetAction(Action):
	"""Teleports actor forward (in direction of its heading) by some offset."""
	
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		obj.carlaActor.set_location(loc)


class SetLocationAction(Action):
	def __init__(self, pos):
		self.pos = pos

	def applyTo(self, obj, sim):
		loc = utils.scenicToCarlaLocation(self.pos, z=obj.elevation)
		obj.carlaActor.set_location(loc)


class SetVelocityAction(Action):
	def __init__(self, xVel, yVel, zVel=0):
		self.xVel = xVel
		self.yVel = yVel
		self.zVel = zVel

	def applyTo(self, obj, sim):
		newVel = utils.scalarToCarlaVector3D(xVel, yVel, zVel)
		obj.carlaActor.set_velocity(newVel)


class SetSpeedAction(Action):
	def __init__(self, speed):
		self.speed = speed

	def applyTo(self, obj, sim):
		newVel = utils.scenicSpeedToCarlaVelocity(speed, obj.carlaActor.heading)
		obj.carlaActor.set_velocity(newVel)


class SetAngularVelocityAction(Action):
	def __init__(self, angularVel):
		self.angularVel = angularVel

	def applyTo(self, obj, sim):
		xAngularVel = self.angularVel * math.cos(obj.heading)
		yAngularVel = self.angularVel * math.sin(obj.heading)
		newAngularVel = utils.scalarToCarlaVector3D(xAngularVel, yAngularVel)
		obj.carlaActor.set_angular_velocity(newAngularVel)


class SetTransformAction(Action):
	def __init__(self, pos, heading):
		self.pos = pos
		self.heading = heading

	def applyTo(self, obj, sim):
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		rot = utils.scenicToCarlaRotation(heading)
		transform = carla.Transform(loc, rot)
		obj.carlaActor.set_transform(transform)


#############################################
# Actions specific to carla.Vehicle objects #
#############################################

class VehicleAction(Action):
	def canBeTakenBy(self, agent):
        return agent.blueprint.startswith('vehicle.')


class SetThrottleAction(VehicleAction):
	def __init__(self, throttle):
		if not 0.0 <= throttle <= 1.0:
			raise RuntimeError('Throttle must be a float in range [0.0, 1.0].')
		self.throttle = throttle

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.throttle = self.throttle
		vehicle.apply_control(ctrl)


class SetSteerAction(VehicleAction):
	def __init__(self, steer):
		if not -1.0 <= steer <= 1.0:
			raise RuntimeError('Steer must be a float in range [-1.0, 1.0].')
		self.steer = steer

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.steer = self.steer
		vehicle.apply_control(ctrl)


class SetBrakeAction(VehicleAction):
	def __init__(self, brake):
		if not 0.0 <= brake <= 1.0:
			raise RuntimeError('Brake must be a float in range [0.0, 1.0].')
		self.brake = brake

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.throttle = 0
		ctrl.brake = self.brake
		vehicle.apply_control(ctrl)


class SetHandBrakeAction(VehicleAction):
	def __init__(self, handBrake):
		if not isinstance(handBrake, bool):
			raise RuntimeError('Hand brake must be a boolean.')
		self.handBrake = handBrake

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.hand_brake = self.handBrake
		vehicle.apply_control(ctrl)


class SetReverseAction(VehicleAction):
	def __init__(self, reverse):
		if not isinstance(reverse, bool):
			raise RuntimeError('Reverse must be a boolean.')
		self.reverse = reverse

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.reverse = self.reverse
		vehicle.apply_control(ctrl)


class SetManualGearShiftAction(VehicleAction):
	def __init__(self, manualGearShift):
		if not isinstance(manualGearShift, bool):
			raise RuntimeError('Manual gear shift must be a boolean.')
		self.manualGearShift = manualGearShift

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		vehicle.apply_control(ctrl)


class SetGearAction(VehicleAction):
	def __init__(self, gear):
		if not isinstance(gear, int):
			raise RuntimeError('Gear must be an int.')
		self.gear = gear

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.gear = self.gear
		vehicle.apply_control(ctrl)


class SetManualFirstGearShiftAction(VehicleAction):
	def applyTo(self, obj, sim):
		ctrl = carla.VehicleControl(manual_gear_shift=True, gear=1)
		obj.carlaActor.apply_control(ctrl)


class FollowLaneAction(VehicleAction):
	"""
	VehiclePIDController is the combination of two PID controllers
	(lateral and longitudinal) to perform the
	low level control a vehicle from client side
	"""

	def __init__(self, throttle, current_steer, past_steer,
	             max_throttle=0.5, max_brake=0.5, max_steering=0.8):
		"""
		Constructor method.

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

		self.max_brake = max_brake
		self.max_throt = max_throttle
		self.max_steer = max_steering
		self.throttle = throttle
		self.current_steer = current_steer
		self.past_steer = past_steer

	def applyTo(self, obj, sim):
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

		#print("steer: ", steering)
		#print("throttle: ", control.throttle)
		#print("brake: ", control.brake)
		control.steer = steering
		control.hand_brake = False
		control.manual_gear_shift = False
		obj.carlaActor.apply_control(control)


#################################################
# Actions available to all carla.Walker objects #
#################################################

class WalkerAction(Action):
	def canBeTakenBy(self, agent):
        return agent.blueprint.startswith('walker.')


class SetRelativeDirectionAction(WalkerAction):
	"""Offsets direction counterclockwise relative to walker's forward vector."""

	def __init__(self, offset, degrees=False):
		self.offset = math.radians(offset) if degrees else offset

	def applyTo(self, obj, sim):
		walker = obj.carlaActor
		ctrl = walker.get_control()
		currDir = ctrl.direction
		sinOffset, cosOffset = math.cos(self.offset), math.sin(self.offset)
		newX = currDir.x * cosOffset - currDir.y * sinOffset
		newY = currDir.x * sinOffset + currDir.y * cosOffset
		ctrl.direction = utils.scalarToCarlaVector3D(newX, newY, currDir.z)
		walker.apply_control(ctrl)


class SetSpeedAction(WalkerAction):
	def __init__(self, speed):
		if not speed >= 0.0:
			raise RuntimeError('Speed must be a non-negative float.')
		self.speed = speed

	def applyTo(self, obj, sim):
		walker = obj.carlaActor
		ctrl = walker.get_control()
		ctrl.speed = self.speed
		walker.apply_control(ctrl)


class SetJumpAction(WalkerAction):
	def __init__(self, jump):
		if not isinstance(jump, bool):
			raise RuntimeError('Jump must be a boolean.')
		self.jump = jump

	def applyTo(self, obj, sim):
		walker = obj.carlaActor
		ctrl = walker.get_control()
		ctrl.jump = self.jump
		walker.apply_control(ctrl)
