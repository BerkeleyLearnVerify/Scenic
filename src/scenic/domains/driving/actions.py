"""Actions for dynamic agents in the driving domain."""

import math

from scenic.core.vectors import Vector
from scenic.core.simulators import Action
import scenic.domains.driving.model as _model

## Actions available to all agents

class SetPositionAction(Action):
	def __init__(self, pos):
		self.pos = pos

	def applyTo(self, obj, sim):
		obj.setPosition(self.pos, obj.elevation)

class OffsetAction(Action):
	"""Teleports actor forward (in direction of its heading) by some offset."""
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)
		obj.setPosition(pos, obj.elevation)

class SetVelocityAction(Action):
	def __init__(self, xVel, yVel, zVel=0):
		self.velocity = (xVel, yVel, zVel)

	def applyTo(self, obj, sim):
		obj.setVelocity(self.velocity)

class SetSpeedAction(Action):
	def __init__(self, speed):
		self.speed = speed

	def applyTo(self, obj, sim):
		vel = Vector(0, self.speed).rotatedBy(obj.heading)
		obj.setVelocity(vel)

## Actions available to vehicles which can steer

class SteeringAction(Action):
	def canBeTakenBy(self, agent):
		return isinstance(agent, _model.Steers)

class SetThrottleAction(SteeringAction):
	def __init__(self, throttle):
		if not 0.0 <= throttle <= 1.0:
			raise RuntimeError('Throttle must be a float in range [0.0, 1.0].')
		self.throttle = throttle

	def applyTo(self, obj, sim):
		obj.setThrottle(self.throttle)

class SetSteerAction(SteeringAction):	# TODO rename?
	def __init__(self, steer):
		if not -1.0 <= steer <= 1.0:
			raise RuntimeError('Steer must be a float in range [-1.0, 1.0].')
		self.steer = steer

	def applyTo(self, obj, sim):
		obj.setSteering(self.steer)

class SetBrakeAction(SteeringAction):
	def __init__(self, brake):
		if not 0.0 <= brake <= 1.0:
			raise RuntimeError('Brake must be a float in range [0.0, 1.0].')
		self.brake = brake

	def applyTo(self, obj, sim):
		obj.setBraking(self.brake)

class SetHandBrakeAction(SteeringAction):	# TODO rename?
	def __init__(self, handBrake):
		if not isinstance(handBrake, bool):
			raise RuntimeError('Hand brake must be a boolean.')
		self.handbrake = handbrake

	def applyTo(self, obj, sim):
		obj.setHandbrake(self.handbrake)

class SetReverseAction(SteeringAction):
	def __init__(self, reverse):
		if not isinstance(reverse, bool):
			raise RuntimeError('Reverse must be a boolean.')
		self.reverse = reverse

	def applyTo(self, obj, sim):
		obj.setReverse(self.reverse)

class FollowLaneAction(SteeringAction):	# TODO rename this!
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
		if self.throttle > 0:
			throttle = min(self.throttle, self.max_throt)
			brake = 0
		else:
			throttle = 0
			brake = min(abs(self.throttle), self.max_brake)

		# Steering regulation: changes cannot happen abruptly, can't steer too much.

		if self.current_steer > self.past_steer + 0.1:
			self.current_steer = self.past_steer + 0.1
		elif self.current_steer < self.past_steer - 0.1:
			self.current_steer = self.past_steer - 0.1

		if self.current_steer >= 0:
			steering = min(self.max_steer, self.current_steer)
		else:
			steering = max(-self.max_steer, self.current_steer)

		obj.setThrottle(throttle)
		obj.setBraking(brake)
		obj.setSteering(steering)

## Actions available to agents that can walk

class WalkingAction(Action):
	def canBeTakenBy(self, agent):
		return isinstance(agent, _model.Walks)

class SetWalkingDirectionAction(WalkingAction):
	def __init__(self, heading):
		self.heading = heading

	def applyTo(self, obj, sim):
		obj.setWalkingDirection(self.heading)

class SetWalkingSpeedAction(WalkingAction):
	def __init__(self, speed):
		self.speed = speed

	def applyTo(self, obj, sim):
		obj.setWalkingSpeed(self.speed)

class SetRelativeDirectionAction(WalkingAction):		# TODO eliminate
	"""Offsets direction counterclockwise relative to walker's forward vector."""

	def __init__(self, offset, degrees=False):
		self.offset = math.radians(offset) if degrees else offset

	def applyTo(self, obj, sim):
		heading = obj.heading + self.offset
		obj.setWalkingDirection(heading)
