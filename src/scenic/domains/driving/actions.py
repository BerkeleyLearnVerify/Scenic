"""Actions for dynamic agents in the driving domain.

These actions are automatically imported when using the driving domain.

The `RegulatedControlAction` is based on code from the `CARLA`_ project, licensed under
the following terms:

	Copyright (c) 2018-2020 CVC.

	This work is licensed under the terms of the MIT license.
	For a copy, see <https://opensource.org/licenses/MIT>.

.. _CARLA: https://carla.org/
"""

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
	"""Set the throttle."""
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

class RegulatedControlAction(SteeringAction):
	"""Regulated control of throttle, braking, and steering.

	Controls throttle and braking using one signal that may be positive or negative.
	Useful with simple controllers that output a single value.
	"""

	def __init__(self, throttle, steer, past_steer,
				 max_throttle=0.5, max_brake=0.5, max_steer=0.8):
		if throttle > 0:
			throttle = min(throttle, max_throttle)
			brake = 0
		else:
			throttle = 0
			brake = min(abs(throttle), max_brake)

		# Steering regulation: changes cannot happen abruptly, can't steer too much.

		if steer > past_steer + 0.1:
			steer = past_steer + 0.1
		elif steer < past_steer - 0.1:
			steer = past_steer - 0.1

		if steer >= 0:
			steer = min(max_steering, steer)
		else:
			steer = max(-max_steering, steer)

		self.throttle, self.brake, self.steer = throttle, brake, steer

	def applyTo(self, obj, sim):
		obj.setThrottle(self.throttle)
		obj.setBraking(self.brake)
		obj.setSteering(self.steer)

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
