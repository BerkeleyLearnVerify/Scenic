"""Actions for dynamic agents in CARLO scenarios."""

import math as _math

import src.scenic.simulators.carlo.CARLO as _carlo

from scenic.domains.driving.actions import *
import scenic.simulators.carla.utils.utils as _utils

################################################
# Actions available to all movable carlo objects #
################################################

class SetControlAction(Action):
	def __init__(self, steering, acceleration):
		self.steering = steering
		self.acceleration = acceleration

	def canBeTakenBy(self, agent):
		return True if agent.movable else False

	def applyTo(self, agent, simulation):
		agent.set_control(self.steering, self.acceleration)