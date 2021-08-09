"""Low-level controllers useful for vehicles.

The Lateral/Longitudinal PID controllers are adapted from `CARLA`_'s PID controllers,
which are licensed under the following terms:

	Copyright (c) 2018-2020 CVC.

	This work is licensed under the terms of the MIT license.
	For a copy, see <https://opensource.org/licenses/MIT>.


.. _CARLA: https://carla.org/
"""

from collections import deque

import numpy as np

class PIDLongitudinalController:
	"""Longitudinal control using a PID to reach a target speed.

	Arguments:
		K_P: Proportional gain
		K_D: Derivative gain
		K_I: Integral gain
		dt: time step
	"""
	def __init__(self, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
		self._k_p = K_P
		self._k_d = K_D
		self._k_i = K_I
		self._dt = dt
		self._error_buffer = deque(maxlen=10)

	def run_step(self, speed_error):
		"""Estimate the throttle/brake of the vehicle based on the PID equations.

		Arguments:
			speed_error: target speed minus current speed

		Returns:
			a signal between -1 and 1, with negative values indicating braking.
		"""
		error = speed_error
		self._error_buffer.append(error)

		if len(self._error_buffer) >= 2:
			_de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
			_ie = sum(self._error_buffer) * self._dt
		else:
			_de = 0.0
			_ie = 0.0

		return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie),
		               -1.0, 1.0)

class PIDLateralController:
	"""Lateral control using a PID to track a trajectory.

	Arguments:
		K_P: Proportional gain
		K_D: Derivative gain
		K_I: Integral gain
		dt: time step
	"""
	def __init__(self, K_P=0.3, K_D=0.2, K_I=0, dt=0.1):
		self.Kp = K_P
		self.Kd = K_D
		self.Ki = K_I
		self.PTerm = 0
		self.ITerm = 0
		self.DTerm = 0
		self.dt = dt
		self.last_error = 0
		self.windup_guard = 20.0
		self.output = 0

	def run_step(self, cte):
		"""Estimate the steering angle of the vehicle based on the PID equations.

		Arguments:
			cte: cross-track error (distance to right of desired trajectory)

		Returns:
			a signal between -1 and 1, with -1 meaning maximum steering to the left.
		"""
		error = cte
		delta_error = error - self.last_error
		self.PTerm = self.Kp * error
		self.ITerm += error * self.dt

		if (self.ITerm < -self.windup_guard):
			self.ITerm = -self.windup_guard
		elif (self.ITerm > self.windup_guard):
			self.ITerm = self.windup_guard

		self.DTerm = delta_error / self.dt

		# Remember last error for next calculation
		self.last_error = error

		self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

		return np.clip(self.output, -1, 1)
