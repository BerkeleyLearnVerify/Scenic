
from collections import deque

import numpy as np
import scipy.linalg as linalg

class PIDLongitudinalController:
	"""
	PIDLongitudinalController implements longitudinal control using a PID.
	"""


	def __init__(self, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
		"""
		Constructor method.

			:param K_P: Proportional term
			:param K_D: Differential term
			:param K_I: Integral term
			:param dt: time differential in seconds
		"""
		self._k_p = K_P
		self._k_d = K_D
		self._k_i = K_I
		self._dt = dt
		self._error_buffer = deque(maxlen=10)
		#print("PIDLongitudinalController Instantiated")

	def run_step(self, speed_error):
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

class PIDLateralController:
	"""
	PIDLateralController implements lateral control using a PID.
	"""

	# def __init__(self, vehicle, K_P=0.01, K_D=0.000001, K_I=0.1, dt=0.1):
	def __init__(self, K_P=0.3, K_D=0.2, K_I=0, dt=0.1):
		"""
		Constructor method. 0.0000005

			:param vehicle: actor to apply to local planner logic onto
			:param K_P: Proportional term
			:param K_D: Differential term
			:param K_I: Integral term
			:param dt: time differential in seconds
		"""
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
		#print("PIDLateralController Instantiated")

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
		#print("cte: ", cte)

		return np.clip(self.output, -1, 1)

class LQR:
	def __init__(self,v_target, wheelbase, Q, R):
		self.v_target = v_target
		self.wheelbase = wheelbase
		self.Q = Q
		self.R = R

	def run_step(self):
		A = np.matrix([[0, self.v_target*(5./18.)], [0, 0]])
		B = np.matrix([[0], [(self.v_target/self.wheelbase)*(5./18.)]])
		V = np.matrix(linalg.solve_continuous_are(A, B, self.Q, self.R))
		K = np.matrix(linalg.inv(self.R)*(B.T*V))
		return K