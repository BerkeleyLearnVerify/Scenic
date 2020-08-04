# added equivalent actions from carla, for lgsvl

import math
import lgsvl
import numpy as np
from scipy import linalg
import scenic.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
import scenic.syntax.veneer as veneer
from scenic.core.vectors import Vector
from collections import deque
import time


class SetThrottleAction(simulators.Action):
	def __init__(self, throttle):
		self.throttle = throttle

	def applyTo(self, obj, lgsvlObject, sim):
		cntrl = lgsvl.VehicleControl()
		cntrl.throttle = self.throttle
		lgsvlObject.apply_control(cntrl, True)


class SetBrakeAction(simulators.Action):
	def __init__(self, brake):
		self.brake = brake

	def applyTo(self, obj, lgsvlObject, sim):
		cntrl = lgsvl.VehicleControl()
		cntrl.brake = self.brake
		cntrl.throttle = 0
		lgsvlObject.apply_control(cntrl, True)

class SetSteerAction(simulators.Action):
	def __init__(self, steer):
		self.steer = steer

	def applyTo(self, obj, lgsvlObject, sim):
		cntrl = lgsvl.VehicleControl()
		cntrl.steer = self.steer
		lgsvlObject.apply_control(cntrl, True)

class SetReverse(simulators.Action):
	def __init__(self, steer):
		self.reverse = reverse

	def applyTo(self, obj, lgsvlObject, sim):
		cntrl = lgsvl.VehicleControl()
		cntrl.reverse = self.reverse
		lgsvlObject.apply_control(cntrl, True)

class MoveAction(simulators.Action):
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, lgsvlObject, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)
		pos = utils.scenicToLGSVLPosition(pos, y=obj.elevation)
		state = lgsvlObject.state
		state.transform.position = pos
		lgsvlObject.state = state

class SetVelocityAction(simulators.Action):
	def __init__(self, velocity):
		self.velocity = utils.scenicToLGSVLPosition(velocity)

	def applyTo(self, obj, lgsvlObject, sim):
		state = lgsvlObject.state
		state.velocity = self.velocity
		lgsvlObject.state = state

class SetSpeedAction(simulators.Action):
	def __init__(self, speed):
		self.speed = speed

	def applyTo(self, obj, lgsvlObject, sim):
		vel = Vector(0, self.speed).rotatedBy(obj.heading)
		velocity = utils.scenicToLGSVLPosition(vel)
		state = lgsvlObject.state
		state.velocity = velocity
		lgsvlObject.state = state



class FollowWaypointsAction(simulators.Action):
	def __init__(self, waypoints):
		self.waypoints = tuple(waypoints)
		if not isinstance(self.waypoints[0], lgsvl.DriveWaypoint):
			pts = []
			for wp in self.waypoints:
				elev = veneer.simulation().groundElevationAt(wp.position)
				pos = utils.scenicToLGSVLPosition(wp.position, y=elev)
				rot = utils.scenicToLGSVLRotation(wp.heading)
				pt = lgsvl.DriveWaypoint(pos, wp.speed, rot)
				pts.append(pt)
			self.waypoints = tuple(pts)

		self.lastTime = -2

	def applyTo(self, obj, lgsvlObject, sim):
		#print(sim.currentTime, self.lastTime)
		if sim.currentTime is not self.lastTime + 1:
			agentType = obj.lgsvlAgentType
			if agentType in (lgsvl.AgentType.NPC, lgsvl.AgentType.PEDESTRIAN):
				lgsvlObject.follow(self.waypoints)
			else:
				raise RuntimeError('used FollowWaypointsAction with'
								   f' unsupported agent {lgsvlObject}')
		self.lastTime = sim.currentTime

class CancelWaypointsAction(simulators.Action):
	def applyTo(self, obj, lgsvlObject, sim):
		lgsvlObject.walk_randomly(False)

class SetDestinationAction(simulators.Action):
	def __init__(self, dest):
		self.dest = dest
		self.timer = 0

	def applyTo(self, obj, lgsvlObject, sim):
		if self.timer == 0:
			print('Setting destination...')
			z = sim.groundElevationAt(self.dest)
			import dreamview
			obj.dreamview.setDestination(self.dest.x, self.dest.y, z,
									  coordType=dreamview.CoordType.Unity)

		# push vehicle for 1 second to start
		oneSec = int(1.0/sim.timeStep)
		if self.timer < oneSec:
			cntrl = lgsvl.VehicleControl()
			cntrl.throttle = 0.5
			lgsvlObject.apply_control(cntrl, True)
		elif self.timer == oneSec:
			print('Autopilot...')
			cntrl = lgsvl.VehicleControl()
			cntrl.throttle = 0.5
			lgsvlObject.apply_control(cntrl, False)
		self.timer = self.timer + 1




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

	def applyTo(self, obj, lgsvlObject, sim):
		"""
		Execute one step of control invoking both lateral and longitudinal
		PID controllers to reach a target waypoint
		at a given target_speed.

			:param target_speed: desired vehicle speed
			:param waypoint: target location encoded as a waypoint
			:return: distance (in meters) to the waypoint
		"""
	
		control = lgsvl.VehicleControl()

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
		lgsvlObject.apply_control(control, True)



class TrackWaypoints(simulators.Action):
	def __init__(self, waypoints, cruising_speed = 10):
		self.waypoints = np.array(waypoints)
		self.curr_index = 1
		self.cruising_speed = cruising_speed

	def LQR(v_target, wheelbase, Q, R):
		A = np.matrix([[0, v_target*(5./18.)], [0, 0]])
		B = np.matrix([[0], [(v_target/wheelbase)*(5./18.)]])
		V = np.matrix(linalg.solve_continuous_are(A, B, Q, R))
		K = np.matrix(linalg.inv(R)*(B.T*V))
		return K

	def applyTo(self, obj, lgsvlObject, sim):
		state = lgsvlObject.state
		pos = state.transform.position
		rot = state.transform.rotation
		velocity = state.velocity
		th, x, y, v = rot.y/180.0*np.pi, pos.x, pos.z, (velocity.x**2 + velocity.z**2)**0.5
		#print('state:', th, x, y, v)
		PREDICTIVE_LENGTH = 3
		MIN_SPEED = 1
		WHEEL_BASE = 3
		v = max(MIN_SPEED, v)

		x = x + PREDICTIVE_LENGTH * np.cos(-th+np.pi/2)
		y = y + PREDICTIVE_LENGTH * np.sin(-th+np.pi/2)
		#print('car front:', x, y)
		dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
		dist_pos = np.argpartition(dists,1)
		index = dist_pos[0]
		if index > self.curr_index and index < len(self.waypoints)-1:
			self.curr_index = index
		p1, p2, p3 = self.waypoints[self.curr_index-1], self.waypoints[self.curr_index], self.waypoints[self.curr_index+1]

		p1_a = np.linalg.norm(p1 - np.array([x, y]))
		p3_a = np.linalg.norm(p3 - np.array([x, y]))
		p1_p2= np.linalg.norm(p1 - p2)
		p3_p2= np.linalg.norm(p3 - p2)

		if p1_a - p1_p2 > p3_a - p3_p2:
			p1 = p2
			p2 = p3

		#print('points:',p1, p2)
		x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
		th_n = -math.atan2(y2-y1,x2-x1)+np.pi/2
		d_th = (th - th_n + 3*np.pi) % (2*np.pi) - np.pi
		d_x = (x2-x1)*y - (y2-y1)*x + y2*x1 - y1*x2
		d_x /= np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
		#print('d_th, d_x:',d_th, d_x)


		K = TrackWaypoints.LQR(v, WHEEL_BASE, np.array([[1, 0], [0, 3]]), np.array([[10]]))
		u = -K * np.matrix([[-d_x], [d_th]])
		u = np.double(u)
		u_steering = min(max(u, -1), 1)

		K = 1
		u = -K*(v - self.cruising_speed)
		u_thrust = min(max(u, -1), 1)

		#print('u:', u_thrust, u_steering)

		cntrl = lgsvl.VehicleControl()
		cntrl.steering = u_steering
		if u_thrust > 0:
			cntrl.throttle = u_thrust
		elif u_thrust < 0.1:
			cntrl.braking = -u_thrust
		lgsvlObject.apply_control(cntrl, True)


# ----- hmm


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