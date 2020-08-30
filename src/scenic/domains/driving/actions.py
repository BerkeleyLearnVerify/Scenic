"""Actions for dynamic agents in the driving domain."""

import math
import numpy as np
from scenic.core.vectors import Vector
from scenic.core.simulators import Action
import scenic.domains.driving.model as drivingModel
import scenic.domains.driving.controllers as controllers


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
		return isinstance(agent, drivingModel.Steers)

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
		return isinstance(agent, drivingModel.Walks)

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

class TrackWaypointsAction(Action):
	def __init__(self, waypoints, cruising_speed = 10):
		self.waypoints = np.array(waypoints)
		self.curr_index = 1
		self.cruising_speed = cruising_speed

	def canBeTakenBy(self, agent):
		# return agent.lgsvlAgentType is lgsvl.AgentType.EGO
		return True

	# def LQR(v_target, wheelbase, Q, R):
	# 	A = np.matrix([[0, v_target*(5./18.)], [0, 0]])
	# 	B = np.matrix([[0], [(v_target/wheelbase)*(5./18.)]])
	# 	V = np.matrix(linalg.solve_continuous_are(A, B, Q, R))
	# 	K = np.matrix(linalg.inv(R)*(B.T*V))
	# 	return K

	def applyTo(self, obj, sim):
		carlaObj = obj.carlaActor
		transform = carlaObj.get_transform()
		pos = transform.location
		rot = transform.rotation
		velocity = carlaObj.get_velocity()
		th, x, y, v = rot.yaw/180.0*np.pi, pos.x, pos.z, (velocity.x**2 + velocity.z**2)**0.5
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

		lqr = controllers.LQR(v_target=v, wheelbase=WHEEL_BASE, Q=np.array([[1, 0], [0, 3]]), R=np.array([[10]]))
		K = lqr.run_step()
		u = -K * np.matrix([[-d_x], [d_th]])
		u = np.double(u)
		u_steering = min(max(u, -1), 1)

		K = 1
		u = -K*(v - self.cruising_speed)
		u_thrust = min(max(u, -1), 1)

		#print('u:', u_thrust, u_steering)

		if u_thrust > 0:
			obj.setThrottle(u_thrust)
		elif u_thrust < 0.1:
			obj.setThrottle(-u_thrust)

		obj.setSteering(u_steering)