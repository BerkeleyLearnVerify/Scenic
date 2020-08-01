# added equivalent actions from carla, for lgsvl

import math
import lgsvl
import numpy as np
from scipy import linalg
import scenic.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
import scenic.syntax.veneer as veneer
from scenic.core.vectors import Vector


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
		ctrl.reverse = self.reverse
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
