import math
import carla

import scenic.simulators as simulators
import scenic.simulators.carla.utils.utils as utils


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
