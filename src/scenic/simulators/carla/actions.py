import math
import carla

import scenic.simulators as simulators
import scenic.simulators.carla.utils.utils as utils


################################################
# Actions available to all carla.Actor objects #
################################################

# NOTE: Equivalent to LGSVL's MoveAction class
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
		self.pos = pos  # Scenic position

	def applyTo(self, obj, carlaActor, sim):
		loc = utils.scenicToCarlaLocation(self.pos, z=obj.elevation)
		carlaActor.set_location(loc)


class SetVelocityAction(simulators.Action):
	def __init__(self, velocity):
		super().__init__()
		self.velocity = velocity

	def applyTo(self, obj, carlaActor, sim):
		currYaw = utils.scenicToCarlaRotation(obj.heading).yaw
		xVel = self.velocity * math.cos(currYaw)
		yVel = self.velocity * math.sin(currYaw)
		newVel = utils.scalarToCarlaVector3D(xVel, yVel)
		carlaActor.set_velocity(newVel)


class SetAngularVelocityAction(simulators.Action):
	def __init__(self, angularVelocity):
		super().__init__()
		self.angularVelocity = angularVelocity

	def applyTo(self, obj, carlaActor, sim):
		xAngularVel = self.angularVelocity * math.cos(obj.heading)
		yAngularVel = self.angularVelocity * math.sin(obj.heading)
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
		super().__init__()
		self.throttle = throttle  # float in range [0.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.throttle = min(max(self.throttle, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class IncreaseThrottleAction(simulators.Action):
	def __init__(self, increment):
		super().__init__()
		self.increment = increment  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle + self.increment, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class DecreaseThrottleAction(simulators.Action):
	def __init__(self, decrement):
		super().__init__()
		self.decrement = decrement  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle - self.decrement, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class SetSteerAction(simulators.Action):
	def __init__(self, steer):
		super().__init__()
		self.steer = steer  # float in range [-1.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.steer = min(max(self.steer, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class IncreaseSteerAction(simulators.Action):
	def __init__(self, increment):
		super().__init__()
		self.increment = increment  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.steer = min(max(ctrl.steer + self.increment, -1), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class DecreaseSteerAction(simulators.Action):
	def __init__(self, decrement):
		super().__init__()
		self.decrement = decrement  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.steer = min(max(ctrl.steer - self.decrement, -1), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class AlignSteerToLaneAction(simulators.Action):
	''' Sets steer to match lane heading '''

	def __init__(self):
		super().__init__()

	def applyTo(self, obj, vehicle, sim):
		# Compute heading's deviation from lane direction
		map_ = sim.world.get_map()
		# TODO: Finish implementation


class SetBrakeAction(simulators.Action):
	def __init__(self, brake):
		super().__init__()
		self.brake = brake  # float in range [0.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.brake = min(max(self.brake, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class IncreaseBrakeAction(simulators.Action):
	def __init__(self, increment):
		super().__init__()
		self.increment = increment  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle + self.increment, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class DecreaseBrakeAction(simulators.Action):
	def __init__(self, decrement):
		super().__init__()
		self.decrement = decrement  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle - self.decrement, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class SetHandBrakeAction(simulators.Action):
	def __init__(self, handBrake):
		super().__init__()
		self.handBrake = handBrake  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.hand_brake = self.handBrake
		vehicle.apply_control(ctrl)


class SetReverseAction(simulators.Action):
	def __init__(self, reverse):
		super().__init__()
		self.reverse = reverse  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.reverse = self.reverse
		vehicle.apply_control(ctrl)


class SetManualGearShiftAction(simulators.Action):
	def __init__(self, manualGearShift):
		super().__init__()
		self.manualGearShift = manualGearShift  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		vehicle.apply_control(ctrl)


class SetGearAction(simulators.Action):
	def __init__(self, gear):
		super().__init__()
		self.gear = gear  # int in range [1, 6]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = min(max(self.gear, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class IncreaseGearAction(simulators.Action):
	def __init__(self, increment=1):
		super().__init__()
		self.increment = increment  # should be 1

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctr.gear = min(max(ctr.gear + self.increment, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)


class DecreaseGearAction(simulators.Action):
	def __init__(self, decrement=1):
		super().__init__()
		self.decrement = decrement  # should be 1

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = min(max(ctrl.gear - self.decrement, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)
