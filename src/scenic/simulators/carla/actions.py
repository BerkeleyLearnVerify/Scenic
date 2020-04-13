import carla
import scenic.simulators as simulators
import scenic.simulators.carla.utils as utils

################################################
# Actions available to all carla.Actor objects #
################################################

# NOTE: Equivalent to LGSVL's MoveAction class
class OffsetAction(simulators.Action):
	''' Teleports actor forward (in direction of its heading) by some offset '''
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, carlaActor, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		carlaActor.set_location(loc)

class SetLocationAction(simulators.Action):
	def __init__(self, pos):
		self.pos = pos  # Scenic position

	def applyTo(self, obj, carlaActor, sim):
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		carlaActor.set_location(loc)

class SetVelocityAction(simulators.Action):
	def __init__(self, velocity):
		self.velocity = utils.scenicToCarlaVector3D(velocity)

	def applyTo(self, obj, carlaActor, sim):
		carlaActor.set_velocity(self.velocity)

class SetAngularVelocityAction(simulators.Action):
	def __init__(self, angularVelocity):
		self.angularVelocity = angularVelocity

	def applyTo(self, obj, carlaActor, sim):
		carlaActor.set_angular_velocity(self.angularVelocity)

class SetTransformAction(simulators.Action):
	def __init__(self, pos, heading):
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
		self.throttle = throttle  # float in range [0.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.throttle = min(max(self.throttle, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class IncreaseThrottleAction(simulators.Action):
	def __init__(self, increment):
		self.increment = increment  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle + self.increment, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class DecreaseThrottleAction(simulators.Action):
	def __init__(self, decrement):
		self.decrement = decrement  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle - self.decrement, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class SetSteerAction(simulators.Action):
	def __init__(self, steer):
		self.steer = steer  # float in range [-1.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.steer = min(max(self.steer, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class SetBrakeAction(simulators.Action):
	def __init__(self, brake):
		self.brake = brake  # float in range [0.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.brake = min(max(self.brake, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class IncreaseBrakeAction(SetBrakeAction):
	def __init__(self, increment):
		self.increment = increment  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle + self.increment, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class DecreaseBrakeAction(SetBrakeAction):
	def __init__(self, decrement):
		self.decrement = decrement  # should be positive (but works regardless)

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.control()
		ctrl.throttle = min(max(ctrl.throttle - self.decrement, 0), 1)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class SetHandBrakeAction(simulators.Action):
	def __init__(self, handBrake):
		self.handBrake = handBrake  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.hand_brake = self.handBrake
		vehicle.apply_control(ctrl)

class SetReverseAction(simulators.Action):
	def __init__(self, reverse):
		self.reverse = reverse  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.reverse = self.reverse
		vehicle.apply_control(ctrl)

class SetManualGearShiftAction(simulators.Action):
	def __init__(self, manualGearShift):
		self.manualGearShift = manualGearShift  # boolean

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		vehicle.apply_control(ctrl)

class SetGearAction(simulators.Action):
	def __init__(self, gear):
		self.gear = gear  # int in range [1, 6]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = min(max(self.gear, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class IncreaseGearAction(simulators.Action):
	def __init__(self, increment=1):
		self.increment = increment  # should be 1

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctr.gear = min(max(ctr.gear + self.increment, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)

class DecreaseGearAction(simulators.Action):
	def __init__(self, decrement=1):
		self.decrement = decrement  # should be 1

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = min(max(ctrl.gear - self.decrement, 1), 6)  # cut off at range bounds
		vehicle.apply_control(ctrl)
