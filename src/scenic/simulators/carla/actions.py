import carla
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
		ctrl.throttle = self.throttle
		vehicle.apply_control(ctrl)

class SetSteerAction(simulators.Action):
	def __init__(self, steer):
		self.steer = steer  # float in range [-1.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.steer = self.steer
		vehicle.apply_control(ctrl)

class SetBrakeAction(simulators.Action):
	def __init__(self, brake):
		self.brake = brake  # float in range [0.0, 1.0]

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.brake = self.brake
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
		self.gear = gear  # (int) gear number 

	def applyTo(self, obj, vehicle, sim):
		ctrl = vehicle.get_control()
		ctrl.gear = self.gear
		vehicle.apply_control(ctrl)
