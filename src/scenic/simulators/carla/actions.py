import math as _math

import carla as _carla

from scenic.domains.driving.actions import *
import scenic.simulators.carla.utils.utils as _utils
import scenic.simulators.carla.model as _carlaModel

################################################
# Actions available to all carla.Actor objects #
################################################

SetLocationAction = SetPositionAction	# TODO refactor

class SetAngularVelocityAction(Action):
	def __init__(self, angularVel):
		self.angularVel = angularVel

	def applyTo(self, obj, sim):
		xAngularVel = self.angularVel * _math.cos(obj.heading)
		yAngularVel = self.angularVel * _math.sin(obj.heading)
		newAngularVel = _utils.scalarToCarlaVector3D(xAngularVel, yAngularVel)
		obj.carlaActor.set_angular_velocity(newAngularVel)

class SetTransformAction(Action):	# TODO eliminate
	def __init__(self, pos, heading):
		self.pos = pos
		self.heading = heading

	def applyTo(self, obj, sim):
		loc = _utils.scenicToCarlaLocation(pos, z=obj.elevation)
		rot = _utils.scenicToCarlaRotation(heading)
		transform = _carla.Transform(loc, rot)
		obj.carlaActor.set_transform(transform)


#############################################
# Actions specific to carla.Vehicle objects #
#############################################

class VehicleAction(Action):
	def canBeTakenBy(self, agent):
		return isinstance(agent, _carlaModel.Vehicle)

class SetManualGearShiftAction(VehicleAction):
	def __init__(self, manualGearShift):
		if not isinstance(manualGearShift, bool):
			raise RuntimeError('Manual gear shift must be a boolean.')
		self.manualGearShift = manualGearShift

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.manual_gear_shift = self.manualGearShift
		vehicle.apply_control(ctrl)


class SetGearAction(VehicleAction):
	def __init__(self, gear):
		if not isinstance(gear, int):
			raise RuntimeError('Gear must be an int.')
		self.gear = gear

	def applyTo(self, obj, sim):
		vehicle = obj.carlaActor
		ctrl = vehicle.get_control()
		ctrl.gear = self.gear
		vehicle.apply_control(ctrl)


class SetManualFirstGearShiftAction(VehicleAction):	# TODO eliminate
	def applyTo(self, obj, sim):
		ctrl = _carla.VehicleControl(manual_gear_shift=True, gear=1)
		obj.carlaActor.apply_control(ctrl)


#################################################
# Actions available to all carla.Walker objects #
#################################################

class PedestrianAction(Action):
	def canBeTakenBy(self, agent):
		return isinstance(agent, _carlaModel.Pedestrian)

class SetJumpAction(PedestrianAction):
	def __init__(self, jump):
		if not isinstance(jump, bool):
			raise RuntimeError('Jump must be a boolean.')
		self.jump = jump

	def applyTo(self, obj, sim):
		walker = obj.carlaActor
		ctrl = walker.get_control()
		ctrl.jump = self.jump
		walker.apply_control(ctrl)
