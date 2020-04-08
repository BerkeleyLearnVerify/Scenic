import carla
import scenic.simulators.carla.utils as utils

class CarlaSimulator(simulators.Simulator):
	def __init__(self, carla_scene, address='localhost', port=2000):
		super().__init__()
		self.client = carla.Client(address, port)
		self.client.set_timeout(10.0)  # limits networking operations (seconds)
		self.world = self.client.get_world()
		self.bpLib = self.world.get_blueprint_library()

		# Set to synchronous with fixed timestep
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = 0.05
		settings.synchronous_mode = True
		self.world.apply_settings(settings)

	def createSimulation(self, scene):
		return CarlaSimulation(scene, self.client)

class CarlaSimulation(simulators.Simulation):
	def __init__(self, scene, client):
		super().__init__(scene)
		self.client = client

		# Allow changing of timestep
		self.timeStep = scene.params.get('time_step', 1.0/30)
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = scene.params.get('time_step', 1.0/30)
		self.world.apply_settings(settings)
		
		# Reloads current world (destroys all actors, except traffic manager instances)
		self.client.reload_world()

		# Create Carla actors corresponding to Scenic objects
		for obj in self.objects:
			# Set up transform
			loc = utils.scenicToCarlaLocation(obj.position, obj.elevation)
			rot = utils.scenicToCarlaRotation(obj.heading)
			transform = carla.Transform(loc, rot)
			
			# Create Carla actor
			carlaActor = self.world.try_spawn_actor(obj.blueprint, transform)
			obj.carlaActor = carlaActor

		self.initState = tuple(obj.position for obj in self.objects)

	def writePropertiesToCarla(self):
		for obj in self.objects:
			# Compute Carla properties
			carlaActor = obj.carlaActor
			newLoc = utils.scenicToCarlaLocation(obj.position, z=obj.elevation)
			newRot = utils.scenicToCarlaRotation(obj.heading)
			newTransform = carla.Transform(newLoc, newRot)

			# Update Carla actor properties
			carlaActor.set_transform(newTransform)

	def readPropertiesFromCarla(self):
		for obj in self.objects:
			# Extract Carla properties
			carlaActor = obj.carlaActor
			currTransform = carlaActor.get_transform()
			currLoc = currTransform.location
			currRot = currTransform.rotation

			# Update Scenic object properties
			obj.position = utils.carlaToScenicPosition(currLocation)
			obj.elevation = utils.carlaToScenicElevation(currLocation)
			obj.heading = utils.carlaToScenicHeading(currTransform.rotation, tolerance2D=5)
			if obj.heading is None:
				raise RuntimeError(f'{carlaActor} has non-planar orientation')

	def currentState(self):
		return tuple(obj.position for obj in self.objects)

	def initialState(self):
		return self.initState

	def step(self, actions):
		# Execute actions
		for obj, action in actions.items():
			if action:
				action.applyTo(obj, obj.carlaActor, self)

		# Run simulation for one timestep
		self.world.tick()

		# Read back the results of the simulation
		self.readPropertiesFromCarla()

		return self.currentState()

################################################
# Actions available to all carla.Actor objects #
################################################

# NOTE: Equivalent to LGSVL's MoveAction class
class OffsetAction(simulators.Action):
	'''Teleports actor forward (in direction of its heading) by some offset'''
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, carlaActor, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset) #w.r. t
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
