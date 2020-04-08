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
		settings.fixed_delta_seconds = 0.05  # TODO: change later based on scene's timestep?
		settings.synchronous_mode = True
		self.world.apply_settings(settings)

	def createSimulation(self, scene):
		return CarlaSimulation(scene, self.client)

class CarlaSimulation(simulators.Simulation):
	def __init__(self, scene, client):
		super().__init__(scene)
		self.client = client

		self.timeStep = scene.params.get('time_step', 1.0/30)  # TODO: find out what this means -- change settings.fixed_delta_seconds accordingly?
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = scene.params.get('time_step', 1.0/30)
		self.world.apply_settings(settings)
		
		# Reloads current world (destroys all actors, except traffic manager instances)
		self.client.reload_world()

		# Create Carla actors corresponding to Scenic objects
		for obj in self.objects:
			# TODO: check this with implementation of self.objects
			# Determine type of Carla actor
			if not hasattr(obj, 'carlaActor'):
				continue  # not a Carla actor
			if not hasattr(obj, 'carlaName'):
				raise RuntimeError(f'Object {obj} does not have carlaName property')
			if not hasattr(obj, 'carlaActorType'):
				raise RuntimeError(f'Object {obj} does not have carlaActorType property')
			if not hasattr(obj, 'carlaActorModel'):
				raise RuntimeError(f'Object {obj} does not have carlaActorModel property')
			bpStr = f'{obj.carlaName}.{obj.carlaActorType}.{obj.carlaActorModel}'

			# Set up blueprint and transform
			blueprint = bpLib.find(bpStr)
			if blueprint is None:
				raise RuntimeError(f'Could not find blueprint with id {bpStr}')
			location = utils.scenicToCarlaLocation(obj.position, obj.elevation)
			rotation = utils.scenicToCarlaRotation(obj.heading)
			transform = carla.Transform(location, rotation)
			
			# Create Carla actor
			carlaActor = self.world.spawn_actor(blueprint, transform)
			if carlaActor is None:
				raise RuntimeError(f'Could not spawn Carla actor with blueprint={blueprint} and transform={transform}')
			obj.carlaActor = carlaActor

		self.initState = tuple(obj.position for obj in self.objects)

	def writePropertiesToCarla(self):
		for obj in self.objects:
			carlaActor = obj.carlaActor
			newLocation = utils.scenicToCarlaLocation(obj.position, z=obj.elevation)
			newRotation = utils.scenicToCarlaRotation(obj.heading)
			newTransform = carla.Transform(newLocation, newRotation)

			# Update Carla actor properties
			carlaActor.set_transform(newTransform)

	def readPropertiesFromCarla(self):
		for obj in self.objects:
			carlaActor = obj.carlaActor
			currTransform = carlaActor.get_transform()
			currLocation = currTransform.location
			currHeading = utils.carlaToScenicHeading(currTransform.rotation, tolerance2D=5)
			if currHeading is None:
				raise RuntimeError(f'{carlaActor} has non-planar orientation')

			# Update Scenic object properties
			obj.position = utils.carlaToScenicPosition(currLocation)
			obj.elevation = utils.carlaToScenicElevation(currLocation)
			obj.heading = currHeading

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

class MoveAction(simulators.Action):
	def __init__(self, offset):
		self.offset = offset

	def applyTo(self, obj, carlaActor, sim):
		pos = obj.position.offsetRotated(obj.heading, self.offset)  # TODO: understand what offsetRotated() does
		loc = utils.scenicToCarlaLocation(pos, z=obj.elevation)
		carlaActor.set_location(loc)

class SetVelocityAction(simulators.Action):
	def __init__(self, velocity):
		self.velocity = utils.scenicToCarlaVector3D(velocity)

	def applyTo(self, obj, carlaActor, sim):
		carlaActor.set_velocity(self.velocity)

# TODO: write more primitive action classes
# - FollowWaypointsAction
# - CancelWaypointsAction
