import carla
import scenic.simulators as simulators
import scenic.simulators.carla.utils as utils


class CarlaSimulator(simulators.Simulator):
	def __init__(self, carla_world, address='localhost', port=2000):
		super().__init__()
		self.client = carla.Client(address, port)
		self.client.set_timeout(10.0)  # limits networking operations (seconds)
		self.world = self.client.load_world(carla_world)

		# Set to synchronous with fixed timestep
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = 0.05
		settings.synchronous_mode = True
		self.world.apply_settings(settings)

	def createSimulation(self, scene):
		sim = CarlaSimulation(scene, self.client)

		# Destroy all actors in world
		for actor in self.world.get_actors():
			destroyed = actor.destroy()  # boolean
			if not destroyed:
				raise RuntimeError(f'Actor with id={actor.id} could not be destroyed.')

		return sim


class CarlaSimulation(simulators.Simulation):
	def __init__(self, scene, client):
		super().__init__(scene)
		self.client = client
		self.world = self.client.get_world()
		self.blueprintLib = self.world.get_blueprint_library()

		# Allow changing of timestep
		self.timeStep = scene.params.get('time_step', 1.0/30)
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = scene.params.get('time_step', 1.0/30)
		self.world.apply_settings(settings)
		
		# Reloads current world (destroys all actors, except traffic manager instances)
		self.client.reload_world()

		# Create Carla actors corresponding to Scenic objects
		for obj in self.objects:
			# Extract blueprint
			blueprint = self.blueprintLib.find(obj.blueprint)

			# Set up transform
			loc = utils.scenicToCarlaLocation(obj.position, obj.elevation)
			rot = utils.scenicToCarlaRotation(obj.heading)
			transform = carla.Transform(loc, rot)
			
			# Create Carla actor
			carlaActor = self.world.spawn_actor(blueprint, transform)  # raises exception if fails
			obj.carlaActor = carlaActor

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
			obj.position = utils.carlaToScenicPosition(currLoc)
			obj.elevation = utils.carlaToScenicElevation(currLoc)
			obj.heading = utils.carlaToScenicHeading(currRot, tolerance2D=5)

			# NOTE: Refer to utils.carlaToScenicHeading
			if obj.heading is None:
				raise RuntimeError(f'{carlaActor} has non-planar orientation')

	def currentState(self):
		return tuple(obj.position for obj in self.objects)

	def initialState(self):
		return self.currentState()

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
