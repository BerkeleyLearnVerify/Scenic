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
		self.timeStep = scene.params.get('time_step', 1.0/30)  # TODO: find out what this means
		
		# Reloads current world (destroys all actors, except traffic manager instances)
		self.client.reload_world()

		# Create Carla actors corresponding to Scenic objects
		for obj in self.objects:
			# Determine type of Carla actor
			if not type(obj) == carla.Actor:
				continue  # not a Carla actor
			# TODO: finish determining type of Carla actor

			# Set up blueprint and transform
			blueprint = None  # TODO: figure out blueprint
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
		for actor, action in actions.items():
			if action:
				action.applyTo(actor, actor.carlaActor, self)

		# Run simulation for one timestep
		self.world.tick()

		# Read back the results of the simulation
		self.readPropertiesFromCarla()

		return self.currentState()
