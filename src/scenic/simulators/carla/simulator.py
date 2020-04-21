import carla
import pygame
import scenic.simulators as simulators
import scenic.simulators.carla.hud as hud
import scenic.simulators.carla.visuals as visuals


class CarlaSimulator(simulators.Simulator):
	def __init__(self, carla_world, address='127.0.0.1', port=2000, render=True):
		super().__init__()
		self.client = carla.Client(address, port)
		self.client.set_timeout(10.0)  # limits networking operations (seconds)
		self.world = self.client.load_world(carla_world)

		# Set to synchronous with fixed timestep
		settings = self.world.get_settings()
		settings.fixed_delta_seconds = 0.05
		settings.synchronous_mode = True
		self.world.apply_settings(settings)

		self.render = render  # visualization mode ON/OFF

	def createSimulation(self, scene):
		return CarlaSimulation(scene, self.client, self.render)


class CarlaSimulation(simulators.Simulation):
	def __init__(self, scene, client, render):
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

		# Setup HUD
		self.render = render
		if self.render:
			self.displayDim = (1280, 720)
			self.displayClock = pygame.time.Clock()
			self.camTransform = 0
			self.hud = visuals.HUD(*self.displayDim)
			self.display = pygame.display.set_mode(
				self.displayDim,
				pygame.HWSURFACE | pygame.DOUBLEBUF
			)
			self.cameraManager = None

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

			# Setup camera manager for ego
			if self.render and obj is self.objects[0]:  # from carla_scenic_taks.py
				camIndex = 0
				camPosIndex = 0
				self.cameraManager = visuals.CameraManager(carlaActor, self.hud)
				self.cameraManager._transform_index = camPosIndex
	            self.cameraManager.set_sensor(camIndex, notify=False)
	            self.cameraManager.set_transform(self.cam_transform)
	            actorType = visuals.get_actor_display_name(carlaActor)
	            self.hud.notification(actorType)

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

		# Render simulation
		if self.render:
			self.hud.tick(self.world, self.displayClock)  # TODO: self.world may be wrong
			self.cameraManager.render(self.display)
			self.hud.render(self.display)

		# Read back the results of the simulation
		self.readPropertiesFromCarla()

		return self.currentState()
