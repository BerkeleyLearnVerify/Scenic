
try:
	import carla
except ImportError as e:
	raise RuntimeError('CARLA scenarios require the "carla" Python package') from e

import pygame

import scenic.simulators as simulators
import scenic.simulators.carla.utils.utils as utils
import scenic.simulators.carla.utils.visuals as visuals


class CarlaSimulator(simulators.Simulator):
	def __init__(self, carla_map, address='127.0.0.1', port=2000, timeout=10, render=True,
	             timestep=0.1):
		super().__init__()
		self.client = carla.Client(address, port)
		self.client.set_timeout(timeout)  # limits networking operations (seconds)
		self.world = self.client.load_world(carla_map)
		self.map = carla_map

		# Set to synchronous with fixed timestep
		settings = self.world.get_settings()
		settings.synchronous_mode = True
		settings.fixed_delta_seconds = timestep  # NOTE: Should not exceed 0.1
		self.world.apply_settings(settings)

		self.render = render  # visualization mode ON/OFF

	def createSimulation(self, scene):
		return CarlaSimulation(scene, self.client, self.render, self.map)


class CarlaSimulation(simulators.Simulation):
	def __init__(self, scene, client, render, map):
		super().__init__(scene)
		self.client = client
		self.client.load_world(map)
		self.world = self.client.get_world()
		self.blueprintLib = self.world.get_blueprint_library()
		
		# Reloads current world: destroys all actors, except traffic manager instances
		# self.client.reload_world()
		
		# Setup HUD
		self.render = render
		if self.render:
			self.displayDim = (1280, 720)
			self.displayClock = pygame.time.Clock()
			self.camTransform = 0
			pygame.init()
			pygame.font.init()
			self.hud = visuals.HUD(*self.displayDim)
			self.display = pygame.display.set_mode(
				self.displayDim,
				pygame.HWSURFACE | pygame.DOUBLEBUF
			)
			self.cameraManager = None

		# Create Carla actors corresponding to Scenic objects
		self.ego = None
		for obj in self.objects:
			# Extract blueprint
			blueprint = self.blueprintLib.find(obj.blueprint)

			# Set up transform
			loc = utils.scenicToCarlaLocation(obj.position, world=self.world)
			rot = utils.scenicToCarlaRotation(obj.heading)
			transform = carla.Transform(loc, rot)
			
			# Create Carla actor
			carlaActor = self.world.try_spawn_actor(blueprint, transform)
			if carlaActor is None:
				raise simulators.SimulationCreationError(
				    f'Unable to spawn object {type(obj)} at position {obj.position}, '
				    'likely from a spawn collision'
				)
			if isinstance(carlaActor, carla.Vehicle):
				carlaActor.apply_control(carla.VehicleControl())  # set default controls
			elif isinstance(carlaActor, carla.Walker):
				carlaActor.apply_control(carla.WalkerControl())

			# Set Carla actor's initial speed (if specified)
			# if obj.speed is not None:
			# 	equivVel = utils.scenicSpeedToCarlaVelocity(obj.speed, obj.heading)
			# 	carlaActor.set_velocity(equivVel)

			obj.carlaActor = carlaActor

			# Check if ego (from carla_scenic_taks.py)
			if obj is self.objects[0]:
				self.ego = obj

				# Setup camera manager and collision sensor for ego
				if self.render:
					camIndex = 0
					camPosIndex = 0
					self.cameraManager = visuals.CameraManager(self.world, carlaActor, self.hud)
					self.cameraManager._transform_index = camPosIndex
					self.cameraManager.set_sensor(camIndex)
					self.cameraManager.set_transform(self.camTransform)

	def readPropertiesFromCarla(self):
		for obj in self.objects:

			# Extract Carla properties
			carlaActor = obj.carlaActor
			currTransform = carlaActor.get_transform()
			currLoc = currTransform.location
			currRot = currTransform.rotation
			currVel = carlaActor.get_velocity()
			# print(carlaActor.get_acceleration())

			# Update Scenic object properties
			obj.position = utils.carlaToScenicPosition(currLoc)
			obj.elevation = utils.carlaToScenicElevation(currLoc)
			obj.heading = utils.carlaToScenicHeading(currRot, tolerance2D=5.0)
			obj.speed = utils.carlaVelocityToScenicSpeed(currVel)

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
			# if obj.carlaActor.get_control().throttle > 0:
			# 	print(obj.carlaActor.get_acceleration())

		# Run simulation for one timestep
		self.world.tick()

		# Render simulation
		if self.render:
			# self.hud.tick(self.world, self.ego, self.displayClock)
			self.cameraManager.render(self.display)
			# self.hud.render(self.display)
			pygame.display.flip()

		# Read back the results of the simulation
		self.readPropertiesFromCarla()

		return self.currentState()
