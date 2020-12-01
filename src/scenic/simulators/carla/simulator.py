
try:
	import carla
except ImportError as e:
	raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

import math

from scenic.syntax.translator import verbosity
if verbosity == 0:	# suppress pygame advertisement at zero verbosity
	import os
	os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame

from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint
import scenic.simulators.carla.utils.utils as utils
import scenic.simulators.carla.utils.visuals as visuals


class CarlaSimulator(DrivingSimulator):
	def __init__(self, carla_map, address='127.0.0.1', port=2000, timeout=10,
		         render=True, record=False, timestep=0.1):
		super().__init__()
		verbosePrint('Connecting to CARLA...')
		self.client = carla.Client(address, port)
		self.client.set_timeout(timeout)  # limits networking operations (seconds)
		self.world = self.client.load_world(carla_map)
		self.map = carla_map
		self.timestep = timestep

		# Set to synchronous with fixed timestep
		settings = self.world.get_settings()
		settings.synchronous_mode = True
		settings.fixed_delta_seconds = timestep  # NOTE: Should not exceed 0.1
		self.world.apply_settings(settings)
		verbosePrint('Map loaded in simulator.')

		self.render = render  # visualization mode ON/OFF
		self.record = record  # whether to save images to disk

	def createSimulation(self, scene, verbosity=0):
		return CarlaSimulation(scene, self.client, self.map, self.timestep,
							   render=self.render, record=self.record,
							   verbosity=verbosity)


class CarlaSimulation(DrivingSimulation):
	def __init__(self, scene, client, map, timestep, render, record, verbosity=0):
		super().__init__(scene, timestep=timestep, verbosity=verbosity)
		self.client = client
		self.client.load_world(map)
		self.world = self.client.get_world()
		self.blueprintLib = self.world.get_blueprint_library()
		
		# Reloads current world: destroys all actors, except traffic manager instances
		# self.client.reload_world()
		
		# Setup HUD
		self.render = render
		self.record = record
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
			carlaActor = self.createObjectInSimulator(obj)

			# Check if ego (from carla_scenic_taks.py)
			if obj is self.objects[0]:
				self.ego = obj

				# Set up camera manager and collision sensor for ego
				if self.render:
					camIndex = 0
					camPosIndex = 0
					self.cameraManager = visuals.CameraManager(self.world, carlaActor, self.hud)
					self.cameraManager._transform_index = camPosIndex
					self.cameraManager.set_sensor(camIndex)
					self.cameraManager.set_transform(self.camTransform)
					self.cameraManager._recording = self.record

		self.world.tick() ## allowing manualgearshift to take effect 	# TODO still need this?

		for obj in self.objects:
			if isinstance(obj.carlaActor, carla.Vehicle):
				obj.carlaActor.apply_control(carla.VehicleControl(manual_gear_shift=False))

		self.world.tick()

		# Set Carla actor's initial speed (if specified)
		for obj in self.objects:
			if obj.speed is not None:
				equivVel = utils.scenicSpeedToCarlaVelocity(obj.speed, obj.heading)
				obj.carlaActor.set_velocity(equivVel)

	def createObjectInSimulator(self, obj):
		# Extract blueprint
		blueprint = self.blueprintLib.find(obj.blueprint)

		# Set up transform
		loc = utils.scenicToCarlaLocation(obj.position, z=obj.elevation, world=self.world)
		rot = utils.scenicToCarlaRotation(obj.heading)
		transform = carla.Transform(loc, rot)

		# Create Carla actor
		carlaActor = self.world.try_spawn_actor(blueprint, transform)
		if carlaActor is None:
			self.destroy()
			raise SimulationCreationError(f'Unable to spawn object {obj}')

		if isinstance(carlaActor, carla.Vehicle):
			# TODO manual gear shift issue? (see above)
			carlaActor.apply_control(carla.VehicleControl(manual_gear_shift=False, gear=1))
		elif isinstance(carlaActor, carla.Walker):
			carlaActor.apply_control(carla.WalkerControl())

		obj.carlaActor = carlaActor
		return carlaActor

	def executeActions(self, allActions):
		super().executeActions(allActions)

		# Apply control updates which were accumulated while executing the actions
		for obj in self.agents:
			ctrl = obj._control
			if ctrl is not None:
				obj.carlaActor.apply_control(ctrl)
				obj._control = None

	def step(self):
		# Run simulation for one timestep
		self.world.tick()

		# Render simulation
		if self.render:
			# self.hud.tick(self.world, self.ego, self.displayClock)
			self.cameraManager.render(self.display)
			# self.hud.render(self.display)
			pygame.display.flip()

	def getProperties(self, obj, properties):
		# Extract Carla properties
		carlaActor = obj.carlaActor
		currTransform = carlaActor.get_transform()
		currLoc = currTransform.location
		currRot = currTransform.rotation
		currVel = carlaActor.get_velocity()
		currAngVel = carlaActor.get_angular_velocity()

		# Prepare Scenic object properties
		velocity = utils.carlaToScenicPosition(currVel)
		speed = math.hypot(*velocity)

		values = dict(
			position=utils.carlaToScenicPosition(currLoc),
			elevation=utils.carlaToScenicElevation(currLoc),
			heading=utils.carlaToScenicHeading(currRot),
			velocity=velocity,
			speed=speed,
			angularSpeed=utils.carlaToScenicAngularSpeed(currAngVel),
		)
		return values
