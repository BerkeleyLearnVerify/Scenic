"""Simulator interface for CARLO."""
try:
    import scenic.simulators.carlo.CARLO
except ImportError as e:
    raise ModuleNotFoundError('Please add the CARLO library to this module for use.') from e

import math
import os

from scenic.syntax.translator import verbosity

if verbosity == 0:  # suppress pygame advertisement at zero verbosity
    import os

    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame

from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.carlo.CARLO.agents import Car, Pedestrian
from scenic.simulators.carlo.CARLO.geometry import Point
import numpy as np
import scenic.simulators.carlo.utils.utils as utils


class CarloSimulator(DrivingSimulator):
    """Implementation of `Simulator` for CARLA."""

    def __init__(self, world,
                 render=True, record='', traffic_manager_port=None):
        super().__init__()

        # Set to synchronous with fixed timestep
        settings = self.world.get_settings()
        self.world.apply_settings(settings)
        verbosePrint('Map loaded in simulator.')

        self.render = render  # visualization mode ON/OFF
        self.record = record  # whether to use the carla recorder
        self.scenario_number = 0  # Number of the scenario executed

    def createSimulation(self, world, verbosity=0):
        self.scenario_number += 1
        return CarloSimulation(world, self.client, self.tm, self.timestep,
                               render=self.render, record=self.record,
                               scenario_number=self.scenario_number, verbosity=verbosity)

    def destroy(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(False)

        super().destroy()

'''
Not currently supported:
Recording episodes
'''
class CarloSimulation(DrivingSimulation):
    def __init__(self, world, tm, render, verbosity=0):
        super().__init__(world, verbosity=verbosity)
        self.world = world
        self.world_dimensions = (self.world.width, self.world.height)
        self.tm = tm

        self.render = render


        # Create the Carlo objects corresponding to Scenic objects
        self.ego = None
        for obj in self.objects:
            carloObj = self.createObjectInSimulator(obj)

            # Check if ego (from carla_scenic_taks.py)
            if obj is self.objects[0]:
                self.ego = obj
            self.world.add(carloObj)
            # Set up camera manager and collision sensor for ego


        self.world.tick()
        if self.render:
            #visualize the constructed map
            self.world.render()

    def createObjectInSimulator(self, obj):
        # Extract blueprint
        blueprint = self.blueprintLib.find(obj.blueprint)
        if obj.rolename is not None:
            blueprint.set_attribute('role_name', obj.rolename)

        # Create Carlo object
        if obj.rolename == "Car":
            carloAgent = Car(Point(obj.position[0], obj.position[1]), obj.heading)
        else:
            carloAgent = Pedestrian(Point(obj.position[0], obj.position[1]), obj.heading)
        carloAgent.velocity = Point(obj.velocity[0], obj.velocity[1])
        carloAgent.max_speed = obj.max_speed
        carloAgent.friction = obj.friction
        self.world.add(carloAgent)
        if carloAgent is None:
            self.destroy()
            raise SimulationCreationError(f'Unable to spawn object {obj}')
        obj.carloAgent = carloAgent

        return carloAgent

    def executeActions(self, allActions):
        super().executeActions(allActions)

        # Apply control updates which were accumulated while executing the actions
        for obj in self.agents:
            ctrl = obj._control
            if ctrl is not None:
                obj.carloAgent.set_control(ctrl)
                obj._control = None

    def step(self):
        # Run simulation for one timestep
        self.world.tick()

        # Render simulation
        if self.render:
            self.world.render()

    def getProperties(self, obj, properties):
        # Extract Carla properties
        carloAgent = obj.carloAgent
        currPos = carloAgent.center
        currHead = carloAgent.heading
        currVel = carloAgent.velocity
        currAngVel = carloAgent.angular_velocity

        # Prepare Scenic object properties
        velocity = utils.carlaToScenicPosition(currVel)
        speed = math.hypot(*velocity)

        values = dict(
            position=utils.carlaToScenicPosition(currPos),
            heading=utils.carlaToScenicHeading(currHead),
            velocity=currVel,
            angularVelocity=currAngVel,
        )
        return values

    def destroy(self):
        self.world.reset()
        super().destroy()
