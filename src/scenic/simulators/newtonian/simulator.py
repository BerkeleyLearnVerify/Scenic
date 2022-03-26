"""Newtonian simulator implementation."""

import math
from math import sin, radians, degrees, copysign
import os
import pathlib
import time

from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint
from scenic.core.vectors import Vector
import scenic.simulators.newtonian.utils.utils as utils
from scenic.domains.driving.roads import Network
import scenic.domains.driving.model as drivingModel
from scenic.syntax.translator import verbosity

import shapely
if verbosity == 0:  # suppress pygame advertisement at zero verbosity
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame

current_dir = pathlib.Path(__file__).parent.absolute()

WIDTH = 1280
HEIGHT = 800
MAX_ACCELERATION = 5.6 # in m/s2, seems to be a pretty reasonable value
MAX_BRAKING = 4.6

class NewtonianSimulator(DrivingSimulator):
    """Implementation of `Simulator` for the Newtonian simulator.

    Args:
        network (Network): road network to display in the background, if any.
        timestep (float): time step to use.
        render (bool): whether to render the simulation in a window.
    """
    def __init__(self, network=None, timestep=0.1, render=False):
        self.timestep = timestep
        self.render = render
        self.network = network

    def createSimulation(self, scene, verbosity=0):
        return NewtonianSimulation(scene, self.network, timestep=self.timestep,
                                   verbosity=verbosity, render=self.render)

class NewtonianSimulation(DrivingSimulation):
    def __init__(self, scene, network, timestep, verbosity=0, render=False):
        super().__init__(scene, timestep=timestep, verbosity=verbosity)
        self.render = render
        self.network = network
        self.ego = self.objects[0]

        # Set actor's initial velocity (if specified)
        for obj in self.objects:
            if obj.speed is not None:
                equivVel = obj.speed*utils.vectorFromHeading(obj.heading)
                obj.setVelocity(equivVel)

        if self.render:
            # determine window size
            min_x, min_y = self.ego.position
            max_x, max_y = self.ego.position
            for obj in self.objects:
                x, y = obj.position
                min_x, max_x = min(min_x, x), max(max_x, x)
                min_y, max_y = min(min_y, y), max(max_y, y)

            pygame.init()
            pygame.font.init()
            self.screen = pygame.display.set_mode((WIDTH,HEIGHT),
                                                  pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.screen.fill((255, 255, 255))
            x, y = self.ego.position
            self.x_window = [min_x-50, max_x+50]
            self.y_window = [min_y-50, max_y+50]
            xlim1, xlim2 = self.x_window
            ylim1, ylim2 = self.y_window
            img_path = os.path.join(current_dir, 'car.png')
            self.car = pygame.image.load(img_path)
            self.car_width = int(3.5 * WIDTH / (xlim2 - xlim1))
            self.car_height = self.car_width
            self.car = pygame.transform.scale(self.car, (self.car_width,
                                                         self.car_height))
            self.parse_network()
            self.draw_objects()

    def parse_network(self):
        self.network_polygons = []
        if not self.network:
            return
        for element in self.network.elements.values():
            if not hasattr(element, 'polygon'):
                continue
            if isinstance(element.polygon, shapely.geometry.multipolygon.MultiPolygon):
                all_polygons = list(element.polygon.geoms)
            else:
                all_polygons = [element.polygon]
            for poly in all_polygons:
                if self.boundingBoxOnScreen(*poly.bounds):
                    screenPoints = map(self.scenicToScreenVal, poly.exterior.coords)
                    self.network_polygons.append(list(screenPoints))

    def scenicToScreenVal(self, pos):
        x, y = pos
        min_x, max_x = self.x_window
        min_y, max_y = self.y_window
        x_prop = (x - min_x) / (max_x - min_x)
        y_prop = (y - min_y) / (max_y - min_y)
        return int(x_prop * WIDTH), HEIGHT - 1 - int(y_prop * HEIGHT)

    def createObjectInSimulator(self, obj):
        pass

    def actionsAreCompatible(self, agent, actions):
        return True

    def boundingBoxOnScreen(self, x1, y1, x2, y2):
        min_x, max_x = self.x_window
        min_y, max_y = self.y_window
        onScreen = lambda x, y: min_x <= x <= max_x and min_y <= y <= max_y
        return (onScreen(x1, y1) or onScreen(x2, y2) or
               onScreen(x1, y2) or onScreen(x2, y1))

    def step(self):
        for obj in self.objects:
            if isinstance(obj, drivingModel.Steers):
                if obj.hand_brake:
                    acceleration = -MAX_BRAKING
                elif obj.brake > 0:
                    acceleration = -obj.brake * MAX_BRAKING
                else:
                    acceleration = obj.throttle * MAX_ACCELERATION
                obj.speed += acceleration * self.timestep
                obj.velocity = Vector(0, obj.speed).rotatedBy(obj.heading)
                if obj.steer:
                    turning_radius = obj.length / sin(obj.steer * math.pi / 2)
                    obj.angularSpeed = -obj.speed / turning_radius
                else:
                    obj.angularSpeed = 0
            obj.position += obj.velocity * self.timestep
            obj.heading += obj.angularSpeed * self.timestep
        if self.render:
            self.draw_objects()

    def draw_objects(self):
        self.screen.fill((255, 255, 255))
        for screenPoints in self.network_polygons:
            pygame.draw.polygon(self.screen, (0, 0, 0), list(screenPoints), width=1)

        for obj in self.objects:
            color = (255, 0, 0) if obj is self.ego else (0, 0, 255)
            h, w = obj.length, obj.width
            pos_vec = Vector(-1.75, 1.75)
            neg_vec = Vector(w / 2, h / 2)
            heading_vec = Vector(0, 10).rotatedBy(obj.heading)
            dx, dy = int(heading_vec.x), -int(heading_vec.y)
            x, y = self.scenicToScreenVal(obj.position)
            rect_x, rect_y = self.scenicToScreenVal(obj.position + pos_vec)
            if isinstance(obj, drivingModel.Vehicle):
                self.rotated_car = pygame.transform.rotate(self.car, math.degrees(obj.heading))
                self.screen.blit(self.rotated_car, (rect_x, rect_y))
            else:
                corners = [self.scenicToScreenVal(corner) for corner in obj.corners]
                pygame.draw.polygon(self.screen, color, corners)

        pygame.display.update()
        time.sleep(self.timestep)

    def getProperties(self, obj, properties):
        values = dict(
            position=obj.position,
            elevation=obj.elevation,
            heading=obj.heading,
            velocity=obj.velocity,
            speed=obj.speed,
            angularSpeed=obj.angularSpeed,
        )
        return values
