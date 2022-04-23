"""Newtonian simulator implementation."""

import math
from math import sin, radians, degrees, copysign
import os
import pathlib
import time

from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.geometry import allChains
from scenic.core.regions import toPolygon
from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint
from scenic.core.vectors import Vector
import scenic.simulators.newtonian.utils.utils as utils
from scenic.domains.driving.roads import Network
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

ROAD_COLOR = (0, 0, 0)
ROAD_WIDTH = 2
LANE_COLOR = (96, 96, 96)
CENTERLINE_COLOR = (224, 224, 224)
SIDEWALK_COLOR = (0, 128, 255)
SHOULDER_COLOR = (96, 96, 96)

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
                obj.velocity = equivVel

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
            self.min_x, self.max_x = min_x-50, max_x+50
            self.min_y, self.max_y = min_y-50, max_y+50
            self.size_x = self.max_x - self.min_x
            self.size_y = self.max_y - self.min_y
            self.screen_poly = shapely.geometry.Polygon((
                (self.min_x, self.min_y),
                (self.max_x, self.min_y),
                (self.max_x, self.max_y),
                (self.min_x, self.max_y)
            ))

            img_path = os.path.join(current_dir, 'car.png')
            self.car = pygame.image.load(img_path)
            self.car_width = int(3.5 * WIDTH / self.size_x)
            self.car_height = self.car_width
            self.car = pygame.transform.scale(self.car, (self.car_width,
                                                         self.car_height))
            self.parse_network()
            self.draw_objects()

    def parse_network(self):
        self.network_polygons = []
        if not self.network:
            return

        def addRegion(region, color, width=1):
            poly = toPolygon(region)
            if not poly or not self.screen_poly.intersects(poly):
                return
            for chain in allChains(poly):
                coords = tuple(self.scenicToScreenVal(pt) for pt in chain.coords)
                self.network_polygons.append((coords, color, width))

        addRegion(self.network.walkableRegion, SIDEWALK_COLOR)
        addRegion(self.network.shoulderRegion, SHOULDER_COLOR)
        for road in self.network.roads: # loop over main roads
            for lane in road.lanes:
                addRegion(lane.leftEdge, LANE_COLOR)
                addRegion(lane.rightEdge, LANE_COLOR)
            addRegion(road, ROAD_COLOR, ROAD_WIDTH)
        for lane in self.network.lanes: # loop over all lanes, even in intersections
            addRegion(lane.centerline, CENTERLINE_COLOR)
        addRegion(self.network.intersectionRegion, ROAD_COLOR)

    def scenicToScreenVal(self, pos):
        x, y = pos
        x_prop = (x - self.min_x) / self.size_x
        y_prop = (y - self.min_y) / self.size_y
        return int(x_prop * WIDTH), HEIGHT - 1 - int(y_prop * HEIGHT)

    def createObjectInSimulator(self, obj):
        pass

    def actionsAreCompatible(self, agent, actions):
        return True

    def isOnScreen(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def step(self):
        for obj in self.objects:
            if hasattr(obj, 'hand_brake'):
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
        for screenPoints, color, width in self.network_polygons:
            pygame.draw.lines(self.screen, color, False, screenPoints, width=width)

        for obj in self.objects:
            color = (255, 0, 0) if obj is self.ego else (0, 0, 255)
            h, w = obj.length, obj.width
            pos_vec = Vector(-1.75, 1.75)
            neg_vec = Vector(w / 2, h / 2)
            heading_vec = Vector(0, 10).rotatedBy(obj.heading)
            dx, dy = int(heading_vec.x), -int(heading_vec.y)
            x, y = self.scenicToScreenVal(obj.position)
            rect_x, rect_y = self.scenicToScreenVal(obj.position + pos_vec)
            if any('Vehicle' in cls.__name__ for cls in type(obj).__mro__):     # TODO improve?
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
