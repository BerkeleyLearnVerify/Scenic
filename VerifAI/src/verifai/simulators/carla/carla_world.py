# This file is a modification of existing CARLA code:

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref

import pygame
import numpy as np

import carla
from carla import ColorConverter as cc


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name

class Entity():
    ''' Bundles an actor in the Carla world with any corresponding data (e.g. blueprint, spawn point).'''
    def __init__(self, world, blueprint_filter,
                 spawn=None, color=None, ego=False):
        self.world = world      # Not a CARLA world, but the wrapper World class defined below
        self.blueprint_filter = blueprint_filter
        self.spawn = spawn
        self.actor = None       # Created when entity spawned.
        self.ego = ego
        self.color = color
        blueprints = self.world.world.get_blueprint_library()
        if self.blueprint_filter:
            blueprints = blueprints.filter(self.blueprint_filter)
        self.blueprint = np.random.choice(blueprints)
        if self.blueprint.has_attribute('color'):
            if self.color is None:
                self.color = random.choice(
                    self.blueprint.get_attribute('color').recommended_values)
            self.blueprint.set_attribute('color', self.color)

    def destroy(self):
        if self.actor is not None:
            self.actor.destroy()
        self.actor = None

    def create(self):
        if self.ego:
            self.blueprint.set_attribute('role_name', 'hero')
        self.actor = None
        tries = 0
        while not self.actor:
            spawn = self.spawn if self.spawn \
                else np.random.choice(self.world.map.get_spawn_points())
            self.actor = self.world.world.try_spawn_actor(self.blueprint, spawn)
            tries += 1
            if tries > 10:
                print('Unable to spawn entity at', self.spawn.location.x,
                        self.spawn.location.y, self.spawn.location.z)
                return False
        return True


class Pedestrian(Entity):
    def __init__(self, world, blueprint_filter='walker.pedestrian.*',
                 spawn=None, color=None, ego=False):
        # Pedestrian cannot be ego:
        assert not ego
        super().__init__(world, blueprint_filter,
                         spawn=spawn, color=color, ego=False)

    def create(self):
        if not super().create():
            return False
        return True

    def destroy(self):
        super().destroy()

class Prop(Entity):
    def __init__(self, world, blueprint_filter='static.prop.*',
                 spawn=None, ego=False):
        # Prop cannot be ego:
        assert not ego
        super().__init__(world, blueprint_filter=blueprint_filter,
                         spawn=spawn, ego=False)

    def create(self):
        if not super().create():
            return False
        return True

    def destroy(self):
        super().destroy()


class Vehicle(Entity):
    ''' Bundles up a CARLA Vehicle with its controller, blueprint, spawn point.'''
    def __init__(self, world, controller, control_params=None,
                 blueprint_filter='vehicle.*', spawn=None, physics=None,
                 has_collision_sensor=False, has_lane_sensor=False, color=None, ego=False):
        super().__init__(world, blueprint_filter,
                         spawn=spawn, color=color, ego=ego)
        self.ego = ego
        self.physics = physics  # CARLA PhysicsControl object.
        self.has_collision_sensor = has_collision_sensor # Should be True if ego.
        self.collision_sensor = None
        self.has_lane_sensor = has_lane_sensor
        self.lane_sensor = None
        self.controller = controller
        self.control_params = control_params
        self.control_agent = None

    def destroy(self):
        super().destroy()
        if self.collision_sensor is not None:
            self.collision_sensor.sensor.destroy()
        self.collision_sensor = None
        if self.lane_sensor is not None:
            self.lane_sensor.sensor.destroy()
        self.lane_sensor = None

    def create(self):
        if not super().create():
            return False
        if self.ego:
            # Keep same camera config if the camera manager exists in World.
            # cam_index = self.world.camera_manager._index \
            #     if self.world.camera_manager is not None else 0
            # cam_pos_index = self.world.camera_manager._transform_index \
            #     if self.world.camera_manager is not None else 0
            cam_index = 0
            cam_pos_index = 0
            self.world.camera_manager = CameraManager(self.actor, self.world.hud)
            self.world.camera_manager._transform_index = cam_pos_index
            self.world.camera_manager.set_sensor(cam_index, notify=False)
            self.world.camera_manager.set_transform(self.world.cam_transform)
            actor_type = get_actor_display_name(self.actor)
            self.world.hud.notification(actor_type)

        if self.has_collision_sensor:
            self.collision_sensor = CollisionSensor(self.actor,
                                                    hud=self.world.hud if self.ego else None)

        if self.has_lane_sensor:
            self.lane_sensor = LaneInvasionSensor(self.actor,
                                                  hud=self.world.hud if self.ego else None)

        if self.physics:
            self.actor.apply_physics_control(self.physics)

        if self.control_params:
            self.control_actor = self.controller(self.actor, self.control_params)
        else:
            self.control_actor = self.controller(self.actor)

        return True

    def get_control(self):
        control = self.control_actor.run_step()
        return control


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_world, hud, cam_transform=0):
        self.world = carla_world
        self.map = self.world.get_map()
        self.hud = hud
        self.entities = []
        self.ego = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.cam_transform = cam_transform
        self.ego_images = []

    def add_vehicle(self, controller, control_params=None, 
                    blueprint_filter='vehicle.*', color=None,
                    spawn=None, physics=None, has_collision_sensor=False,
                    has_lane_sensor=False, ego=False):
        '''
        Add vehicle to vehicles list with controller CONTROLLER, blueprint 
        BLUEPRINT and spawn point SPAWN. If EGO, this will be the ego vehicle 
        attached to camera manager, HUD, sensors. BLUEPRINT and SPAWN default 
        to being chosen randomly.
        '''
        vehicle = Vehicle(self, controller, control_params=control_params,
                          blueprint_filter=blueprint_filter, color=color, 
                          spawn=spawn, physics=physics,
                          has_collision_sensor=has_collision_sensor,
                          has_lane_sensor=has_lane_sensor, ego=ego)
        self.entities.append(vehicle)
        if ego:
            self.ego = vehicle
        return vehicle

    def add_pedestrian(self, blueprint_filter='walker.pedestrian.*',
                       spawn=None, color=None, ego=False):
        ped = Pedestrian(self, blueprint_filter=blueprint_filter,
                         spawn=spawn, color=color, ego=ego)
        self.entities.append(ped)
        return ped

    def add_prop(self, blueprint_filter='static.prop.*',
                       spawn=None, ego=False):
        prop = Prop(self, blueprint_filter=blueprint_filter,
                         spawn=spawn, ego=ego)
        self.entities.append(prop)
        return prop

    def restart(self):
        # Destroy all entities in world first.
        self.destroy()

        # Spawn all entities.
        for e in self.entities:
            e.create()

    def get_control_cmds(self):
        return [carla.command.ApplyVehicleControl(v.actor.id, v.get_control())
                for v in self.entities if isinstance(v, Vehicle) and v.actor is not None]

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.world.set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self): 
        for e in self.entities:
            if e is not None:
                e.destroy()

        if self.camera_manager is not None:
            self.camera_manager.sensor.destroy()
            self.camera_manager.images = []


# ==============================================================================
# -- HUD -----------------------------------------------------------------
# ==============================================================================

class HUD(object):
    def __init__(self, width, height, show_hud=False):
        self.dim = (width, height)
        self._show_info = show_hud
        if self._show_info:
            font = pygame.font.Font(pygame.font.get_default_font(), 20)
            fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
            default_font = 'ubuntumono'
            mono = default_font if default_font in fonts else fonts[0]
            mono = pygame.font.match_font(mono)
            self._font_mono = pygame.font.Font(mono, 14)
            self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if not self._show_info:
            return
        if world.ego.actor is None:
            # Ego not spawned yet
            return
        t = world.ego.actor.get_transform()
        v = world.ego.actor.get_velocity()
        c = world.ego.actor.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.ego.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16d FPS' % self.server_fps,
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.ego.actor, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear),
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x)
                        for x in vehicles if x.id != world.ego.actor.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        # self._notifications.tick(world, clock)
        pass

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):        
        # self._notifications.set_text(text, seconds=seconds)
        pass

    def error(self, text):
        # self._notifications.set_text('Error: %s' % text, (255, 0, 0))
        pass

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item: # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        # self._notifications.render(display)
        pass


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud=None):
        self.sensor = None
        self._history = []
        self._parent = parent_actor
        self._parent_mass = parent_actor.get_physics_control().mass
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_speeds(self):
        '''Convert collision intensities from momentem (kg*m/s) to speed (km/h).'''
        return [(c[0], 3.6 * c[1] / self._parent_mass)
                         for c in self._history]

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self._history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        if self._hud:
            self._hud.notification('Collision with %r, id = %d'
                                   % (actor_type, event.other_actor.id))
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self._history.append((event.frame, intensity))
        if len(self._history) > 4000:
            self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud=None):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        self._history = []
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
        if self._hud:
            self._hud.notification('Crossed line %s' % ' and '.join(text))
        self._history.append((event.frame, 1))


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self.images = []
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            item.append(bp)
        self._index = None

    def toggle_camera(self):
        set_transform((self._transform_index + 1) % len(self._camera_transforms))

    def set_transform(self, idx):
        self._transform_index = idx
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame)
        self.images.append(image)
