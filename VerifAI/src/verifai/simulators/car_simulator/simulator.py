import pyglet
import pyglet.gl as gl
import pyglet.graphics as graphics
import numpy as np
from typing import NamedTuple
import os

class simulator_world(NamedTuple):
     lanes:list = []
     cars:list = []
     objects:list = []

class simulator:
    def __init__(self, dt =0.1, fullscreen=False, name='Simulator', magnify=1., iters=100, sprite_scale= 0.15/600.,
                 window_size= 600):
        self.dt = dt
        self.fullscreen = fullscreen
        self.name = name
        self.magnify = magnify
        self.max_iters = iters
        self.iter = 0
        self.window_size = window_size

        working_dir = os.path.dirname(os.path.realpath(__file__))
        pyglet.resource.path = [os.path.join(working_dir, 'imgs')]
        pyglet.resource.reindex()

        def centered_image(filename):
            img = pyglet.resource.image(filename)
            img.anchor_x = img.width / 2.
            img.anchor_y = img.height / 2.
            return img

        def car_sprite(color, scale=sprite_scale):
            sprite = pyglet.sprite.Sprite(centered_image('car-{}.png'.format(color)), subpixel=True)
            sprite.scale = scale
            return sprite

        def object_sprite(name, scale=sprite_scale):
            sprite = pyglet.sprite.Sprite(centered_image('{}.png'.format(name)), subpixel=True)
            sprite.scale = scale
            return sprite

        self.sprites = {c: car_sprite(c) for c in ['red', 'yellow', 'purple', 'white', 'orange', 'gray', 'blue']}
        self.obj_sprites = {c: object_sprite(c) for c in ['cone', 'firetruck']}

    def init_simulator(self, world, task_name):
        self.event_loop = pyglet.app.EventLoop()
        self.window = pyglet.window.Window(self.window_size, self.window_size,
                                           fullscreen=self.fullscreen, caption=self.name)
        self.background = pyglet.resource.texture('grass.png')
        self.task_name = task_name
        self.label = pyglet.text.Label(
            task_name,
            font_name='Times New Roman',
            font_size=24,
            x=30, y=self.window.height - 30,
            anchor_x='left', anchor_y='top'
        )
        self.window.on_draw = self.on_draw
        self.world = world
        self.viewpoint_car = None if len(self.world.cars) == 0 else self.world.cars[0]

    def draw_lane_surface(self, lane):
        gl.glColor3f(0.4, 0.4, 0.4)
        W = 1000
        graphics.draw(4, gl.GL_QUAD_STRIP, ('v2f',
                                            np.hstack([lane.p - lane.m * W - 0.5 * lane.w * lane.n,
                                                       lane.p - lane.m * W + 0.5 * lane.w * lane.n,
                                                       lane.q + lane.m * W - 0.5 * lane.w * lane.n,
                                                       lane.q + lane.m * W + 0.5 * lane.w * lane.n])
                                            ))

    def draw_lane_lines(self, lane):
        gl.glColor3f(1., 1., 1.)
        W = 1000
        graphics.draw(4, gl.GL_LINES, ('v2f',
                                       np.hstack([lane.p - lane.m * W - 0.5 * lane.w * lane.n,
                                                  lane.p + lane.m * W - 0.5 * lane.w * lane.n,
                                                  lane.p - lane.m * W + 0.5 * lane.w * lane.n,
                                                  lane.p + lane.m * W + 0.5 * lane.w * lane.n])
                                       ))

    def draw_car(self, x, color='yellow', opacity=255):
        sprite = self.sprites[color]
        sprite.x, sprite.y = x[0], x[1]
        sprite.rotation = -x[3] * 180. / np.pi
        sprite.opacity = opacity
        sprite.draw()

    def draw_object(self, obj):
        sprite = self.obj_sprites[obj.name]
        sprite.x, sprite.y = obj.x[0], obj.x[1]
        sprite.rotation = obj.x[2] if len(obj.x) >= 3 else 0.
        sprite.draw()

    def center(self):
        if self.viewpoint_car is not None:
            return np.array(self.viewpoint_car.trajectory[-1][0:2])
        else:
            return np.array([0., 0.])

    def camera(self):
        o = self.center()
        gl.glOrtho(o[0] - 1. / self.magnify, o[0] + 1. / self.magnify, o[1] - 1. / self.magnify,
                   o[1] + 1. / self.magnify, -1., 1.)

    def reset(self):
        for car in self.world.cars:
            car.reset()

    def on_draw(self):
        self.window.clear()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        self.camera()
        gl.glEnable(self.background.target)
        gl.glEnable(gl.GL_BLEND)
        gl.glBindTexture(self.background.target, self.background.id)
        W = 10000.
        graphics.draw(4, gl.GL_QUADS,
                      ('v2f', (-W, -W, W, -W, W, W, -W, W)),
                      ('t2f', (0., 0., W * 5., 0., W * 5., W * 5., 0., W * 5.))
                      )
        gl.glDisable(self.background.target)
        for lane in self.world.lanes:
            self.draw_lane_surface(lane)
            self.draw_lane_lines(lane)
        for obj in self.world.objects:
            self.draw_object(obj)
        for car in self.world.cars:
            self.draw_car(car.trajectory[-1], car.color)
        gl.glPopMatrix()

        self.label.text = self.task_name
        self.label.draw()
        self.iter +=1
        if self.iter%10 == 0:
            print('Iterations: ', self.iter)
        if self.iter == self.max_iters:
            self.event_loop.exit()

    def simulation_step(self, _=None):
        for car in self.world.cars:
            car.step()

    def run(self, reset=True):
        if reset:
            self.reset()
        pyglet.clock.schedule_interval(self.simulation_step, self.dt)
        self.event_loop.run()

    def exit_simulation(self):
        self.window.close()









