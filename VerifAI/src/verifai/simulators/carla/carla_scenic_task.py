from verifai.simulators.carla.client_carla import *
from verifai.simulators.carla.carla_world import *
from verifai.simulators.carla.carla_task import *

import numpy as np
from dotmap import DotMap
import time
import carla
from carla import Transform, Rotation, Location

from verifai.simulators.carla.agents.brake_agent import *
from verifai.simulators.carla.agents.pid_agent import *

WEATHER_PARAMS = ['cloudiness', 'precipitation', 'precipitation_deposits',
                  'wind_intensity', 'sun_azimuth_angle', 'sun_altitude_angle']

AGENTS = {'BrakeAgent': BrakeAgent, 'PIDAgent': PIDAgent}

class scenic_sampler_task(carla_task):
    def __init__(self,
                 n_sim_steps=100,
                 display_dim=(1280,720),
                 carla_host='127.0.0.1',
                 carla_port=2000,
                 carla_timeout=4.0,
                 world_map='Town01',
                 cam_transform=0):
        super().__init__(
            n_sim_steps=n_sim_steps,
            display_dim=display_dim,
            carla_host=carla_host,
            carla_port=carla_port,
            carla_timeout=carla_timeout,
            world_map=world_map,
            cam_transform=cam_transform
        )

    def snap_to_ground(self, location):
        '''Mutates @location to have the same z-coordinate as the nearest waypoint.'''
        waypoint = self.world.world.get_map().get_waypoint(location)
        location.z = waypoint.transform.location.z + 1
        return location

    def use_sample(self, sample):
        print('USE_SAMPLE Sample:', sample)
        weather = carla.WeatherParameters(**{k: sample.params._asdict()[k] for k in WEATHER_PARAMS})
        self.world.world.set_weather(weather)
        for obj in sample.objects:
            spawn = Transform(self.snap_to_ground(Location(x=obj.position[0],
                                                           y=-obj.position[1], z=1)),
                              Rotation(yaw=-obj.heading * 180 / math.pi - 90))
            attrs = dict()
            if 'color' in obj._fields:
                color = str(int(obj.color.r * 255)) + ',' \
                    + str(int(obj.color.g * 255)) + ',' + str(int(obj.color.b * 255))
                attrs['color'] = color
            if 'blueprint' in obj._fields:
                attrs['blueprint_filter'] = obj.blueprint
            agent = BrakeAgent
            if 'agent' in obj._fields:
                agent = AGENTS[obj.agent]
            if obj.type in ['Vehicle', 'Car', 'Truck', 'Bicycle', 'Motorcycle']:
                self.world.add_vehicle(agent,
                                       spawn=spawn,
                                       has_collision_sensor=False,
                                       has_lane_sensor=False,
                                       ego=obj is sample.objects[0],
                                       **attrs)
            elif obj.type == 'Pedestrian':
                self.world.add_pedestrian(spawn=spawn, **attrs)
            elif obj.type in ['Prop', 'Trash', 'Cone']:
                self.world.add_prop(spawn=spawn, **attrs)
            else:
                print('Unsupported object type:', obj.type)

    def trajectory_definition(self):
        # To save image each iteration:
        # self.world.camera_manager.images[-1].save_to_disk(str(int(time.time())))
        return {}
