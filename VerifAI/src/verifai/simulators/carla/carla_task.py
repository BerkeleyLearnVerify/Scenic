import numpy as np
import pygame
import carla
from .carla_world import *


class carla_task():
    def __init__(self,
                 n_sim_steps=500,
                 display_dim=(1280,720),
                 carla_host='127.0.0.1',
                 carla_port=2000,
                 carla_timeout=4.0,
                 world_map='Town03',
                 cam_transform=0):
        self.n_sim_steps = n_sim_steps
        self.display_dim = display_dim
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(carla_timeout)
        self.clock = pygame.time.Clock()
        self.world_map = world_map
        self.timestep = 0
        self.cam_transform = cam_transform
        print("[carla_task] Finished initializing carla task.")

    def step_world(self):
        # print("[carla_task] Stepping world.")
        self.world.world.wait_for_tick()
        # print("[carla_task] Before tick")
        self.world.tick(self.clock)
        # print("[carla_task] After tick")
        self.world.render(self.display)
        # print("[carla_task] After render")
        self.client.apply_batch(self.world.get_control_cmds())
        # print("[carla_task] Batch applied.")
        pygame.display.flip()


    def run_task(self, sample):
        try:
            pygame.init()
            pygame.font.init()
            self.hud = HUD(*self.display_dim)
            self.display = pygame.display.set_mode(
                self.display_dim,
                pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            # print("[carla_task] Setting up world.")
            if self.client.get_world().get_map().name == self.world_map:
                self.world = World(self.client.get_world(), self.hud, 
                        cam_transform=self.cam_transform)
            else:
                self.world = World(self.client.load_world(self.world_map), 
                        self.hud, cam_transform=self.cam_transform)
            # print("[carla_task] World setup complete.")
            self.use_sample(sample)
            self.world.restart()
            self.timestep = 0
            while self.timestep < self.n_sim_steps:
                self.step_world()
                self.timestep += 1
            traj = self.trajectory_definition()
        finally:
            self.world.destroy()
            pygame.quit()
        return traj


    def use_sample(self, sample):
        raise NotImplementedError('Method should be implemented in subclass.')


    def trajectory_definition(self):
        raise NotImplementedError('Method should be implemented in subclass.')
